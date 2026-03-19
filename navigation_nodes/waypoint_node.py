import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import time
from gz.transport13 import Node as GzNode
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
import struct
from std_msgs.msg import Int32, Float32, String
import numpy as np
from drone_nav.astar_planner import AStarPlanner

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')

        # 1. Inputs & Config
        self.goal_x = float(input("Enter goal X: "))
        self.goal_y = float(input("Enter goal Y: "))
        self.goal_z = float(input("Enter goal Z: "))
        
        self.grid_offset = (40.0, 40.0) 
        self.planner = AStarPlanner(grid_size=(100, 100), resolution=1.0, grid_offset=self.grid_offset)
        self.occupancy_grid = np.zeros((100, 100)) 
        self.current_path = []

        # 2. State Variables
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_orientation = None
        self.is_armed = False
        self.current_state = State()
        self.phase = "WAIT_CONNECTION"
        self.setpoint_counter = 0
        
        self.min_distance = 10.0
        self.critical_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        # 3. ROS Communications
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        
        # 4. Gazebo Transport
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            PointCloudPacked,
            "/world/default/model/x500_depth_0/link/lidar_link/sensor/gpu_lidar_3d/scan/points",
            self.gz_scan_callback
        )

        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("Node Initialized. Waiting for local position...")

    def state_callback(self, msg):
        self.current_state = msg
        self.is_armed = msg.armed

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        self.current_orientation = msg.pose.orientation

    def get_yaw(self):
        if self.current_orientation is None: return 0.0
        q = self.current_orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def gz_scan_callback(self, msg):
        print("GZ CALLBACK RUNNING")
        yaw = self.get_yaw()
        data = msg.data
        step = msg.point_step
        
        # Temporary lists to store distances for each direction
        front_points = []
        left_points = []
        right_points = []

        # Decay old obstacles slightly to keep map fresh
        self.occupancy_grid *= 0.95 

        for i in range(0, len(data), step * 2): # Step*2 to save CPU
            x_l, y_l, z_l = struct.unpack_from('fff', data, i)
            
            # Filter ground and far noise
            if z_l < -0.5 or z_l > 1.0: continue
            dist = math.sqrt(x_l**2 + y_l**2)
            if dist > 8.0 or dist < 0.4: continue

            # Map to World Coordinates
            world_obs_x = self.current_x + (x_l * math.cos(yaw) - y_l * math.sin(yaw))
            world_obs_y = self.current_y + (x_l * math.sin(yaw) + y_l * math.cos(yaw))

            # Mark Grid
            gx = int((world_obs_x + self.grid_offset[0]))
            gy = int((world_obs_y + self.grid_offset[1]))
            if 0 <= gx < 100 and 0 <= gy < 100:
                self.occupancy_grid[gx][gy] = 1.0
            
            # FRONT: Objects ahead within 1.0m width corridor
            if x_l > 0 and abs(y_l) < 1.0:
                front_points.append(dist)
            
            # LEFT: Objects to the left
            if y_l > 1.0:
                left_points.append(dist)
                
            # RIGHT: Objects to the right
            if y_l < -1.0:
                right_points.append(dist)

        # Use median for stability, or default to 10.0 if no points found
        self.min_distance = np.median(front_points) if front_points else 10.0
        self.critical_dist = min(front_points) if front_points else 10.0
        self.left_dist = np.median(left_points) if left_points else 10.0
        self.right_dist = np.median(right_points) if right_points else 10.0

        # 4. PRINT TO TERMINAL
        self.get_logger().info(f"Lidar -> FRONT: {self.min_distance:.2f}m | LEFT: {self.left_dist:.2f}m | RIGHT: {self.right_dist:.2f}m")


    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        
        # Simple yaw to quaternion
        msg.pose.orientation.z = math.sin(yaw/2)
        msg.pose.orientation.w = math.cos(yaw/2)
        self.publisher.publish(msg)

    def main_loop(self):
        if self.current_x is None: return
        # phase-1 : waiting connection
        # Connection & Arming Logic
        if self.phase == "WAIT_CONNECTION":
            if self.current_state.connected:
                self.get_logger().info("Connected to FCU")
                self.phase = "STARTUP_STREAM"
            return
        #phase-2 : setpoint
        if self.phase == "STARTUP_STREAM":
            self.publish_setpoint(self.current_x, self.current_y, 2.0)
            self.setpoint_counter += 1
            if self.setpoint_counter > 20: self.phase = "SET_OFFBOARD"
            return
        #phase-3 : set offboard
        if self.phase == "SET_OFFBOARD":
            self.get_logger().info("Setting OFFBOARD mode")
            self.set_mode("OFFBOARD")
            self.phase = "ARM"
            return
        #phase-4 : Arming
        if self.phase == "ARM":
            if self.current_state.mode == "OFFBOARD":
                self.get_logger().info("Arming...")
                req = CommandBool.Request(); req.value = True
                self.arm_client.call_async(req)
                self.phase = "TAKEOFF"
            return
        #phase-5 : Takeoff
        if self.phase == "TAKEOFF":
            self.publish_setpoint(self.current_x, self.current_y, self.goal_z)
            if abs(self.current_z - self.goal_z) < 0.3:
                self.get_logger().info("Altitude reached. Switching to NAVIGATE.")
                self.phase = "NAVIGATE"
            return
        #phase-6 : Navigation
        # --- NAVIGATION PHASE ---
        if self.phase == "NAVIGATE":
            dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
            if dist_to_goal < 0.8:
                self.get_logger().info("GOAL REACHED! Switching to Landing.")
                self.phase = "LANDING"
                return

            # Replan if path is empty or blocked
            if not self.current_path or self.critical_dist < 2.5:
                start = (self.current_x, self.current_y)
                goal = (self.goal_x, self.goal_y)
                self.get_logger().info(f"A* Planning: {start} -> {goal}")
                self.current_path = self.planner.plan(start, goal, self.occupancy_grid)
                if not self.current_path:
                    self.get_logger().warn("Path blocked! Hovering...")
                    self.publish_setpoint(self.current_x, self.current_y, self.goal_z)
                    return

            target_x, target_y = self.current_path[0]
            
            # SPEED CONTROL
            max_step = 0.6
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist > max_step:
                target_x = self.current_x + (dx/dist)*max_step
                target_y = self.current_y + (dy/dist)*max_step

            # Popping reached waypoints
            if dist < 0.8:
                self.current_path.pop(0)

            move_yaw = math.atan2(dy, dx)
            self.publish_setpoint(target_x, target_y, self.goal_z, move_yaw)
        #phase-7 : Landing
        if self.phase == "LANDING":
            self.set_mode("AUTO.LAND")
            self.phase = "FINISHED"

    def set_mode(self, mode):
        client = self.create_client(SetMode, '/mavros/set_mode')
        req = SetMode.Request(); req.custom_mode = mode
        client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
