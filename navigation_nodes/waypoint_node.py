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
import cv2
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point

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
        self.setpoint_counter = 0
        
        self.min_distance = 10.0
        self.critical_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0
        self.phase = "WAIT_CONNECTION"  # "IDLE", "NAVIGATE"
        self.latest_ai_decision = "PATH_SAFE"  # "PATH_SAFE", "STOP"
        self.stop_distance_threshold = 3.0
        self.hover_pos = None
        self.stop_cooldown = 0
        # 3. ROS Communications
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        # waypoint_node.py
        self.path_pub = self.create_publisher(Path, '/drone/visual_path', 10)
        self.goal_pub = self.create_publisher(Marker, '/drone/visual_goal', 10)
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
        # Inside WaypointNode.__init__
        self.latest_ai_decision = "PATH_SAFE" 
        self.create_subscription(String, '/drone/vla_decision', self.vla_callback, 10)
    
    def show_debug_map(self):
        """Creates a standalone visualization window without using ROS 2 bridge."""
        # 1. Create a 500x500 base image (scaling the 100x100 grid by 5)
        display_map = np.zeros((500, 500, 3), dtype=np.uint8)

        # 2. Draw Occupancy Grid (Red for Obstacles)
        # We find indices where grid is 1.0 (obstacle)
        obs_idx = np.where(self.occupancy_grid > 0.5)
        for i, j in zip(obs_idx[0], obs_idx[1]):
            cv2.rectangle(display_map, (j*5, i*5), (j*5+5, i*5+5), (0, 0, 255), -1)

        # 3. Draw A* Path (Blue Line)
        if self.current_path:
            for k in range(len(self.current_path) - 1):
                # Convert world coords to grid index
                p1 = (int((self.current_path[k][1] + self.grid_offset[1]) * 5), 
                      int((self.current_path[k][0] + self.grid_offset[0]) * 5))
                p2 = (int((self.current_path[k+1][1] + self.grid_offset[1]) * 5), 
                      int((self.current_path[k+1][0] + self.grid_offset[0]) * 5))
                cv2.line(display_map, p1, p2, (255, 0, 0), 2)

        # 4. Draw the Drone (Green Circle)
        drone_gx = int((self.current_x + self.grid_offset[0]) * 5)
        drone_gy = int((self.current_y + self.grid_offset[1]) * 5)
        cv2.circle(display_map, (drone_gy, drone_gx), 8, (0, 255, 0), -1)

        # 5. Show Window
        cv2.imshow("A* Navigation Debug", display_map)
        cv2.waitKey(1)

    def vla_callback(self, msg):
        self.latest_ai_decision = msg.data.upper() # Standardize to upper case

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))    

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
        # --- PHASE-6: NAVIGATION (Consolidated Logic) ---
        if self.phase == "NAVIGATE":
            # 1. Goal Check
            dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
            if dist_to_goal < 0.8:
                self.get_logger().info("GOAL REACHED! Switching to Landing.")
                self.phase = "LANDING"
                return

            # 3. PATH PLANNING: A* Path Generation
            if not self.current_path or self.critical_dist < 2.5:
                self.current_path = self.planner.plan((self.current_x, self.current_y), 
                                                      (self.goal_x, self.goal_y), 
                                                      self.occupancy_grid)

            if not self.current_path:
                self.get_logger().warn("No path available. Hovering...")
                self.publish_setpoint(self.current_x, self.current_y, self.goal_z)
                return
            
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for waypoint in self.current_path:
                pose = PoseStamped()
                pose.pose.position.x = float(waypoint[0])
                pose.pose.position.y = float(waypoint[1])
                pose.pose.position.z = self.goal_z
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)

            if self.current_path:
                target_x, target_y = self.current_path[0]

                # PATH-AWARE CHECK: Only stop if we are heading toward the obstacle
                target_yaw = math.atan2(target_y - self.current_y, target_x - self.current_x)
                angle_diff = abs(self.normalize_angle(target_yaw - self.get_yaw()))

                # 1. Logic for "Sticky" STOP command
                if "STOP" in self.latest_ai_decision and angle_diff < 0.8:
                    self.stop_cooldown = 10 # Stay stopped for 10 cycles (1.0 second)
    
                # 2. Check if we are in the "Sticky" stop state
                if self.stop_cooldown > 0:
                    if self.hover_pos is None:
                        self.hover_pos = (self.current_x, self.current_y)
        
                    self.publish_setpoint(self.hover_pos[0], self.hover_pos[1], self.goal_z)
                    self.stop_cooldown -= 1 # Countdown every 0.1s cycle
                    return
                else:
                    # Only clear hover once the cooldown is totally finished
                    self.hover_pos = None

            # 5. APPLY REACTIVE OFFSETS (LEFT/RIGHT)
            offset_x, offset_y = 0.0, 0.0
            if "LEFT" in self.latest_ai_decision:
                yaw = self.get_yaw()
                offset_x = -1.5 * math.sin(yaw) 
                offset_y = 1.5 * math.cos(yaw)
                self.get_logger().info("AI REACTING: Veering Left.")
            elif "RIGHT" in self.latest_ai_decision:
                yaw = self.get_yaw()
                offset_x = 1.5 * math.sin(yaw)
                offset_y = -1.5 * math.cos(yaw)
                self.get_logger().info("AI REACTING: Veering Right.")

            final_target_x = target_x + offset_x
            final_target_y = target_y + offset_y

            # 6. SPEED CONTROL & SMOOTHING
            # Apply max_step to the FINAL modified target
            max_step = 0.6
            dx = final_target_x - self.current_x
            dy = final_target_y - self.current_y
            dist_to_target = math.sqrt(dx**2 + dy**2)

            if dist_to_target > max_step:
                final_target_x = self.current_x + (dx/dist_to_target) * max_step
                final_target_y = self.current_y + (dy/dist_to_target) * max_step
            self.show_debug_map()
            # 7. WAYPOINT PROGRESSION
            # Check distance to the ORIGINAL A* point to see if we reached it
            base_dist = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
            if base_dist < 0.8:
                self.current_path.pop(0)

            # 8. FINAL PUBLISH (Exactly once per loop)
            move_yaw = math.atan2(dy, dx)
            self.publish_setpoint(final_target_x, final_target_y, self.goal_z, move_yaw)
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
