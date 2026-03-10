import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import Float32 # Add to imports
from std_msgs.msg import String

class WaypointNode(Node):

    def __init__(self):
        super().__init__('waypoint_node')

        # Ask user for goal
        self.goal_x = float(input("Enter goal X: "))
        self.goal_y = float(input("Enter goal Y: "))
        self.goal_z = float(input("Enter goal Z: "))

        # Publisher
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')   
        # QoS for MAVROS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to position
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile)

        # Subscribe to state
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)

        self.current_state = State()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.offboard_set = False
        self.armed = False
        self.mission_started = False
        self.landing_initiated = False 
        self.phase = "WAIT_CONNECTION"
        self.setpoint_counter = 0
        
        self.min_distance = 999.0
        
        self.timer = self.create_timer(0.05, self.main_loop)
        # QoS for MAVROS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/world/baylands/model/x500_depth_0/link/lidar_link/sensor/gpu_lidar_3d/scan',
            self.scan_callback,
            10
        )
        self.ai_status = 0
        self.ai_sub = self.create_subscription(Int32, '/drone/ai_status', self.ai_callback, 10)
        self.alt_pub = self.create_publisher(Float32, '/drone/current_altitude', 10)
        
        self.vla_sub = self.create_subscription(String, '/drone/vla_decision', self.vla_cb, 10)
        self.vla_action = "STRAIGHT"
    
    
    def vla_cb(self, msg):
        self.vla_action = msg.data    
    def ai_callback(self, msg):
            self.ai_status = msg.data

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        #publish altitude
        alt_msg=Float32()
        alt_msg.data=self.current_z
        self.alt_pub.publish(alt_msg)

    def set_mode(self, mode):
        client = self.create_client(SetMode, '/mavros/set_mode')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        request = SetMode.Request()
        request.custom_mode = mode
        client.call_async(request)

    def arm(self):
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        request = CommandBool.Request()
        request.value = True
        client.call_async(request)
        
    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > 0.5]
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            # FORCE PRINT TO TERMINAL
            print(f"--- LIDAR DISTANCE: {self.min_distance:.2f}m ---")
            self.get_logger().info(f"Min distance: {self.min_distance:.2f}")
            
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (x, y, z, w)
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]            

    def main_loop(self):

        # PHASE 1 — Wait for connection
        if self.phase == "WAIT_CONNECTION":
            if self.current_state.connected:
                self.get_logger().info("Connected to FCU")
                self.phase = "SEND_SETPOINTS"
            return

        # PHASE 2 — Send dummy setpoints
        if self.phase == "SEND_SETPOINTS":
            hold_msg = PoseStamped()
            hold_msg.header.frame_id = "map"
            hold_msg.pose.position.x = self.current_x
            hold_msg.pose.position.y = self.current_y
            hold_msg.pose.position.z = 2.0

            self.publisher.publish(hold_msg)
            self.setpoint_counter += 1

            if self.setpoint_counter > 40:
                self.phase = "SET_OFFBOARD"
            return

        # PHASE 3 — Set OFFBOARD
        if self.phase == "SET_OFFBOARD":
            self.get_logger().info("Setting OFFBOARD mode")
            self.set_mode("OFFBOARD")
            self.phase = "ARM"
            return

        # PHASE 4 — Arm
        if self.phase == "ARM":
            if self.current_state.mode == "OFFBOARD":
                self.get_logger().info("Arming...")
                self.arm()
                self.phase = "TAKEOFF"
            return

        # PHASE 5 — Wait until armed
        if self.phase == "TAKEOFF":
            if self.current_state.armed:
                self.get_logger().info("Armed! Starting navigation")
                self.phase = "NAVIGATE"
            return
        # PHASE 6 — Navigation
        if self.phase == "NAVIGATE":
            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            dz = self.goal_z - self.current_z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < 1.0:
                self.get_logger().info("Goal reached! Switching to LANDING phase.")
                self.phase = "LANDING"
                return
         
            nav_msg = PoseStamped()
            nav_msg.header.frame_id = "map"

            # --- FIX: Initialize variables with current position as a fallback ---
            target_x, target_y, target_z = self.current_x, self.current_y, self.current_z

            # STATE 1: EMERGENCY STOP
            if self.ai_status == 2:
                self.get_logger().warn("AI AGENT: EMERGENCY STOP ACTIVATED")
                # Positions stay as current_x/y/z initialized above
        
            elif self.ai_status == 0:
                if "LEFT" in self.vla_action:
                    self.get_logger().info("VLA Suggestion: Moving LEFT")
                    target_x = self.current_x + 1.0
                    target_y = self.current_y + 1.5
                    target_z = self.goal_z
                elif "RIGHT" in self.vla_action:
                    self.get_logger().info("VLA Suggestion: Moving RIGHT")
                    target_x = self.current_x + 1.0
                    target_y = self.current_y - 1.5
                    target_z = self.goal_z
                else:
                    # Normal Mission Path (Straight to goal)
                    # We use a unit vector to move 1 meter at a time
                    if distance > 0.1:
                        direction_x = dx / distance
                        direction_y = dy / distance
                        target_x = self.current_x + direction_x
                        target_y = self.current_y + direction_y
                    target_z = self.goal_z

            # --- STEP 2: CALCULATE YAW TOWARD TARGET ---
            look_dx = target_x - self.current_x
            look_dy = target_y - self.current_y
            
            if abs(look_dx) < 0.01 and abs(look_dy) < 0.01:
                target_yaw = 0.0 
            else:
                target_yaw = math.atan2(look_dy, look_dx)
            
            q = self.get_quaternion_from_euler(0, 0, target_yaw)

            # --- STEP 3: CONSTRUCT MESSAGE ---
            nav_msg.pose.position.x = target_x
            nav_msg.pose.position.y = target_y
            nav_msg.pose.position.z = target_z
            nav_msg.pose.orientation.x = q[0]
            nav_msg.pose.orientation.y = q[1]
            nav_msg.pose.orientation.z = q[2]
            nav_msg.pose.orientation.w = q[3]
            
            self.publisher.publish(nav_msg)
            return
            
  
            self.get_logger().info(f"Distance: {distance:.2f}")

            
        # PHASE 7 — Dedicated Landing Phase
        if self.phase == "LANDING":
            self.get_logger().info("Initiating Auto-Land...")
            self.set_mode("AUTO.LAND")
            self.phase = "WAIT_FOR_TOUCHDOWN"
            return

        if self.phase == "WAIT_FOR_TOUCHDOWN":
            if self.current_z < 0.3:
                self.get_logger().info("Touchdown! Disarming.")
                # Add your disarm call here
                self.phase = "FINISHED"
            return

            # If landing started → descend gradually
            if self.landing_initiated:

                land_msg = PoseStamped()
                land_msg.header.frame_id = "map"
                land_msg.pose.position.x = self.goal_x
                land_msg.pose.position.y = self.goal_y

                # Gradual descent
                #new_z = self.current_z - 0.30
                #if new_z < 0.1:
                    #new_z = 0.0

                land_msg.pose.position.z = 0.0
                #We must also keep the Yaw fixed so it doesn't spin while landing
                target_yaw = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                q = self.get_quaternion_from_euler(0, 0, target_yaw)
                land_msg.pose.orientation.x, land_msg.pose.orientation.y, land_msg.pose.orientation.z, land_msg.pose.orientation.w = q
                self.publisher.publish(land_msg)

                # Stop after touchdown
                if self.current_z < 0.2:
                    self.get_logger().info("Touchdown detected... Waiting 4 seconds to settle.")
                    time.sleep(2.0)
                    
     
                    request = CommandBool.Request()
                    request.value = False
                    self.arm_client.call_async(request)
                    self.phase = "LANDED"
                return

            step_size = 1.0
            if distance < 0.01:
                return
            direction_x = dx / distance
            direction_y = dy / distance

            # Calculate Yaw toward target
            target_yaw = math.atan2(dy, dx)
            q = self.get_quaternion_from_euler(0, 0, target_yaw)

            nav_msg = PoseStamped()
            nav_msg.header.frame_id = "map"
            nav_msg.pose.position.x = self.current_x + direction_x * step_size
            nav_msg.pose.position.y = self.current_y + direction_y * step_size
            nav_msg.pose.position.z = self.goal_z

            # Set the orientation so the camera faces the destination
            nav_msg.pose.orientation.x = q[0]
            nav_msg.pose.orientation.y = q[1]
            nav_msg.pose.orientation.z = q[2]
            nav_msg.pose.orientation.w = q[3]

            self.publisher.publish(nav_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
