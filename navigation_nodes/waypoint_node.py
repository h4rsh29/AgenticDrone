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
import threading

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')

        # 1. Inputs & Config
        self.goal_x = 0.0 
        self.goal_y = 0.0
        self.goal_z = 2.5
        self.phase = "WAIT_CONNECTION"
        self.mission_active = False
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
        self.takeoff_complete = False
        # 3D Grid for Bridge Safety
        self.voxel_grid = np.zeros((100, 100, 20)) 
        self.v_res = 1.0 # 1-meter resolution
        self.v_offset = (50, 50, 0) # Center the grid
        self.min_distance = 10.0
        self.critical_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0 # 
        self.latest_ai_decision = "PATH_SAFE"  # "PATH_SAFE", "STOP"
        self.stop_distance_threshold = 3.0
        self.hover_pos = None
        self.stop_cooldown = 0
        self.current_inspection_radius = 7.0 # Start a bit further back
        self.target_inspection_dist = 5.0    # Maintain exactly 5m from surface

        self.search_pattern = []
        self.current_mission = "nav"

        self.bridge_y = -35.0
        self.bridge_z = 5.0
        
        # Waypoints now fly ALONG the 20m bridge (X-axis)
        # Side A (North)
        side_a = [(-12.0, -28.0, 5.0), (-6.0, -28.0, 5.0), (0.0, -28.0, 5.0), (6.0, -28.0, 5.0), (12.0, -28.0, 5.0)]
        # Transition around the edge
        transition = [(12.0, -42.0, 5.0)]
        # Side B (South)
        side_b = [(12.0, -42.0, 5.0), (6.0, -42.0, 5.0), (0.0, -42.0, 5.0), (-6.0, -42.0, 5.0), (-12.0, -42.0, 5.0)]

        self.inspection_waypoints = side_a + transition + side_b
        self.return_to_point_idx = 2 # Index 2 is the bridge center (0, -30)
        self.inspection_sequence_finished = False
        self.waypoint_idx = 0
        self.travel_speed = 1.5      # 1.5 m/s for getting to the target
        self.inspection_speed = 0.4  # 0.3 m/s for steady, high-quality scanning
        self.current_max_speed = self.travel_speed
        self.takeoff_complete = False

        self.cmd_pub = self.create_publisher(String, '/drone/user_command', 10)
        self.create_subscription(Point, '/drone/new_goal', self.new_goal_cb, 10)
        self.create_subscription(String, '/drone/vla_decision', self.vla_callback, 10)
        self.status_pub = self.create_publisher(String, '/drone/pilot_status', 10)
        # 3. ROS Communications
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.create_subscription(String, '/drone/active_mission', self.mission_type_cb, 10)
        # waypoint_node.py
        self.path_pub = self.create_publisher(Path, '/drone/visual_path', 10)
        self.goal_pub = self.create_publisher(Marker, '/drone/visual_goal', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.fault_log_file = "drone_mission_log.txt"
        self.get_logger().info("Pilot: Online. Waiting for a Mission Command...")
        self.create_subscription(String, '/drone/vla_description', self.log_callback, 10)
        # 4. Gazebo Transport
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            PointCloudPacked,
            "/world/default/model/x500_depth_0/link/lidar_link/sensor/gpu_lidar_3d/scan/points",
            self.gz_scan_callback
        )

        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("Node Initialized. Waiting for local position...")

        # 3. Start the "Chat" interface in a separate thread
        threading.Thread(target=self.terminal_input_loop, daemon=True).start()
        
        self.get_logger().info("Pilot Online. Type your mission here anytime!")

    def terminal_input_loop(self):
        """Allows you to type commands without stopping the flight logic"""
        while rclpy.ok():
            user_input = input("\n[Drone Commander] > ")
            if user_input.strip():
                msg = String()
                msg.data = user_input
                self.cmd_pub.publish(msg)
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
        if self.current_x is None or self.current_y is None:
            return # Wait for MAVROS to give us a position first
        #print("GZ CALLBACK RUNNING")
        yaw = self.get_yaw()
        data = msg.data
        step = msg.point_step
        
        # Temporary lists to store distances for each direction
        front_points = []
        left_points = []
        right_points = []

        # Decay old obstacles slightly to keep map fresh
        self.occupancy_grid *= 0.995 

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
            
            if 0 <= gx < 100 and 0 <= gy < 100:
                self.occupancy_grid[gx][gy] = 1.0
                # Mark 3D Voxel if it's part of the bridge
                if z_l > 0.5: # Obstacles above the drone
                    # CORRECTED: Use World Z (Drone Alt + LiDAR local Z)
                    world_obs_z = self.current_z + z_l 
                    vz = int(world_obs_z / self.v_res)
                    if 0 <= vz < 20:
                        self.voxel_grid[gx, gy, vz] = 1

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
        #self.get_logger().info(f"Lidar -> FRONT: {self.min_distance:.2f}m | LEFT: {self.left_dist:.2f}m | RIGHT: {self.right_dist:.2f}m")


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

    def mission_type_cb(self, msg):
        self.current_mission = msg.data.lower()
        self.get_logger().info(f"PILOT: Mission mode updated to {self.current_mission}")  
        # CRITICAL FIX: Reactivate the pilot so it can process the new mission
        self.mission_active = True 
        
        # Reset the orbit index if starting a new inspection
        if self.current_mission == "inspection":
            self.search_idx = 0.0

        # NEW: Ensure pattern is built when mode switches, even if coordinates arrived first
        if self.current_mission == "surveillance" and self.goal_x is not None:
            self.search_pattern = [
                (self.goal_x, self.goal_y), 
                (self.goal_x + 10.0, self.goal_y), 
                (self.goal_x + 10.0, self.goal_y - 10.0), 
                (self.goal_x, self.goal_y - 10.0)
            ]
            self.search_idx = 0
            self.get_logger().info("Search pattern generated from current goal.")
    def new_goal_cb(self, msg):
        """Automatically updates the pilot's destination"""
        """This replaces the terminal input with AI coordinates"""
        self.get_logger().info(f"🎯 NEW MISSION RECEIVED: Moving to {msg.x}, {msg.y}, {msg.z}")
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_z = msg.z
        self.mission_active = True
        self.current_path = []

        # If the mission is surveillance, create a search square around the goal
        if self.current_mission == "surveillance":
            self.search_pattern = [
                (msg.x, msg.y), 
                (msg.x + 10.0, msg.y), 
                (msg.x + 10.0, msg.y - 10.0), 
                (msg.x, msg.y - 10.0)
            ]
            self.search_idx = 0

        # Safety: Reset to Takeoff if we were on the ground
        if self.phase == "FINISHED" or self.phase == "IDLE":
            self.phase = "WAIT_CONNECTION"
    
    def log_callback(self, msg):
        """Notes down faults with timestamps and coordinates"""
        if "FAULT" in self.latest_ai_decision or "REACHED" in self.latest_ai_decision:
            with open(self.fault_log_file, "a") as f:
                timestamp = self.get_clock().now().to_msg().sec
                log_entry = f"[{timestamp}] POS:({self.current_x:.2f},{self.current_y:.2f}) | {msg.data}\n"
                f.write(log_entry)
                self.get_logger().info(f"POINT NOTED: {msg.data}")

    def main_loop(self):
        if self.current_x is None: 
            return

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
            dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

            # 2. ARRIVAL HANDSHAKE: Signal Brain when 3.0m from bridge
            # This fixes the "Landing instead of Inspection" issue
            if self.mission_active and dist_to_goal < 5.0 and self.current_mission == "nav":
                self.get_logger().info("Visual range reached. Signaling Brain.")
                self.status_pub.publish(String(data="ARRIVED_AT_TARGET"))
                self.mission_active = False # Hover and wait for Brain to switch missions
                self.publish_setpoint(self.current_x, self.current_y, self.goal_z)
                return   
            # 2. GUIDED INSPECTION PATH
            # Replace your current "if self.current_mission == 'inspection':" block
            if self.current_mission == "inspection":
                # 1. Choose target based on sequence progress
                if not self.inspection_sequence_finished:
                    tx, ty, tz = self.inspection_waypoints[self.waypoint_idx]
                else:
                    # AFTER Point 10: Head back to Point 5
                    tx, ty, tz = self.inspection_waypoints[self.return_to_point_idx]

                # 2. Calculate Distance
                dx, dy, dz = tx - self.current_x, ty - self.current_y, tz - self.current_z
                dist_to_wp = math.sqrt(dx**2 + dy**2 + dz**2)
                # PHASE-AWARE SPEED:
                # Use travel_speed (1.5m/s) to get to Point 1.
                # Only use inspection_speed (0.3m/s) DURING the scan.
                if self.waypoint_idx == 0 and not self.inspection_sequence_finished:
                    active_speed = self.travel_speed 
                else:
                    active_speed = self.inspection_speed

                # 3. Handle Sequence Transitions
                if dist_to_wp < 0.8:
                    if not self.inspection_sequence_finished:
                        if self.waypoint_idx < len(self.inspection_waypoints) - 1:
                            self.waypoint_idx += 1
                            self.get_logger().info(f"Point {self.waypoint_idx} Cleared.")
                        else:
                            # Reached Point 10!
                            self.inspection_sequence_finished = True
                            self.get_logger().info("Scan complete. Returning to Point 5.")
                    else:
                        # Reached Point 5 on the way back!
                        self.get_logger().info("Back at Point 5. Returning to Hangar for Landing.")
                        self.phase = "LANDING"
                        return

                # --- CAMERA HEADING: Point directly at the bridge center line ---
                look_yaw = math.atan2(self.bridge_y - self.current_y, 0.0)

                # 5. Smooth Movement (0.3 m/s)
                max_step = self.inspection_speed * 0.2
                if dist_to_wp > max_step:
                    step_x = self.current_x + (dx/dist_to_wp)*max_step
                    step_y = self.current_y + (dy/dist_to_wp)*max_step
                    step_z = self.current_z + (dz/dist_to_wp)*max_step
                else:
                    step_x, step_y, step_z = tx, ty, tz

                self.publish_setpoint(step_x, step_y, step_z, look_yaw)
                return


            if self.current_mission == "surveillance":
                if not self.search_pattern:
                    self.get_logger().info("Waiting for surveillance coordinates...")
                    self.publish_setpoint(self.current_x, self.current_y, self.goal_z)
                    return
                self.goal_x, self.goal_y = self.search_pattern[self.search_idx]
                dist_to_wp = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

                if dist_to_wp < 1.0:
                    if self.search_idx < len(self.search_pattern) - 1:
                        # Move to next point in the box
                        self.search_idx += 1
                        self.get_logger().info(f"Moving to point {self.search_idx}")
                        self.current_path = [] 
                    else:
                        # FINISHED: Completed the box. Now land!
                        self.get_logger().info("Surveillance complete. Landing.")
                        self.phase = "LANDING"
                        return
                       

            if not self.mission_active:
                self.publish_setpoint(self.current_x, self.current_y, self.goal_z)
                return
            if self.mission_active and dist_to_goal < 1.0 and self.current_y < -30.0:
                self.get_logger().info("BRIDGE REACHED! Switching to Landing.")
                self.phase = "LANDING"
                return
            elif dist_to_goal < 1.0:
                # Just hover and wait for the bridge command coordinates
                self.publish_setpoint(0.0, 0.0, self.goal_z)
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

            # --- UPDATED NAVIGATION BLOCK ---
            if self.current_path:
                # 1. Get the current target waypoint
                target_x, target_y = self.current_path[0]
                
                # 2. Check distance to THIS waypoint (not just the final goal)
                dist_to_waypoint = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

                # 3. WAYPOINT PROGRESSION: If we are close to the waypoint, pop it!
                if dist_to_waypoint < 0.6:
                    if len(self.current_path) > 1:
                        self.current_path.pop(0) # Remove the reached point
                        target_x, target_y = self.current_path[0] # Get the NEW next point
                        self.get_logger().info(f"Waypoint Reached! Next point: {target_x}, {target_y}")
                    else:
                        # We are at the very last point of the path
                        self.get_logger().info("Last waypoint reached.")

                # 4. PATH-AWARE CHECK: Only stop if we are heading toward the obstacle
                target_yaw = math.atan2(target_y - self.current_y, target_x - self.current_x)
                angle_diff = abs(self.normalize_angle(target_yaw - self.get_yaw()))

                # Logic for "Sticky" STOP command
                if "STOP" in self.latest_ai_decision and angle_diff < 0.8:
                    self.stop_cooldown = 10 
    
                if self.stop_cooldown > 0:
                    if self.hover_pos is None:
                        self.hover_pos = (self.current_x, self.current_y)
                    self.publish_setpoint(self.hover_pos[0], self.hover_pos[1], self.goal_z)
                    self.stop_cooldown -= 1 
                    return
                else:
                    self.hover_pos = None

            # 5. APPLY REACTIVE OFFSETS (LEFT/RIGHT)
            offset_x, offset_y = 0.0, 0.0
            if "STEER_LEFT" in self.latest_ai_decision:
                yaw = self.get_yaw()
                offset_x = -1.5 * math.sin(yaw) 
                offset_y = 1.5 * math.cos(yaw)
            elif "STEER_RIGHT" in self.latest_ai_decision:
                yaw = self.get_yaw()
                offset_x = 1.5 * math.sin(yaw)
                offset_y = -1.5 * math.cos(yaw)

            # Use the (possibly updated) target_x/y
            final_target_x = target_x + offset_x
            final_target_y = target_y + offset_y

            # 6. SPEED CONTROL & SMOOTHING
            max_step = 0.6
            dx = final_target_x - self.current_x
            dy = final_target_y - self.current_y
            dist_to_target = math.sqrt(dx**2 + dy**2)

            if dist_to_target > max_step:
                final_target_x = self.current_x + (dx/dist_to_target) * max_step
                final_target_y = self.current_y + (dy/dist_to_target) * max_step

            # 7. GOAL REACHED CHECK (Land if we are near the final bridge coords)
            #MISSION PROGRESSION CHECK
            if dist_to_goal < 1.2 and self.mission_active:
                if abs(self.current_x) < 1.0 and abs(self.current_y) < 1.0:
                    return
                if self.current_mission == "nav":
                    self.get_logger().info("Destination Reached. Commencing LAND.")
                    self.phase = "LANDING"
                    return    
                elif self.current_mission == "surveillance":
                    # Cycle to the next point in the pattern instead of landing
                    self.search_idx = (self.search_idx + 1) % len(self.search_pattern)
                    self.get_logger().info(f"Switching to search point: {self.search_idx}")
                    # Update goal_x/y to the next point in the pattern
                    self.goal_x, self.goal_y = self.search_pattern[self.search_idx]
                    self.current_path = [] # Clear path to force A* recalculation
                else:
                    # For non-surveillance missions (like 'nav'), land normally
                    self.get_logger().info("GOAL REACHED! Switching to Landing.")
                    self.phase = "LANDING"
                    return

            # 8. FINAL PUBLISH
            move_yaw = math.atan2(dy, dx)
            self.publish_setpoint(final_target_x, final_target_y, self.goal_z, move_yaw)
            self.show_debug_map()

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
