import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import cv2
import torch
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
import os
import numpy as np
import struct
import math
from decision_nodes.agent import DroneAgent
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import time
from models import yolo_model

class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')
        self.declare_parameter('target_object', 'bridge')
        self.temp_img_path = os.path.expanduser("~/AgenticDrone/temp_vla_frame.jpg")
        self.agent = DroneAgent()
        self.actual_decision = "PATH_SAFE"
        self.latest_frame = None
        self.min_lidar_dist = 10.0
        self.stop_counter = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.latest_thermal_frame = None
        self.intended_final_mission = "land"
        self.latest_ai_results = None # To store boxes between AI thoughts
        cv2.namedWindow("Drone AI - Live Surveillance", cv2.WINDOW_NORMAL)
        cv2.startWindowThread()

        # Initialize the YOLO model for the UI
        self.ui_yolo = yolo_model 
        # Initialize the frame variable as None
        self.annotated_ui_frame = None

        # 2. START THE BRAIN THREAD: Prevents lagging/freezing
        threading.Thread(target=self.brain_loop, daemon=True).start()
        
        # 3. UI Timer: Smooth 20 FPS refresh
        self.create_timer(0.05, self.draw_live_window)
        
        # Communications
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(String, '/drone/user_command', self.command_cb, 10)
        self.pub = self.create_publisher(String, '/drone/vla_decision', 10)
        self.goal_pub = self.create_publisher(Point, '/drone/new_goal', 10)
        self.create_subscription(String, '/drone/pilot_status', self.pilot_status_cb, 10)
        self.desc_pub = self.create_publisher(String, '/drone/vla_description', 10)
        self.mission_type_pub = self.create_publisher(String, '/drone/active_mission', 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile)
        
        # Gazebo Transport
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image", self.gz_img_cb)
        self.gz_node.subscribe(GzImage, "/drone/thermal_image", self.process_thermal_image)
        self.get_logger().info("BRAIN: Online and listening for text commands...")

    # node.py - Corrected Handshake Logic
        self.intended_final_mission = "land" # Default to landing

    def command_cb(self, msg):
        try:
            plan = self.agent.understand_command(msg.data)
            
            # Store what we want to do AFTER arriving
            self.intended_final_mission = plan['mission'] 
            
            # Reset logs
            for log in ["surveillance_log.txt", "inspection_report.txt"]:
                with open(log, "w") as f: f.write(f"--- SESSION: {plan['mission'].upper()} ---\n")

            # Pull coordinates from the AI plan, including the 'z' altitude
            gx = float(plan.get('x', 0.0))
            gy = float(plan.get('y', 0.0))
            gz = float(plan.get('z', 3.5)) # Use the AI's 'z' value, or 3.5 if it's missing
            
            # Publish the full coordinate to the pilot
            self.goal_pub.publish(Point(x=gx, y=gy, z=gz))
            
            # SET MISSION TO 'NAV' (This tells the pilot: "Just fly there, don't orbit yet")
            self.mission_type_pub.publish(String(data="nav"))
            self.get_logger().info(f"PILOT TASKED: Navigate to ({gx}, {gy}) for {self.intended_final_mission}")
        except Exception as e:
            self.get_logger().error(f"Cmd Error: {e}")

    def pilot_status_cb(self, msg):
        """THE HANDSHAKE: Transition only when pilot reports arrival"""
        if msg.data == "ARRIVED_AT_TARGET":
            # Determine the final command
            final_task = self.intended_final_mission
            
            # CRITICAL FIX: If user only said 'nav', then 'nav' at destination means 'land'
            if final_task == "nav":
                final_task = "land"
                self.get_logger().info("Navigation complete. Commanding Pilot to LAND.")
            else:
                self.get_logger().info(f"Arrival confirmed. Starting {final_task} phase.")
            
            # Send the final mission type to the Pilot
            self.agent.current_mission = final_task
            self.mission_type_pub.publish(String(data=final_task))

    def gz_img_cb(self, msg):
        img_map = np.frombuffer(msg.data, dtype=np.uint8)
        frame = img_map.reshape((msg.height, msg.width, 3))
        self.latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        results = self.ui_yolo(self.latest_frame, conf=0.25, verbose=False)[0]
        self.annotated_ui_frame = results.plot()

    def pose_cb(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def brain_loop(self):
        """Runs in the background so the main window doesn't freeze"""
        while rclpy.ok():
            if self.latest_frame is not None:
                self.think() # Call the thinking logic
            import time
            time.sleep(0.1) # Small rest to save CPU

    def process_thermal_image(self, msg):
        try:
            # 1. Interpret as 16-bit data (2 bytes per pixel) 
            # Your terminal confirmed 153600 bytes for a 320x240 image
            raw_data = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

            # 2. DYNAMIC AUTO-SCALING (The "No-Black-Screen" Solution)
            # This ignores absolute Kelvin and scales the visible contrast 
            frame_8bit = cv2.normalize(raw_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
            # 3. Apply the Magma Colormap for the purple/orange look 
            self.latest_thermal_frame = cv2.applyColorMap(frame_8bit, cv2.COLORMAP_MAGMA)
        
            # Optional: Log the middle pixel value to your terminal to see the real Kelvin
            #mid_val = raw_data[120, 160]
            #self.get_logger().info(f"Thermal Data Sample: {mid_val}")

        except Exception as e:
            self.get_logger().error(f"Thermal Processing Error: {e}")

    def draw_live_window(self):
        # 1. Always show the RGB Surveillance Window
        if self.annotated_ui_frame is not None:
            display = cv2.resize(self.annotated_ui_frame, (800, 600))
            cv2.imshow("Drone AI - Live Surveillance", display)

        # 2. THE CONDITIONAL TRIGGER: Only show thermal for specific missions
        # We check the current mission set in the agent 
        active_thermal_missions = ["inspection", "surveillance"]
    
        if self.agent.current_mission in active_thermal_missions:
            if self.latest_thermal_frame is not None:
                thermal_display = cv2.resize(self.latest_thermal_frame, (800, 600))
                cv2.imshow("Drone Thermal - Ironbow View", thermal_display)
        else:
            # If we are in 'nav' or 'land', close the window to save CPU/Screen space
            try:
                cv2.destroyWindow("Drone Thermal - Ironbow View")
            except cv2.error:
                pass # Window was already closed or not yet created

        # 3. Handle OpenCV events
        cv2.waitKey(1)

    def think(self):
        if self.latest_frame is None: return
        
        target = self.get_parameter('target_object').get_parameter_value().string_value
        cv2.imwrite(self.temp_img_path, cv2.resize(self.latest_frame, (720, 720)))
        vis_frame = self.latest_frame.copy()
        # NEW: Save Thermal Evidence if the window is active
        thermal_img_path = None
        if self.latest_thermal_frame is not None:
            thermal_img_path = os.path.expanduser("~/AgenticDrone/thermal_evidence.jpg")
            cv2.imwrite(thermal_img_path, self.latest_thermal_frame)    

        try:
            ai_output = self.agent.get_decision(self.temp_img_path, self.min_lidar_dist, target=target)
            self.latest_ai_results = ai_output
            self.desc_pub.publish(String(data=ai_output["visual_analysis"]))
            decision = ai_output["final_decision"]
            actual_decision = "PATH_SAFE" 

            if self.agent.current_mission == "inspection":
                self.get_logger().info("generating inspection report...")
                self.generate_inspection_report(ai_output, thermal_img_path)
            # --- NEW: PASSIVE SURVEILLANCE LOGGING WITH IMAGES ---
            elif "OBJECT_REPORT" in decision or self.agent.current_mission == "surveillance":
                self.get_logger().info("generating surveillance report...")
                timestamp = self.get_clock().now().to_msg().sec
                img_name = f"detection_{timestamp}.jpg"
                save_dir = os.path.expanduser("~/AgenticDrone/detections/")
                
                # Ensure the directory exists
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                
                # Save the full-quality image
                img_path = os.path.join(save_dir, img_name)
                cv2.imwrite(img_path, self.latest_frame)

                # Log: Timestamp | X, Y | Image Filename | Description
                log_entry = f"[{timestamp}] POS:({self.current_x:.1f}, {self.current_y:.1f}) | IMG: {img_name} | {ai_output['visual_analysis']}"
                with open("surveillance_log.txt", "a") as f:
                    f.write(log_entry + "\n")
                
                self.get_logger().info(f"📸 DATA CAPTURED: {img_name} at ({self.current_x:.1f}, {self.current_y:.1f})")
                actual_decision = "PATH_SAFE"

            elif "STEER" in decision:
                actual_decision = decision
                self.stop_counter = 0
            else:
                self.stop_counter = (self.stop_counter + 1) if decision == "STOP" else 0
                actual_decision = "STOP" if self.stop_counter >= 2 else "PATH_SAFE"
            
            self.pub.publish(String(data=str(actual_decision)))
            self.get_logger().info(f"AI DECISION ({self.agent.current_mission}): {actual_decision}")
        except Exception as e:
            self.get_logger().error(f"Thinking Failed: {e}")
        finally:
            torch.cuda.empty_cache()
    def generate_inspection_report(self, ai_output, thermal_path=None):
        """Creates a descriptive, multi-sensor inspection log"""
        timestamp = self.get_clock().now().to_msg().sec
        report_file = "inspection_report.txt"

        status = "CRITICAL" if "FAULT" in ai_output["final_decision"] else "HEALTHY"

        with open(report_file, "a") as f:
            f.write(f"\n{'#'*50}\n")
            f.write(f"BRIDGE INSPECTION RECORD: {timestamp}\n")
            f.write(f"COORDINATES: X: {self.current_x:.2f}, Y: {self.current_y:.2f}\n")
            f.write(f"OVERALL STATUS: {status}\n")
            f.write(f"VISUAL ANALYSIS: {ai_output['visual_analysis']}\n")

            if thermal_path:
                f.write(f"THERMAL PROOF: Attached ({os.path.basename(thermal_path)})\n")
                # Optional: Logic to calculate if moisture is present based on pixel color

            f.write(f"{'#'*50}\n")       

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VLANode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
