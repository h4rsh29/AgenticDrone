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
from datetime import datetime
import re

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
        self.total_healthy = 0
        self.total_critical = 0
        self.confidences = [] # To calculate AI Average
        self.crowd_hotspots = [] # To store (x, y, person_count)
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
        self.intended_bridge_y = 0.0 # Initialize the variable

    def command_cb(self, msg):
        try:
            plan = self.agent.understand_command(msg.data)

            # --- INSERT THE NEW MANUAL SUMMARY TRIGGER HERE ---
            # If we are currently in a mission and the new command is to land/nav, 
            # we trigger the summary before switching tasks.
            if self.agent.current_mission in ["inspection", "surveillance"]:
                if plan['mission'] in ["land", "nav"]:
                    self.trigger_mission_summary()
            
            # Store what we want to do AFTER arriving
            self.intended_final_mission = plan['mission'] 
            # Initialize professional Surveillance Report
            with open("surveillance_report.md", "w") as f:
                f.write("# 🛸 Mission Intelligence Report: Surveillance\n")
                f.write(f"**Status:** `ACTIVE` | **Session ID:** `{plan['mission'].upper()}`\n\n")
                f.write("--- \n\n")
                f.write("## 📍 Intelligence Logs\n") # Removed the table header

            # Initialize professional Inspection Report (.md)
            with open("inspection_report.md", "w") as f:
                f.write("# 🌉 Structural Inspection Report\n")
                f.write(f"**Target:** `{plan['mission'].upper()}` | **System:** AgenticDrone\n\n")
                f.write("--- \n\n")
                f.write("## 🔍 Inspection Findings\n")

            # Pull coordinates from the AI plan, including the 'z' altitude
            gx = float(plan.get('x', 0.0))
            gy = float(plan.get('y', 0.0))
            self.intended_bridge_y = gy # Store the bridge center for the report
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

            if final_task == "land":
                self.trigger_mission_summary()  
            
            # Send the final mission type to the Pilot
            self.agent.current_mission = final_task
            self.mission_type_pub.publish(String(data=final_task))

    def gz_img_cb(self, msg):
        #print(f"DEBUG: Frame Received! Resolution: {msg.width}x{msg.height}")
        try:
            img_map = np.frombuffer(msg.data, dtype=np.uint8)
            # Calculate bytes per pixel automatically
            channels = len(msg.data) // (msg.width * msg.height)
            frame = img_map.reshape((msg.height, msg.width, channels))
            
            # Convert to BGR for OpenCV
            if channels == 4:
                self.latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            else:
                self.latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
            # Update YOLO overlay
            results = self.ui_yolo(self.latest_frame, conf=0.25, verbose=False)[0]
            self.annotated_ui_frame = results.plot()
        except Exception as e:
            self.get_logger().error(f"Image Reshape Error: {e}")
    def pose_cb(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def brain_loop(self):
        """Runs in the background so the main window doesn't freeze"""
        while rclpy.ok():
            if self.latest_frame is not None:
                self.think() # Call the thinking logic
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
            # 1. Save AI Confidence (for Average calculation)
            if "detection_boxes" in ai_output:
                for box in ai_output["detection_boxes"]:
                    self.confidences.append(box["conf"])

            # 2. Update Inspection Counters
            if self.agent.current_mission == "inspection":
                if ai_output["final_decision"] == "CRITICAL":
                    self.total_critical += 1
                else:
                    self.total_healthy += 1

            # 3. Update Surveillance Crowd Data
            elif self.agent.current_mission == "surveillance":
                raw_yolo = ai_output.get("yolo_report", "")
                
                # Extract the actual number after "Person: "
                match = re.search(r"Person: (\d+)", raw_yolo)
                person_count = int(match.group(1)) if match else 0
                
                if person_count > 0:
                    self.crowd_hotspots.append({
                        "x": self.current_x, 
                        "y": self.current_y, 
                        "count": person_count
                    })
            self.latest_ai_results = ai_output
            self.desc_pub.publish(String(data=ai_output["visual_analysis"]))
            decision = ai_output["final_decision"]
            actual_decision = "PATH_SAFE" 

            # --- SELECTIVE IMAGE SAVING ---
            fault_img_path = None
            if self.agent.current_mission == "inspection":
                self.get_logger().info("generating inspection report...")
                timestamp = self.get_clock().now().to_msg().sec
                vis_path = None
                therm_path = None
                
                # --- SELECTIVE IMAGE SAVING (Only for Critical Faults) ---
                if decision == "CRITICAL":
                    timestamp = self.get_clock().now().to_msg().sec
                    fault_dir = os.path.expanduser("~/AgenticDrone/faults/")
                    if not os.path.exists(fault_dir): os.makedirs(fault_dir)
                    vis_path = os.path.join(fault_dir, f"fault_{timestamp}_vis.jpg")
                    therm_path = os.path.join(fault_dir, f"fault_{timestamp}_therm.jpg")
                    
                    cv2.imwrite(vis_path, self.latest_frame)
                    if self.latest_thermal_frame is not None:
                        cv2.imwrite(therm_path, self.latest_thermal_frame)
                    
                    self.get_logger().info(f"🚨 FAULT CAPTURED: {decision} status confirmed.")
                
                # --- ALWAYS GENERATE REPORT (For both Healthy and Critical) ---
                # This is now outside the 'if decision == "CRITICAL"' block
                self.generate_inspection_report(ai_output, visual_path=vis_path, thermal_path=therm_path)
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

                # CALL THE NEW FUNCTION HERE
                self.generate_surveillance_report(ai_output, img_name)
                
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

    def generate_surveillance_report(self, ai_output, img_name):
        """Appends a structured entry to the Markdown dashboard"""
        timestamp = self.get_clock().now().to_msg().sec
        # NEW: Create the readable format: 2026-04-09 | 14:45:01
        readable_time = datetime.now().strftime("%Y-%m-%d | %H:%M:%S")
        
        # Identify if the scene is crowded based on VLM analysis
        status = "CROWDED" if "person" in ai_output['visual_analysis'].lower() else "ACTIVE"
    
        # Get the count summary we created in Step 1
        object_counts = ai_output.get("yolo_report", "No data")
        img_rel_path = f"detections/{img_name}"

        with open("surveillance_report.md", "a") as f:
            f.write(f"### 🛰️ Detection Record: `{timestamp}`\n")
            f.write(f"- **Position:** `({self.current_x:.1f}, {self.current_y:.1f})` | **Status:** **[{status}]**\n")
            # NEW: Timestamp added under Position
            f.write(f"- **Timestamp:** `{readable_time}`\n")
            f.write(f"- **Live Counts:** `{object_counts}`\n\n")
            f.write(f"#### 📜 Technical Analysis\n\n{ai_output['visual_analysis']}\n\n")
            f.write(f"**Evidence:**\n![Detection]({img_rel_path})\n\n")
            f.write("---\n")

    def generate_inspection_report(self, ai_output, visual_path=None, thermal_path=None):
        """Creates a professional Markdown inspection log with embedded evidence"""
        timestamp = self.get_clock().now().to_msg().sec
        readable_time = datetime.now().strftime("%Y-%m-%d | %H:%M:%S")
        bridge_type = "STEEL_BRIDGE" if self.current_y > 0 else "CONCRETE_BRIDGE"
        status = ai_output["final_decision"]
        
        # Color-coded status tags for instant visual recognition
        status_tag = f"🔴 **{status}**" if status == "CRITICAL" else f"🟢 **{status}**"

        with open("inspection_report.md", "a") as f:
            f.write(f"### 📍 Record: `{timestamp}`\n")
            f.write(f"- **Location:** {bridge_type} | **GPS:** `({self.current_x:.2f}, {self.current_y:.2f})`\n")
            f.write(f"- **Timestamp:** `{readable_time}`\n")
            f.write(f"- **Status:** {status_tag}\n")
            f.write(f"- **Analysis:** {ai_output['visual_analysis']}\n\n")
            
            # Embed image evidence directly in the report
            # Note: We use the 'faults/' prefix so Markdown can find the files
            if visual_path:
                img_rel_path = f"faults/{os.path.basename(visual_path)}"
                f.write(f"**Visual Evidence:**\n![Visual]({img_rel_path})\n\n")
            if thermal_path:
                therm_rel_path = f"faults/{os.path.basename(thermal_path)}"
                f.write(f"**Thermal Proof:**\n![Thermal]({therm_rel_path})\n\n")
            
            f.write("---\n")  
    def finalize_inspection_analytics(self):
        """Appends the Structural Summary to inspection_report.md"""
        total = self.total_healthy + self.total_critical
        avg_conf = (sum(self.confidences) / len(self.confidences)) * 100 if self.confidences else 0
        
        with open("inspection_report.md", "a") as f:
            f.write("\n## 📊 Final Structural Analytics\n")
            f.write("| Metric | Value |\n| :--- | :--- |\n")
            f.write(f"| **Total Records** | {total} |\n")
            f.write(f"| **Healthy Sections (🟢)** | {self.total_healthy} |\n")
            f.write(f"| **Critical Faults (🔴)** | {self.total_critical} |\n")
           # f.write(f"| **AI Confidence Avg** | {avg_conf:.1f}% |\n")
           # f.write(f"| **System Health** | **{'Optimal' if avg_conf > 85 else 'Check Sensors'}** |\n")

    def finalize_surveillance_analytics(self):
        """Appends the Intelligence Summary to surveillance_report.md"""
        avg_conf = (sum(self.confidences) / len(self.confidences)) * 100 if self.confidences else 0
        # Sort to find the top 3 densest areas
        top_spots = sorted(self.crowd_hotspots, key=lambda x: x['count'], reverse=True)[:3]

        with open("surveillance_report.md", "a") as f:
            f.write("\n## 📊 Mission Intelligence Summary\n")
            f.write("| Metric | Value |\n| :--- | :--- |\n")
            f.write(f"| **Total Observations** | {len(self.confidences)} |\n")
            f.write(f"| **AI Confidence Avg** | {avg_conf:.1f}% |\n")
            f.write(f"| **System Health** | **Optimal** |\n\n")
            
            if top_spots:
                f.write("### 📍 Crowd Density Map (Top Hotspots)\n")
                for spot in top_spots:
                    f.write(f"- `{spot['count']} People` detected at GPS `({spot['x']:.1f}, {spot['y']:.1f})`\n")           
    
    def trigger_mission_summary(self):
        """Checks the current mission and writes the correct summary table"""
        if self.agent.current_mission == "inspection":
            self.get_logger().info("Manual Stop: Finalizing Inspection Report...")
            self.finalize_inspection_analytics()
        elif self.agent.current_mission == "surveillance":
            self.get_logger().info("Manual Stop: Finalizing Surveillance Report...")
            self.finalize_surveillance_analytics()
    def destroy_node(self):
        """Ensures the report is saved even if you Ctrl+C the terminal"""
        self.get_logger().info("Shutting down... saving final mission analytics.")
        self.trigger_mission_summary()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VLANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected!")
    finally:
        # destroy_node ALREADY calls trigger_mission_summary(), 
        # so we only need to call destroy_node here.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
