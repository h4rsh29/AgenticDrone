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
        self.desc_pub = self.create_publisher(String, '/drone/vla_description', 10)
        self.mission_type_pub = self.create_publisher(String, '/drone/active_mission', 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile)
        # Gazebo Transport
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image", self.gz_img_cb)
        self.get_logger().info("BRAIN: Online and listening for text commands...")

    # node.py -> improved command_cb
    def command_cb(self, msg):
        try:
            # 1. Ask the AI to understand the command
            plan = self.agent.understand_command(msg.data)

            # 1. RESET LOG: Wipe the file ONLY when a new mission starts
            with open("surveillance_log.txt", "w") as f:
                f.write(f"--- NEW SESSION: {plan['mission'].upper()} STARTED ---\n")
            
            # 2. Update the Agent's mission state
            self.agent.current_mission = plan['mission'] 
            
            # 3. Broadcast the mission to the Pilot
            mission_msg = String(data=plan['mission'])
            self.mission_type_pub.publish(mission_msg)

            # 4. ROBUST KEY CHECKING: Use .get() to avoid 'x' or 'target' errors
            target_name = plan.get('target', 'area')
            # If AI forgets x/y/z, use current position so the drone doesn't move unexpectedly
            gx = float(plan.get('x', self.current_x))
            gy = float(plan.get('y', self.current_y))
            gz = float(plan.get('z', 3.0)) # Default to 3m altitude

            self.set_parameters([rclpy.parameter.Parameter(
                'target_object', rclpy.Parameter.Type.STRING, target_name
            )])

            # 5. Send the goal to the Pilot
            goal = Point(x=gx, y=gy, z=gz)
            self.goal_pub.publish(goal)
            
            self.get_logger().info(f"MISSION STARTED: {plan['mission']} at {gx},{gy}")
            
        except Exception as e:
            # This captures the 'x' error you saw
            self.get_logger().error(f"Orchestration Error: {e}")

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

    def draw_live_window(self):
        if self.annotated_ui_frame is None: return
        display = cv2.resize(self.annotated_ui_frame, (800, 600))
        cv2.imshow("Drone AI - Live Surveillance", display)
        cv2.waitKey(1)

    def think(self):
        if self.latest_frame is None: return
        
        target = self.get_parameter('target_object').get_parameter_value().string_value
        cv2.imwrite(self.temp_img_path, cv2.resize(self.latest_frame, (720, 720)))
        vis_frame = self.latest_frame.copy()

        try:
            ai_output = self.agent.get_decision(self.temp_img_path, self.min_lidar_dist, target=target)
            self.latest_ai_results = ai_output
            self.desc_pub.publish(String(data=ai_output["visual_analysis"]))
            decision = ai_output["final_decision"]
            # --- NEW: PASSIVE SURVEILLANCE LOGGING WITH IMAGES ---
            if "OBJECT_REPORT" in decision or self.agent.current_mission == "surveillance":
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
            

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VLANode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
