import cv2
import numpy as np
import time
import rclpy # Added ROS 2
from rclpy.node import Node as RosNode
from std_msgs.msg import Float32 # To send distance to the drone
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from ultralytics import YOLO 
from std_msgs.msg import Int32

class DronePerception(RosNode):
    def __init__(self):
        # Initialize ROS 2 Node
        super().__init__('drone_perception')
        self.safety_pub = self.create_publisher(Int32, '/drone/ai_status', 10)
        # Initialize Gazebo Node
        self.gz_node = GzNode()
        self.model = YOLO('yolo26n.pt') 
        self.latest_min_dist = 100.0 

        # Subscriptions
        self.gz_node.subscribe(Image, "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image", self.camera_callback)
        self.gz_node.subscribe(LaserScan, "/world/baylands/model/x500_depth_0/link/lidar_link/sensor/gpu_lidar_3d/scan", self.lidar_callback)
        
        self.current_alt=0.0
        self.alt_pub=self.create_subscription(Float32,'/drone/current_altitude', self.alt_cb, 10)
        
        
    def alt_cb(self,msg):
        self.current_alt=msg.data

    def lidar_callback(self, msg):
        # 0.5m filter to ignore drone body
        valid_ranges = [r for r in msg.ranges if 1.1 < r < 20.0]
        if valid_ranges:
            self.latest_min_dist = min(valid_ranges)
        else:
            self.latest_min_dist = 100.0

    def camera_callback(self, msg):
        # Image conversion
        img_map = np.frombuffer(msg.data, dtype=np.uint8)
        frame = img_map.reshape((msg.height, msg.width, 3))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # YOLO processing
        results = self.model(frame, conf=0.15, imgsz=320, verbose=False)
        obj_count = len(results[0].boxes)
        
        
        # 3. AI AGENT DECISION LOGIC
        ai_status = 0 # 0: CLEAR, 1: AVOID, 2: STOP
        status_text = "CLEAR"
        
        
        
        
        # Only trigger Emergency Stop if we are higher than 1.0m (not on ground)
        if self.current_alt > 1.0 and self.latest_min_dist < 1.5:
            ai_status = 2
            status_text = "EMERGENCY STOP"
        elif obj_count > 0 and self.latest_min_dist < 3.5:
            ai_status = 1
            status_text = "AVOIDING OBSTACLE"
        
    
        # This will print every time a camera frame is processed
        obj_count = len(results[0].boxes)
        print(f"🤖 AGENT STATUS | Objects Detected: {obj_count} | Nearest Obstacle: {self.latest_min_dist:.2f}m")

        # Visual Alert and publishing
        status_msg=Int32()
        status_msg.data=ai_status
        self.safety_pub.publish(status_msg)  #sending to waypoint
        
        annotated_frame=results[0].plot()
        if obj_count > 0 and self.latest_min_dist < 3.0:
            cv2.putText(annotated_frame, f"COMBINED ALERT: {self.latest_min_dist:.2f}m", 
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("YOLO Safety Monitor", annotated_frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    perception_node = DronePerception()
    try:
        # Keep ROS 2 spinning while Gazebo callbacks run in the background
        while rclpy.ok():
            rclpy.spin_once(perception_node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
