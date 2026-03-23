import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
import struct
import torch
from sensor_msgs.msg import Image as ROSImage

# Native Gazebo Transport Imports
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
import time
from decision_nodes.agent import DroneAgent

class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')
        self.declare_parameter('target_object', 'person')
        self.agent = DroneAgent() 
        self.bridge = CvBridge()
        
        # Gazebo Native Node
        self.gz_node = GzNode()
        
        self.obstacle_detected = False
        self.latest_frame = None
        self.current_alt = 0.0
        self.min_lidar_dist = 100.0
        self.create_timer(1.0, self.think) # Run AI every 2.5 seconds instead of 1.0
        self.stop_counter = 0
        # ROS 2 Subscription for Altitude (Keep this, it's coming from waypoint_node)
        self.create_subscription(Float32, '/drone/current_altitude', self.alt_cb, 10)
        
        # DIRECT GAZEBO SUBSCRIPTIONS (Replaces ROS 2 subscriptions)
        self.gz_node.subscribe(GzImage, "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image", self.gz_img_cb)
        self.gz_node.subscribe(PointCloudPacked, "/world/default/model/x500_depth_0/link/lidar_link/sensor/gpu_lidar_3d/scan/points", self.gz_lidar_cb)
        
        # ROS 2 Publishers for Waypoint Node
        self.pub = self.create_publisher(String, '/drone/vla_decision', 10)
        self.ai_pub = self.create_publisher(Int32, '/drone/ai_status', 10)
        self.ros_img_pub = self.create_publisher(ROSImage, '/drone/camera_rviz', 10)
        self.get_logger().info("VLA NODE STARTED: Using Direct Gazebo Transport")

    def alt_cb(self, msg):
        self.current_alt = msg.data

    def gz_lidar_cb(self, msg):
    # Unpack binary data from PointCloudPacked
        data = msg.data
        step = msg.point_step
        front_distances = []

        for i in range(0, len(data), step * 4): # Sample every 4th point for speed
            # Unpack x, y, z coordinates
            x, y, z = struct.unpack_from('fff', data, i)
        
            # Only look at points directly in front of the drone
            if 0.5 < x < 5.0 and abs(y) < 1.0 and abs(z) < 0.5:
                dist = math.sqrt(x**2 + y**2)
                front_distances.append(dist)

        if front_distances:
            self.min_lidar_dist = min(front_distances)
        else:
            self.min_lidar_dist = 10.0 # Clear path default

    def gz_img_cb(self, msg):
        # Native Gazebo Image to OpenCV conversion
        img_map = np.frombuffer(msg.data, dtype=np.uint8)
        frame = img_map.reshape((msg.height, msg.width, 3))
        self.latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert OpenCV frame to ROS 2 Image message
        ros_img = self.bridge.cv2_to_imgmsg(self.latest_frame, "bgr8")
        ros_img.header.frame_id = "camera_link"
        self.ros_img_pub.publish(ros_img)

    def think(self):
        if self.latest_frame is not None:
            current_target = self.get_parameter('target_object').get_parameter_value().string_value
            # 1. Resize image to 320x320 before saving
            # This significantly reduces memory usage for the VLM
            small_frame = cv2.resize(self.latest_frame, (224, 224))
            temp_img = 'temp_vla_frame.jpg'
            success = cv2.imwrite(temp_img, small_frame)
            if not success:
                self.get_logger.error("failed to write frame to disk")
                return

            decision = "PATH_SAFE"
            
            try:
                decision = self.agent.get_decision(temp_img, self.min_lidar_dist, target=current_target)
                # Simple Stability Filter
                if decision == "STOP":
                    self.stop_counter += 1
                else:
                    self.stop_counter = 0
            except Exception as e:
                self.get_logger().error(f"AI Agent Failed: {e}")
                self.stop_counter = 0
            finally:
                #release gpu memory
                torch.cuda.empty_cache()    

            # Only publish STOP if we see it for 3 consecutive frames
            actual_decision = "STOP" if self.stop_counter >= 2 else "PATH_SAFE"

            res_msg = String()
            res_msg.data = str(actual_decision)
            self.pub.publish(res_msg)

            status_msg = Int32()
            status_msg.data = 2 
            self.ai_pub.publish(status_msg)

            self.get_logger().info(f"AGENT: {decision}")
            self.latest_frame = None

def main(args=None):
    rclpy.init(args=args)
    node = VLANode()
    try:
        # We use rclpy.spin to keep ROS 2 alive; Gazebo callbacks run in background threads
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:    
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
