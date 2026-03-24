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

class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')
        self.declare_parameter('target_object', 'bridge')
        self.temp_img_path = os.path.expanduser("~/AgenticDrone/temp_vla_frame.jpg")
        self.agent = DroneAgent()
        self.latest_frame = None
        self.min_lidar_dist = 10.0
        self.stop_counter = 0

        # Communications
        self.create_subscription(String, '/drone/user_command', self.command_cb, 10)
        self.pub = self.create_publisher(String, '/drone/vla_decision', 10)
        self.goal_pub = self.create_publisher(Point, '/drone/new_goal', 10)
        self.desc_pub = self.create_publisher(String, '/drone/vla_description', 10)

        # Gazebo Transport
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image", self.gz_img_cb)
        
        self.create_timer(1.0, self.think)
        self.get_logger().info("BRAIN: Online and listening for text commands...")

    def command_cb(self, msg):
        try:
            plan = self.agent.understand_command(msg.data)
            self.set_parameters([rclpy.parameter.Parameter('target_object', rclpy.Parameter.Type.STRING, plan['target'])])
            goal = Point(x=float(plan['x']), y=float(plan['y']), z=float(plan['z']))
            self.goal_pub.publish(goal)
            self.get_logger().info(f"MISSION STARTED: {plan['mission']} -> {plan['x']},{plan['y']}")
        except Exception as e:
            self.get_logger().error(f"Orchestration Error: {e}")

    def gz_img_cb(self, msg):
        img_map = np.frombuffer(msg.data, dtype=np.uint8)
        frame = img_map.reshape((msg.height, msg.width, 3))
        self.latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def think(self):
        if self.latest_frame is None: return
        
        target = self.get_parameter('target_object').get_parameter_value().string_value
        cv2.imwrite(self.temp_img_path, cv2.resize(self.latest_frame, (224, 224)))

        try:
            ai_output = self.agent.get_decision(self.temp_img_path, self.min_lidar_dist, target=target)
            self.desc_pub.publish(String(data=ai_output["visual_analysis"]))
            
            decision = ai_output["final_decision"]
            if "STEER" in decision:
                actual_decision = decision
            else:
                self.stop_counter = (self.stop_counter + 1) if decision == "STOP" else 0
                actual_decision = "STOP" if self.stop_counter >= 2 else "PATH_SAFE"
                self.pub.publish(String(data=str(actual_decision)))
                self.get_logger().info(f"AI DECISION: {actual_decision}")
        except Exception as e:
            self.get_logger().error(f"Thinking Failed: {e}")
        finally:
            torch.cuda.empty_cache()
            self.latest_frame = None

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VLANode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
