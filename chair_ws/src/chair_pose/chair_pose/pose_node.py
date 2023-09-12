import os
import sys
import rclpy
import cv2
import datetime
import numpy as np
import pandas as pd
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from ultralytics.engine.results import Results, Keypoints
from ament_index_python.packages import get_package_share_directory

class YOLO_Node(Node):
    def __init__(self):
        super().__init__('pose_node')

        # params
        self._model_file = os.path.join(get_package_share_directory('chair_pose_yolo'), 'yolov8n-pose.pt') 
        self.declare_parameter("model", self._model_file) 
        model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")
        self._device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self._threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self._enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.declare_parameter("camera_topic", "/mycamera/image_raw")
        self._camera_topic = self.get_parameter(
            "camera_topic").get_parameter_value().string_value
        
        self._move_flag = False
        self._bridge = CvBridge()
        self._model = YOLO(model)
        self._model.fuse()
        self._BODY_PARTS = {0: "NOSE", 1: "LEFT_EYE", 2: "RIGHT_EYE", 3: "LEFT_EAR", 4: "RIGHT_EAR", 5: "LEFT_SHOULDER", 6: "RIGHT_SHOULDER",
                            7: "LEFT_ELBOW", 8: "RIGHT_ELBOW", 9: "LEFT_WRIST", 10: "RIGHT_WRIST", 11: "LEFT_HIP", 12: "RIGHT_HIP", 13: "LEFT_KNEE",
                            14: "RIGHT_KNEE", 15:"LEFT_ANKLE", 16: "RIGHT_ANKLE"}

        # subs
        self._sub = self.create_subscription(
            Image, self._camera_topic, self._camera_callback, qos_profile_sensor_data)
        
        #pubs
        self._chair_publisher = self.create_publisher(Twist, '/chair_z/cmd_vel', 10)
        
    def parse_keypoints(self, results: Results):

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:        
            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):
                if conf >= self._threshold:
                    keypoints_list.append([kp_id, p[0], p[1], conf])

        return keypoints_list
    
    def _camera_callback(self, data):
        if self._enable:
            img = self._bridge.imgmsg_to_cv2(data)
            # Run inference on the incoming frame
            results = self._model.predict(
                source = img,
                verbose = False,
                stream = False,
                conf = self._threshold,
                device = self._device
            )
            
            results: Results = results[0].cpu()
            if results.keypoints:
                keypoints = self.parse_keypoints(results)
            
            for i in range(len(keypoints)):
                if keypoints[i][0] == 9:
                    self._move_flag = True
                    self.get_logger().info(f'Left Wrist Found: {self._move_flag}')
                    move = Twist()
                    move.linear.x = 0.0
                    move.angular.y = 0.0
                    self._chair_publisher.publish(move)
                
                if keypoints[i][0] == 10:
                    self._move_flag = True
                    self.get_logger().info(f'Right Wrist Found: {self._move_flag}')
                    move = Twist()
                    move.linear.x = 0.5
                    move.angular.y = 0.5
                    self._chair_publisher.publish(move)

            # Visualize results on frame
            annotated_frame = results[0].plot()

            cv2.imshow('Results', annotated_frame)
            cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)
    node = YOLO_Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()