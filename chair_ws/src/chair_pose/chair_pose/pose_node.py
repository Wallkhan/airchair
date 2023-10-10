import os
import sys
import rclpy
import cv2
import datetime
import numpy as np
import pandas as pd
import math
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
    _BODY_PARTS = ["NOSE", "LEFT_EYE", "RIGHT_EYE", "LEFT_EAR", "RIGHT_EAR", "LEFT_SHOULDER", "RIGHT_SHOULDER",
                   "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE",
                   "RIGHT_KNEE", "LEFT_ANKLE", "RIGHT_ANKLE"]
    def __init__(self):
        super().__init__('pose_node')

        # params
        self._model_file = os.path.join(get_package_share_directory('chair_pose'), 'yolov8n-pose.pt') 
        self.declare_parameter("model", self._model_file) 
        model = self.get_parameter("model").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")
        self._device = self.get_parameter("device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self._threshold = self.get_parameter("threshold").get_parameter_value().double_value

        self.declare_parameter("min_shoulder_height", 0.5)
        self._min_shoulder_height = self.get_parameter("min_shoulder_height").get_parameter_value().double_value

        self.declare_parameter("max_shoulder_height", 0.90)
        self._max_shoulder_height = self.get_parameter("max_shoulder_height").get_parameter_value().double_value

        self.declare_parameter("camera_topic", "/mycamera/image_raw")
        self._camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value

        self.declare_parameter("chair_name", "chair_a")
        self._chair_name = self.get_parameter('chair_name').get_parameter_value().string_value

        
        self._move_flag = False
        self._bridge = CvBridge()
        self._model = YOLO(model)
        self._model.fuse()

        # subs
        self._sub = self.create_subscription(Image, self._camera_topic, self._camera_callback, 1) #qos_profile_sensor_data)
        
        #pubs
        self._chair_publisher = self.create_publisher(Twist, f'/{self._chair_name}/target_vel', 1)
        
    def parse_keypoints(self, results: Results):

        keypoints_list = []

        for points in results.keypoints:        
            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):
                if conf >= self._threshold:
                    keypoints_list.append([kp_id, p[0], p[1], conf])

        return keypoints_list
    
    def _camera_callback(self, data):
        self.get_logger().info(f'{self.get_name()} camera callback')
        img = self._bridge.imgmsg_to_cv2(data)
        results = self._model.predict(
                source = img,
                verbose = False,
                stream = False,
                conf = self._threshold,
                device = self._device
        )

        if len(results) != 1:
            self.get_logger().info(f'{self.get_name()}  Nothing to see here or too much {len(results)}')
            return
            
        results = results[0].cpu()
        if len(results.boxes.data) == 0:
            self.get_logger().info(f'{self.get_name()}  boxes are too small')
            return

        if results.keypoints:
            keypoints = self.parse_keypoints(results)
            left_shoulder = None
            right_shoulder = None
            if len(keypoints) > 0:
                for i in range(len(keypoints)):
                    if YOLO_Node._BODY_PARTS[keypoints[i][0]] == "LEFT_SHOULDER":
                        left_shoulder = [keypoints[i][1], keypoints[i][2]]
                    if YOLO_Node._BODY_PARTS[keypoints[i][0]] == "RIGHT_SHOULDER":
                        right_shoulder = [keypoints[i][1], keypoints[i][2]]

                if (left_shoulder is not None) and (right_shoulder is not None):
                    self.get_logger().info(f'{self.get_name()} left {left_shoulder} right {right_shoulder}')
                    height = ((left_shoulder[1] + right_shoulder[1]) / 2) / img.shape[0]
                    mid = ((left_shoulder[0] + right_shoulder[0]) / 2) / img.shape[1] - 0.5
                    self.get_logger().info(f'{self.get_name()} height {height} mid {mid}')

                    if height >= self._min_shoulder_height:
                        twist_msg = Twist()

                        if height < self._max_shoulder_height:
                            twist_msg.linear.x = 0.2
                        if mid > 0.1:
                            twist_msg.angular.z = 0.2
                        if mid < -0.1:
                            twist_msg.angular.z = -0.2

                        self._chair_publisher.publish(twist_msg)


                # Visualize results on frame        
                annotated_frame = results[0].plot()
                cv2.imshow('Results', annotated_frame)
                cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args=args)
    node = YOLO_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()


