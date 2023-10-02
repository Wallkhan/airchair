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

        self.declare_parameter("camera_topic", "/mycamera/image_raw")
        self._camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        
        self._move_flag = False
        self._bridge = CvBridge()
        self._model = YOLO(model)
        self._model.fuse()

        # subs
        self._sub = self.create_subscription(
            Image, self._camera_topic, self._camera_callback, qos_profile_sensor_data)
        
        #pubs
        self._chair_publisher = self.create_publisher(Twist, '/chair_z/cmd_vel', 10)
        
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
            
        results: Results = results[0].cpu()
        self.get_logger().info(f'{self.get_name()} {results.names[0]} {results.boxes.data} {results.boxes.conf} {results.orig_shape} {len(results.boxes.data)}')
        if len(results.boxes.data) == 0:
            self.get_logger().info(f'{self.get_name()}  boxes are too small')
            return
        box = results.boxes.data[0]
        mid = (box[0] + box[2]) / 2
        full = results.orig_shape[1] / 2
        diff = (full - mid) /   full
        height = (box[3] - box[1]) /  results.orig_shape[0]
        # self.get_logger().info(f'{self.get_name()} from -1..+1 {diff} height {height}')
        

        if results.keypoints:
            keypoints = self.parse_keypoints(results)
            if len(keypoints) > 0:
                self.get_logger().info(f'{self.get_name()} got {len(keypoints)}')
                shoulders = self._find_shoulder_width(keypoints)
                torso = self._find_torso_length(keypoints)
                self.get_logger().info(f'Shoulder Distance = {shoulders}, Torso Length = {torso}')
                for i in range(len(keypoints)):
                    self.get_logger().info(f'{self.get_name()} got keypoint {YOLO_Node._BODY_PARTS[keypoints[i][0]]}')
                    if keypoints[i][0] == 9:
                        self._move_flag = True
                        # self.get_logger().info(f'Left Wrist Found: {self._move_flag}')
                        move = Twist()
                        move.linear.x = 0.0
                        move.angular.y = 0.0
                        self._chair_publisher.publish(move)
                    
                    if keypoints[i][0] == 10:
                        self._move_flag = True
                        # self.get_logger().info(f'Right Wrist Found: {self._move_flag}')
                        move = Twist()
                        move.linear.x = 0.5
                        move.angular.y = 0.5
                        self._chair_publisher.publish(move)

                # Visualize results on frame        
                annotated_frame = results[0].plot()
    
                cv2.imshow('Results', annotated_frame)
                cv2.waitKey(3)
    
    def _find_shoulder_width(self, keypoints):
        try:
            left_shoulder_point = [keypoints[5][1], keypoints[5][2]]
            right_shoulder_point = [keypoints[6][1], keypoints[6][2]]
            difference = math.dist(left_shoulder_point, right_shoulder_point)
            return difference
        except:
            self.get_logger().info(f'Shoulders not found')
    
    def _find_torso_length(self, keypoints):
        try:
            if (keypoints[5] and keypoints[11] is not None) or (keypoints[6] and keypoints[12] is not None):
                left_shoulder_point = [keypoints[5][1], keypoints[5][2]]
                left_hip_point = [keypoints[11][1], keypoints[11][2]]
                right_shoulder_point = [keypoints[6][1], keypoints[6][2]]
                right_hip_point = [keypoints[12][1], keypoints[12][2]]
                difference = math.dist(left_shoulder_point, left_hip_point)
                self.get_logger().info(f'Shoulder to Torso = {difference}')
        except:
            self.get_logger().info(f'Keypoints not found!')
       
def main(args=None):
    rclpy.init(args=args)
    node = YOLO_Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


