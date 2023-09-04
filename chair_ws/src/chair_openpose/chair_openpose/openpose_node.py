import sys
import rclpy
import cv2
import datetime
import numpy as np
import pandas as pd
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
from chair_openpose.openpose_wrapper import OpenPoseWrapper
# from rcl_interfaces.msg import ParameterDescriptor, ParameterType
# image = self.bridge.cv2_to_imgmsg(result.cvOutputData, "rgb8")

class OpenPose(Node):
    def __init__(self):
        super().__init__('openpose')
        # openpose_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='The path of openpose project root.')
        # self._declare_parameter('openpose_root', '/openpose', openpose_descriptor)
        # openpose_root: str = self._get_parameter("openpose_root").get_parameter_value().string_value
        self._openpose_wrapper = OpenPoseWrapper('/home/walleed/openpose')
        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, '/openpose/preview', 1)
        self._chair_publisher = self.create_publisher(Twist, '/chair_c/cmd_vel', 10)
        self._received_flag = True
        self._move_flag = False
        self._BODY_PARTS = { 0: "Nose", 1: "Neck", 2: "RShoulder", 3: "RElbow", 4: "RWrist", 5: "LShoulder", 6: "LElbow", 7: "LWrist", 
                            8: "MidHip", 9: "RHip", 10: "RKnee", 11: "RAnkle", 12: "LHip", 13: "LKnee", 14: "LAnkle", 15: "REye", 
                            16: "LEye", 17: "REar", 18: "LEar", 19: "LBigToe", 20: "LSmallToe", 21: "LHeel", 22: "RBigToe",
                            23: "RSmallToe", 24: "RHeel", 25: "Background"}
        self._subsciption = self.create_subscription(
            Image, 
            '/mycamera/image_raw', 
            self.img_callback, 
            10)
        
    def publish_from_img(self, img: np.ndarray, timestamp: Time, frame_id: str =""):
        result = self._openpose_wrapper.body_from_image(img)
        result_image: Image = self._bridge.cv2_to_imgmsg(result.cvOutputData, "rgb8")
        result_image.header.stamp = timestamp
        result_image.header.frame_id = frame_id
        keypoints = result.poseKeypoints
        if keypoints is not None:
            # self.get_logger().info(f'{keypoints.shape[0]} people detected')
            for i in range(keypoints.shape[0]):
                for j in range(keypoints.shape[1]):
                    if keypoints[i][j][2] > 0.0:
                        # self.get_logger().info(f'{self._BODY_PARTS[j]}: {keypoints[i][j]}')
                        if(self._BODY_PARTS[j] == 'RWrist'):
                            self._move_flag = True
                            self.get_logger().info(f'RWrist Found: {self._move_flag}')
                            move = Twist()
                            move.linear.x = 1.0
                            move.angular.y = 1.0
                            self._chair_publisher.publish(move)
                        # if keypoints[i][j] == 'RWrist':
                        #     self._move_flag = True
                        #     self.get_logger().info(f'{self._BODY_PARTS[j]}: {keypoints[i][j]}')

        self._publisher.publish(result_image)
    
    
    def img_callback(self, image_raw):
        try:
            if self._received_flag:
                print('[' + str(datetime.datetime.now()) + '] Image received')
                self._received_flag = False
            image: np.ndarray = self._bridge.imgmsg_to_cv2(image_raw)
            self.publish_from_img(image, image_raw.header.stamp, image_raw.header.frame_id)
        except Exception as err:
            self.get_logger().error(err)
    

def main(args=None):
    rclpy.init(args=args)
    node = OpenPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    
