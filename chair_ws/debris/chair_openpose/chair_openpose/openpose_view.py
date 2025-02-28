import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ViewCamera(Node):
    def __init__(self):
        super().__init__('view_camera')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('image', "/openpose/preview")

        self._image_topic = self.get_parameter('image').get_parameter_value().string_value
        self.create_subscription(Image, self._image_topic, self._image_callback, 1)
        self._bridge = CvBridge()

    def _image_callback(self, msg):
        image = self._bridge.imgmsg_to_cv2(msg, "bgr8")

        cv2.imshow('image', image)
        cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)
    node = ViewCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

