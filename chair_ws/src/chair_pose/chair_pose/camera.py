import sys
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class OpenCVCamera(Node):

    def __init__(self):
        super().__init__('opencv_camera')
        self.get_logger().info(f'{self.get_name()} starting')

        self.declare_parameter("video_channel", "0")
        video_channel = self.get_parameter("video_channel").get_parameter_value().integer_value

        self.declare_parameter("camera_topic", "/mycamera/image_raw")
        video_topic = self.get_parameter("camera_topic").get_parameter_value().string_value


        try:
            self._camera =  cv2.VideoCapture(video_channel, cv2.CAP_V4L2)
        except Exception as e:
            self.get_logger().error(f'{self.get_name()} Unable to open camera {video_channel}')
            sys.exit(1)

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, video_topic, 1)

    def stream(self):
        try:
            while True:
                _, frame = self._camera.read()
                image_message = self._bridge.cv2_to_imgmsg(frame, "bgr8")

                self._publisher.publish(image_message)
                rclpy.spin_once(self, timeout_sec=0)
        except Exception as e:
            self.get_logger().error(f'{self.get_name()} capture error {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVCamera()
    try:
        node.stream()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
