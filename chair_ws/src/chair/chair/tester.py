import math
import yaml


import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('tester')

        # Declare and acquire `target_frame` parameter

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                        "a",
                        "b",
                        rclpy.time.Time())
            self.get_logger().info(f"Got back {t}")
        except TransformException as ex:
            self.get_logger().info(f"Unable to get transform in namespace {self.get_namespace()}")
        _frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        self.get_logger().info(f"{_frames_dict}")

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

