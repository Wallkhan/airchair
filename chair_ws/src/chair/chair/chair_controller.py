import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_msgs.msg import Bool
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from chair_interface.srv import EStop


class ChairController(Node):
    def __init__(self):
        super().__init__('chair_controller')
        self.get_logger().info(f'{self.get_name()} created')

        self._chair_status = 'e-stop'
        self._target_status = False

        self.declare_parameter('chair_status', "/chair_a/chair_status")      # chair's current running status [e-stop, manual, engaged]
        self.declare_parameter('target_status', "/chair_a/target_status")    # chair's view of target status [True, False] (for leader is of person)
        self.declare_parameter('convoy_state', "/chair_a/convoy_state")      # static state of convoy [chair_a, chair_b, chair_c]
        self.declare_parameter('chair_state', "/chair_a/chair_state")        # static state of chair Json string
        self.declare_parameter('estop', "/chair_a/estop")                    # push or reset estop button
 
        # latched
        chair_state = self.get_parameter('chair_state').get_parameter_value().string_value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self._chair_state_pub = self.create_publisher(String, chair_state, qos)

        # latched
        convoy_state = self.get_parameter('convoy_state').get_parameter_value().string_value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self._convoy_state_pub = self.create_publisher(String, convoy_state, qos)

        chair_status = self.get_parameter('chair_status').get_parameter_value().string_value
        self._chair_status_pub = self.create_publisher(String, chair_status, 10)

        target_status = self.get_parameter('target_status').get_parameter_value().string_value
        self._target_status_pub = self.create_publisher(Bool, target_status, 10)

        # timer callback to publish status information
        self._timer = self.create_timer(0.1, self._timer_callback)

        estop_button = self.get_parameter('estop').get_parameter_value().string_value
        self.create_service(EStop, estop_button, self._handle_estop)


    def _send_chair_state(self, text):
        self._chair_state_pub.publish(String(data=text))

    def _send_convoy_state(self, text):
        self._convoy_state_pub.publish(String(data=text))

    def _timer_callback(self):
        self._chair_status_pub.publish(String(data=self._chair_status))
        self._target_status_pub.publish(Bool(data=self._target_status))

    def _handle_estop(self, request, response):
        self.get_logger().info(f'{self.get_name()} setting estop to {request}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ChairController()
    try:
        node._send_convoy_state("Mares eat oats")
        node._send_chair_state("Does eat oats")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

