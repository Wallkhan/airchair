from enum import Enum
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from chair_interface.srv import EStop
from chair_interface.srv import Engage

class State(Enum):
    ESTOP = "E-stop"
    MANUAL = "Manual"
    ENGAGED = "Engaged"

class ChairController(Node):
    """This is a class that encompasses the basic controller for the wheelchair. It is in one of the
       states defined above. There are service calls to (try) to change from one state to another."""

    TARGET_TIMEOUT = 2 * 1e9  # in nanoseconds

    def __init__(self):
        super().__init__('chair_controller')
        self.get_logger().info(f'{self.get_name()} created')

        self._chair_status = State.ESTOP
        self._target_last_time = self.get_clock().now()

        self.declare_parameter('convoy_description', "convoy.json")
        self.declare_parameter('chair_descriptions', "chairs.json")
        self.declare_parameter('chair_name', "chair_a")
        self.declare_parameter('chair_status', "chair_status")      # chair's current running status (a State as defined above)
        self.declare_parameter('target_status', "target_status")    # chair's view of target status [0=lost, 1=good]
        self.declare_parameter('convoy_state', "convoy_state")      # static state of convoy Json string
        self.declare_parameter('chair_state', "chair_state")        # static state of this chair Json string
        self.declare_parameter('estop', "estop")                    # push or reset estop button
        self.declare_parameter('engage', "engage")                  # if not stopped and target status, engage the chair
 
        self.chair_name = self.get_parameter('chair_name').get_parameter_value().string_value
        self.get_logger().info(f'{self.get_name()} chair_name {self.chair_name}')

        convoy_description = self.get_parameter('convoy_description').get_parameter_value().string_value
        self.convoy_description = self._load_json(convoy_description)
        chair_descriptions = self.get_parameter('chair_descriptions').get_parameter_value().string_value
        self.chair_descriptions = self._load_json(chair_descriptions)
   
        # latched
        chair_state = self.get_parameter('chair_state').get_parameter_value().string_value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self._chair_state_pub = self.create_publisher(String, f"/{self.chair_name}/{chair_state}", qos)

        # latched
        convoy_state = self.get_parameter('convoy_state').get_parameter_value().string_value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self._convoy_state_pub = self.create_publisher(String, f"/{self.chair_name}/{convoy_state}", qos)

        chair_status = self.get_parameter('chair_status').get_parameter_value().string_value
        self._chair_status_pub = self.create_publisher(String, f"/{self.chair_name}/{chair_status}", 10)

        target_status = self.get_parameter('target_status').get_parameter_value().string_value
        self._target_status_pub = self.create_publisher(Float32, f"/{self.chair_name}/{target_status}", 10)

        # timer callback to publish status information
        self._timer = self.create_timer(0.1, self._timer_callback)

        estop_button = self.get_parameter('estop').get_parameter_value().string_value
        self.create_service(EStop, f"/{self.chair_name}/{estop_button}", self._handle_estop)

        engage_switch = self.get_parameter('engage').get_parameter_value().string_value
        self.create_service(Engage, f"/{self.chair_name}/{engage_switch}", self._handle_engage)

        # subscribe to teleop messages so we can forward them if in appropriate mode mode
        self.create_subscription(Twist, f"/{self.chair_name}/commanded_vel", self._teleop_subscriber, 10)
        self.create_subscription(Twist, f"/{self.chair_name}/target_vel", self._target_vel_subscriber, 10)
        self._teleop_publisher = self.create_publisher(Twist, f"/{self.chair_name}/cmd_vel", 10)

    def _load_json(self, file):
        try:
            return json.load(open(file, 'r'))
        except Exception as e:
            self.get_logger().info(f'{self.get_name()} unable to load/parse json file {file} {e}')
            return None

    def _send_chair_state(self, text):
        self._chair_state_pub.publish(String(data=text))

    def _send_convoy_state(self, text):
        self._convoy_state_pub.publish(String(data=text))

    def _timer_callback(self):
        """Deal with publishing target and chair status information"""
        self._chair_status_pub.publish(String(data=self._chair_status.value))
        
        v = max(1 - (self.get_clock().now() - self._target_last_time).nanoseconds / ChairController.TARGET_TIMEOUT, 0)
        if self._chair_status == State.ENGAGED:
            if v == 0:
                self._chair_status = State.ESTOP
                self._teleop_publisher.publish(Twist())
                self.get_logger().info(f'{self.get_name()} no messages so ESTOP')
            self.get_logger().info(f'{self.get_name()} timer callback estopping when engaged with v {v}')
        self._target_status_pub.publish(Float32(data=float(v)))

    def _handle_estop(self, request, response):
        """Request to set or reset estop"""
#        self.get_logger().info(f'{self.get_name()} setting estop to {request.estop}')
        if request.estop:
            self._chair_status = State.ESTOP
            msg = Twist()
            self._teleop_publisher.publish(msg)
        else:
            self._chair_status = State.MANUAL
        return response

    def _handle_engage(self, request, response):
        """Request to engage chair into convoy mode"""
#        self.get_logger().info(f'{self.get_name()} setting engage to {request.engage}')
        if self._chair_status == State.ESTOP:
            response.status = False
        elif self._chair_status == State.ENGAGED:
            response.status = False
        else:  # in manual mode
            v = max(1 - (self.get_clock().now() - self._target_last_time).nanoseconds / ChairController.TARGET_TIMEOUT, 0)
            if v >= 0:
                response.status = True
                self._chair_status = State.ENGAGED
                self._teleop_publisher.publish(Twist())
                self._target_last_time = self.get_clock().now()
                self.get_logger().info(f'{self.get_name()} now ENGAGED')
            else:
                response.status = False
        return response

    def _teleop_subscriber(self, msg):
#        self.get_logger().info(f'{self.get_name()} got a twist message {msg} in state {self._chair_status}')
        if self._chair_status == State.MANUAL:
            self._teleop_publisher.publish(msg)

    def _target_vel_subscriber(self, msg):
#        self.get_logger().info(f'{self.get_name()} got a twist message {msg} in state {self._chair_status}')
        self._target_last_time = self.get_clock().now()
        if self._chair_status == State.ENGAGED:
            self._teleop_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ChairController()
    convoy_description = node.convoy_description
    chair_description = node.chair_descriptions[node.chair_name]
    try:
        node._send_convoy_state(json.dumps(convoy_description))
        node._send_chair_state(json.dumps(chair_description))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

