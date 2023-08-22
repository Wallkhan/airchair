import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist


from geometry_msgs.msg import PointStamped

class ChairFollower(Node):
    def __init__(self):
        super().__init__('chair_follower')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('chair_name', "chair_a")
        self.declare_parameter('target_offset', "target_offset")

        self._chair_name = self.get_parameter('chair_name').get_parameter_value().string_value
        self._target_topic = self.get_parameter('target_offset').get_parameter_value().string_value

        self.create_subscription(PointStamped, f"/{self._chair_name}/{self._target_topic}", self._tracker_callback, 1)
        self._publisher = self.create_publisher(Twist, f'/{self._chair_name}/target_vel', 1)


    def _tracker_callback(self, msg):

        d = math.sqrt(msg.point.x * msg.point.x + msg.point.y * msg.point.y)
        theta = 180 * math.atan2(msg.point.y, msg.point.x) / math.pi


        twist_msg = Twist()
        if d > 1.0:
            twist_msg.linear.x = 0.5
        if theta > 15:
            self.get_logger().info(f'{self.get_name()} theta > 15')
            twist_msg.angular.z = 0.5
        if theta < -15:
            self.get_logger().info(f'{self.get_name()} theta < -15')
            twist_msg.angular.z = -0.5

        self.get_logger().info(f'{self.get_name()} d {d} theta {theta} {twist_msg}')
        self._publisher.publish(twist_msg)




def main(args=None):
    rclpy.init(args=args)
    node = ChairFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

