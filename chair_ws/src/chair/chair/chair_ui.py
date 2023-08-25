# Provide a simulation of the 'on chair' user interface
# This would be something that would be used by the actual chair occupant
#

import sys
import rclpy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from chair_interface.srv import EStop
from chair_interface.srv import Engage
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QTextEdit, QLineEdit, QLabel, QGridLayout, QStyle
from PyQt5.QtGui import QPixmap, QPalette, QColor, QIcon
from PyQt5.QtCore import Qt, QTimer, QSize

# used in debugging, now not needed
class Color(QWidget):
    def __init__(self, color):
        super().__init__()
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)

class RobotControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self._node = rclpy.create_node('chair_ui')
        home = get_package_share_directory('chair')
        self._node.get_logger().info(f'{self._node.get_name()} You are loading from {home}/')
        self._green_led = QPixmap(f'{home}/led-green-on.png').scaled(24, 24)
        self._red_led = QPixmap(f"{home}/led-red-on.png").scaled(24, 24)
        self._black_led = QPixmap(f"{home}/led-off.jpg").scaled(24, 24)
        self._stop_icon = QPixmap(f"{home}/stop-96.png").scaled(96, 96)
        self._up_icon = QIcon(QPixmap(f"{home}/up-100.png"))
        self._down_icon = QIcon(QPixmap(f"{home}/down-100.png"))
        self._left_icon = QIcon(QPixmap(f"{home}/left-100.png"))   
        self._right_icon = QIcon(QPixmap(f"{home}/right-100.png"))
        self._diag_up_left_icon = QIcon(QPixmap(f"{home}/up-left-100.png"))
        self._diag_up_right_icon = QIcon(QPixmap(f"{home}/up-right-100.png"))
        self._diag_down_left_icon = QIcon(QPixmap(f"{home}/down-left-100.png"))
        self._diag_down_right_icon = QIcon(QPixmap(f"{home}/down-right-100.png"))

        self._node.declare_parameter('chair-name', "chair_a")
        self._chair = self._node.get_parameter('chair-name').get_parameter_value().string_value
        self._node.get_logger().info(f'{self._node.get_name()} We are controlling chair {self._chair}')

        self._publisher = self._node.create_publisher(Twist, f'/{self._chair}/commanded_vel', 1)
        self._node.create_subscription(String, f'/{self._chair}/chair_status', self._chair_status_callback, 1)

        client = self._node.create_client(EStop, f'/{self._chair}/estop')
        while not client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info(f'{self._node.get_name()} Waiting for /{self._chair}/estop')
        self._EStop_req = EStop.Request()
        self._EStop_cli = client

        self.init_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._timerCallback)
        self._timer.start(100)

    def _timerCallback(self):
        try:
            rclpy.spin_once(self._node)  # deal with subscription callbacks
        except Exception as e:
            print(f'rclpy spin_once fails (likely rclpy closed down) Exception {e}') 
            sys.exit()

    def _chair_status_callback(self, msg):  # NB self is a QMainWindow
        self._estop_led.setPixmap(self._black_led)
        self._manual_led.setPixmap(self._black_led)
        self._engaged_led.setPixmap(self._black_led)
        if msg.data == 'E-stop':
            self._estop_led.setPixmap(self._red_led)
        elif msg.data == 'Manual':
            self._manual_led.setPixmap(self._red_led)
        elif msg.data == 'Engaged':
            self._engaged_led.setPixmap(self._red_led)
        else:
            self._node.get_logger().info(f'{self._node.get_name()} got a message {msg}')

    def init_ui(self):
        self.setWindowTitle(f'{self._chair}')
        self.setGeometry(100, 100, 300, 300)
        self.setStyleSheet("background-color: white;")
        layout = QVBoxLayout()

        estop = QVBoxLayout()
        estop.addWidget(QLabel("Status"), alignment=Qt.AlignmentFlag.AlignHCenter)
        estop1 = QGridLayout()

        self._estop_led = QLabel()
        self._estop_led.setPixmap(self._black_led)
        estop1.addWidget(self._estop_led, 0, 0, alignment=Qt.AlignmentFlag.AlignHCenter)
        self._manual_led = QLabel()
        self._manual_led.setPixmap(self._black_led)
        estop1.addWidget(self._manual_led, 0, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        self._engaged_led = QLabel()
        self._engaged_led.setPixmap(self._black_led)
        estop1.addWidget(self._engaged_led, 0, 2, alignment=Qt.AlignmentFlag.AlignHCenter)
        estop1.addWidget(QLabel("EStop"), 1, 0, alignment=Qt.AlignmentFlag.AlignHCenter)
        estop1.addWidget(QLabel("Manual"), 1, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        estop1.addWidget(QLabel("Engaged"), 1, 2, alignment=Qt.AlignmentFlag.AlignHCenter)
        estop.addLayout(estop1)
        layout.addLayout(estop)

        buttons = QVBoxLayout()
        buttons.addWidget(QLabel("Reset EStop Status"), alignment=Qt.AlignmentFlag.AlignHCenter)
        resetEStop = QPushButton("Reset")
        buttons.addWidget(resetEStop, alignment=Qt.AlignmentFlag.AlignHCenter)
        resetEStop.clicked.connect(self._resetEStop)
        layout.addLayout(buttons)

        joystick = QGridLayout()
        self._up_button = QPushButton(self)
        self._up_button.setIcon(self._up_icon)
        self._up_button.setIcon(QIcon(self._up_icon))
        self._down_button = QPushButton(self)
        self._down_button.setIcon(QIcon(self._down_icon))
        self._left_button = QPushButton(self)
        self._left_button.setIcon(QIcon(self._left_icon))
        self._right_button = QPushButton(self)
        self._right_button.setIcon(QIcon(self._right_icon))
        self._diag_up_left_button = QPushButton(self)
        self._diag_up_left_button.setIcon(QIcon(self._diag_up_left_icon))
        self._diag_up_right_button = QPushButton(self)
        self._diag_up_right_button.setIcon(QIcon(self._diag_up_right_icon))
        self._diag_down_left_button = QPushButton(self)
        self._diag_down_left_button.setIcon(QIcon(self._diag_down_left_icon))
        self._diag_down_right_button = QPushButton(self)
        self._diag_down_right_button.setIcon(QIcon(self._diag_down_right_icon))
        self._stop_button = QPushButton(self)
        self._stop_button.setIcon(QIcon(self._stop_icon))
        joystick.addWidget(self._diag_up_left_button, 0, 0, alignment=Qt.AlignmentFlag.AlignHCenter)
        joystick.addWidget(self._up_button, 0, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        joystick.addWidget(self._diag_up_right_button, 0, 2, alignment=Qt.AlignmentFlag.AlignHCenter)
        joystick.addWidget(self._left_button, 1, 0, alignment=Qt.AlignmentFlag.AlignHCenter)
        joystick.addWidget(self._stop_button, 1, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        joystick.addWidget(self._right_button, 1, 2, alignment=Qt.AlignmentFlag.AlignHCenter)
        joystick.addWidget(self._diag_down_left_button, 2, 0, alignment=Qt.AlignmentFlag.AlignHCenter)
        joystick.addWidget(self._down_button, 2, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        joystick.addWidget(self._diag_down_right_button, 2, 2, alignment=Qt.AlignmentFlag.AlignHCenter)
        button_size = 40
        for button in [self._up_button, self._down_button, self._left_button, self._right_button,
                       self._diag_down_left_button, self._diag_down_right_button, 
                       self._diag_up_left_button, self._diag_up_right_button, self._stop_button]:
            button.setIconSize(QSize(button_size, button_size))
            button.setFixedSize(button_size, button_size)

        layout.addLayout(joystick)
        
        self._up_button.clicked.connect(lambda: self.move_robot('up'))
        self._down_button.clicked.connect(lambda: self.move_robot('down'))
        self._left_button.clicked.connect(lambda: self.move_robot('left'))
        self._right_button.clicked.connect(lambda: self.move_robot('right'))
        self._diag_up_left_button.clicked.connect(lambda: self.move_robot('diag_up_left'))
        self._diag_up_right_button.clicked.connect(lambda: self.move_robot('diag_up_right'))
        self._diag_down_left_button.clicked.connect(lambda: self.move_robot('diag_down_left'))
        self._diag_down_right_button.clicked.connect(lambda: self.move_robot('diag_down_right'))
        self._stop_button.clicked.connect(lambda: self.move_robot('stop'))
        
        buttons = QVBoxLayout()
        buttons.addWidget(QLabel("ESTOP"), alignment=Qt.AlignmentFlag.AlignHCenter)
        engageEStop = QPushButton("Engage")
        buttons.addWidget(engageEStop, alignment=Qt.AlignmentFlag.AlignHCenter)
        engageEStop.clicked.connect(self._engageEStop)
        layout.addLayout(buttons)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

    def _resetEStop(self):
        self._node.get_logger().info(f'{self._node.get_name()} resetting eStop')
        self._EStop_req = EStop.Request()
        self._EStop_req.estop = False
        self._future = self._EStop_cli.call_async(self._EStop_req) # ignoring the future

    def _engageEStop(self):
        self._node.get_logger().info(f'{self._node.get_name()} engagin eStop')
        self._EStop_req = EStop.Request()
        self._EStop_req.estop = True
        self._future = self._EStop_cli.call_async(self._EStop_req) # ignoring the future


    def move_robot(self, direction):
        self._node.get_logger().info(f'{self._node.get_name()} direction is {direction}')
        twist_msg = Twist()
        if direction == 'up':
            twist_msg.linear.x = 0.5
        elif direction == 'down':
            twist_msg.linear.x = -0.5
        elif direction == 'left':
            twist_msg.angular.z = 0.5
        elif direction == 'right':
            twist_msg.angular.z = -0.5
        elif direction.startswith('diag'):
            angle = math.pi / 4.0
            if 'up' in direction:
                twist_msg.linear.x = 0.5
            else:
                twist_msg.linear.x = -0.5
            if 'left' in direction:
                twist_msg.angular.z = angle
            else:
                twist_msg.angular.z = -angle
        elif direction == 'stop':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
        self._publisher.publish(twist_msg)
        self._node.get_logger().info(f'{self._node.get_name()} Published {direction} command to {self._chair}/commanded_vel')

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = RobotControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
