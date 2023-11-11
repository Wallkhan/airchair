# Provide a simulation of the 'on chair' user interface
# This would be something that would be used by the actual chair occupant
#

import sys
import rclpy
import math
import json
from rclpy import qos
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from chair_interface.srv import EStop
from chair_interface.srv import Engage
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QTextEdit, QLineEdit, QLabel, QGridLayout, QStyle, QComboBox
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
        self._orange_led = QPixmap(f"{home}/led-orange.png").scaled(24, 24)
        self._stop_icon = QPixmap(f"{home}/stop-96.png").scaled(96, 96)
        self._up_icon = QIcon(QPixmap(f"{home}/up-100.png"))
        self._down_icon = QIcon(QPixmap(f"{home}/down-100.png"))
        self._left_icon = QIcon(QPixmap(f"{home}/left-100.png"))   
        self._right_icon = QIcon(QPixmap(f"{home}/right-100.png"))
        self._diag_up_left_icon = QIcon(QPixmap(f"{home}/up-left-100.png"))
        self._diag_up_right_icon = QIcon(QPixmap(f"{home}/up-right-100.png"))
        self._diag_down_left_icon = QIcon(QPixmap(f"{home}/down-left-100.png"))
        self._diag_down_right_icon = QIcon(QPixmap(f"{home}/down-right-100.png"))

        self._node.declare_parameter('convoy_description', "convoy.json")
        convoy_description = self._node.get_parameter('convoy_description').get_parameter_value().string_value
        self.convoy_description = self._load_json(convoy_description)['chairs']
        self._controlled_chair = self.convoy_description[0]

        # for every chair, 
        self._publisher = {}
        self._estop =  {}
        self._engage = {}
        self._node.get_logger().info(f'{self._node.get_name()} all chairs {self.convoy_description}')
        for chair in self.convoy_description:
            self._node.get_logger().info(f'{self._node.get_name()} Creating links for chair {chair}')
            self._publisher[chair] = self._node.create_publisher(Twist, f'/{chair}/commanded_vel', 1)
            self._node.create_subscription(String, f'/{chair}/chair_status', self._generate_status_callback(chair), 1)

            self._node.create_subscription(Float32, f'/{chair}/target_status', self._generate_connection_callback(chair), 1)


            client = self._node.create_client(EStop, f'/{chair}/estop')
            while not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().info(f'{self._node.get_name()} Waiting for /{chair}/estop')
            self._estop[chair] = client

            client = self._node.create_client(Engage, f'/{chair}/engage')
            while not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().info(f'{self._node.get_name()} Waiting for /{chair}/engage')
            self._engage[chair] = client
            self._node.get_logger().info(f'{self._node.get_name()} Creating links for chair {chair} done')

        self.init_ui()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._timerCallback)
        self._timer.start(100)

    def _generate_status_callback(self, chair_name):
        return lambda msg : self._chair_status_callback(msg, chair_name)

    def _generate_connection_callback(self, chair_name):
        return lambda msg : self._chair_connection_callback(msg, chair_name)

    def _load_json(self, file):
        try:
            return json.load(open(file, 'r'))
        except Exception as e:
            self._node.get_logger().info(f'{self._node.get_name()} unable to load/parse json file {file} {e}')
            return None

    def _timerCallback(self):
        try:
            rclpy.spin_once(self._node)  # deal with subscription callbacks
        except Exception as e:
            print(f'rclpy spin_once fails (likely rclpy closed down) Exception {e}') 
            sys.exit()

    def _chair_status_callback(self, msg, chair):  
        """Display chair status"""
        self._modes[chair]['E-stop'].setPixmap(self._black_led)
        self._modes[chair]['Manual'].setPixmap(self._black_led)
        self._modes[chair]['Engaged'].setPixmap(self._black_led)
        try:
            self._modes[chair][msg.data].setPixmap(self._red_led)
        except Exception as e:
            self._node.get_logger().info(f'{self._node.get_name()} chair {chair} got an unknown status {msg}')

    def _selectChair(self, i):
        self._controlled_chair = self.convoy_description[i]
        self._node.get_logger().info(f'{self._node.get_name()} Now selecting chair {i} {self._controlled_chair}')

    def init_ui(self):
        self.setWindowTitle(f'Operator')
        self.setGeometry(100, 100, 300, 300)
        self.setStyleSheet("background-color: white;")
        layout = QVBoxLayout()

        layout.addWidget(QLabel("Convoy Status"), alignment=Qt.AlignmentFlag.AlignHCenter)
        self._strength = {}
        self._modes = {}
        for chair in self.convoy_description:
            tmp = QHBoxLayout()
            tmp.addWidget(QLabel(chair))
            strength = QGridLayout()
            self._strength[chair] = [None] * 5
            for i in range(5):
                self._strength[chair][i] = QLabel()
                self._strength[chair][i].setPixmap(self._black_led)
                strength.addWidget(self._strength[chair][i], 0, i, alignment=Qt.AlignmentFlag.AlignHCenter)
            tmp.addLayout(strength)

            mode = QGridLayout()
            self._modes[chair] = {}
            mode.addWidget(QLabel(" STP "), 0, 0)
            self._modes[chair]['E-stop'] = QLabel()
            self._modes[chair]['E-stop'].setPixmap(self._black_led)
            mode.addWidget(self._modes[chair]['E-stop'], 0, 1)
            mode.addWidget(QLabel(" MAN "), 0, 2)
            self._modes[chair]['Manual'] = QLabel()
            self._modes[chair]['Manual'].setPixmap(self._black_led)
            mode.addWidget(self._modes[chair]['Manual'], 0, 3)
            mode.addWidget(QLabel(" ENG "), 0, 4)
            self._modes[chair]['Engaged'] = QLabel()
            self._modes[chair]['Engaged'].setPixmap(self._black_led)
            mode.addWidget(self._modes[chair]['Engaged'], 0, 5)
            
            tmp.addLayout(mode)
            layout.addLayout(tmp)
        layout.addWidget(QLabel(""), alignment=Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(QLabel("Chair Control"), alignment=Qt.AlignmentFlag.AlignHCenter)
        self._cb = QComboBox()
        for chair in self.convoy_description:
            self._cb.addItem(chair)
        self._cb.currentIndexChanged.connect(self._selectChair)
        layout.addWidget(self._cb)
        layout.addWidget(QLabel(""), alignment=Qt.AlignmentFlag.AlignHCenter)

        followButton = QPushButton("Follow")
        layout.addWidget(followButton, alignment=Qt.AlignmentFlag.AlignHCenter)
        followButton.clicked.connect(self._requestFollow)
        layout.addWidget(QLabel(""), alignment=Qt.AlignmentFlag.AlignHCenter)

        layout.addWidget(QLabel("Manual Control"), alignment=Qt.AlignmentFlag.AlignHCenter)
        buttons = QVBoxLayout()
        resetEStop = QPushButton("Enable")
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
        engageEStop = QPushButton("STOP Wheelchair")
        buttons.addWidget(engageEStop, alignment=Qt.AlignmentFlag.AlignHCenter)
        engageEStop.clicked.connect(self._engageEStop)
        layout.addLayout(buttons)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

#        self.setFixedSize(self.size())

    def _requestFollow(self):
        req = Engage.Request()
        req.engage = True
        self._future = self._engage[self._controlled_chair].call_async(req) # ignoring the future

    def _resetEStop(self):
        req = EStop.Request()
        req.estop = False
        self._future = self._estop[self._controlled_chair].call_async(req) # ignoring the future

    def _engageEStop(self):
        req = EStop.Request()
        req.estop = True
        self._future = self._estop[self._controlled_chair].call_async(req) # ignoring the future

    def _chair_connection_callback(self, msg, chair):
        thresholds = [0.0, 0.2, 0.4, 0.6, 0.8]
        
        for i in range(5):
            self._strength[chair][i].setPixmap(self._black_led)
        for i in range(5):
            if msg.data >= thresholds[i]:
                self._strength[chair][i].setPixmap(self._red_led)
                if (i >= 1) and (i < 3):
                    self._strength[chair][i].setPixmap(self._orange_led)
                if i >= 3:
                    self._strength[chair][i].setPixmap(self._green_led)
        
        if msg.data < thresholds[0]:
            self._node.get_logger().info(f'{self._node.get_name()} got bad connection strength chair {chair} {msg}')

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
                twist_msg.linear.x = 0.30
            else:
                twist_msg.linear.x = -0.30
            if 'left' in direction:
                twist_msg.angular.z = angle
            else:
                twist_msg.angular.z = -angle
        elif direction == 'stop':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
        self._publisher[self._controlled_chair].publish(twist_msg)
        self._node.get_logger().info(f'{self._node.get_name()} Published {direction} command to {self._controlled_chair}/commanded_vel')

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = RobotControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
