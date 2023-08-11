#!/usr/bin/env python3

import sys
import rclpy
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QTextEdit, QLineEdit

class RobotControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self._node = rclpy.create_node('robot_control_gui')
        self._topic_input_text = '/chair_a/commanded_vel' # Default topic 
        self._publisher = self._node.create_publisher(Twist, self._topic_input_text, 1)
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Robot Control')
        self.setGeometry(100, 100, 400, 300)
        self.setStyleSheet("background-color: white;")

        layout = QVBoxLayout()

        self._topic_input = QLineEdit(self)
        self._topic_input.setPlaceholderText('Enter robot topic')
        layout.addWidget(self._topic_input)

        self._text_box = QTextEdit(self)
        layout.addWidget(self._text_box)

        button_layout = QVBoxLayout()
        self._up_button = QPushButton('Up', self)
        self._down_button = QPushButton('Down', self)
        self._left_button = QPushButton('Left', self)
        self._right_button = QPushButton('Right', self)

        self._up_button.clicked.connect(lambda: self.move_robot('up'))
        self._down_button.clicked.connect(lambda: self.move_robot('down'))
        self._left_button.clicked.connect(lambda: self.move_robot('left'))
        self._right_button.clicked.connect(lambda: self.move_robot('right'))

        button_layout.addWidget(self._up_button)
        button_layout.addWidget(self._down_button)
        button_layout.addWidget(self._left_button)
        button_layout.addWidget(self._right_button)

        layout.addLayout(button_layout)

        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def move_robot(self, direction):
        self._topic_input_text = self._topic_input.text()
        if self._topic_input_text:
            twist_msg = Twist()
            if direction == 'up':
                twist_msg.linear.x = 1.0
            elif direction == 'down':
                twist_msg.linear.x = -1.0
            elif direction == 'left':
                twist_msg.angular.z = 1.0
            elif direction == 'right':
                twist_msg.angular.z = -1.0
            
            self._publisher.publish(twist_msg)
            self._text_box.append(f'Published {direction} command to {self._topic_input_text}')

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = RobotControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
