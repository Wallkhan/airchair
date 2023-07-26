# airchair
Air Chair project

The chair project is a colcon 'pure python' ros2 package. The chair_interface provides an interface to 
the chair and contains the srv/msg.

A sample scenario can be found in chair_ws/scenario
Scenarios can be launched using

ros2 launch chair scenario.launch.py chairs:=one_chair.json  convoy:=one_chair_convoy.json

where the chairs and convoy files are appropriate paths to those files.

Assuming that the controller is running, you can set or release the estop button using

ros2 service call /chair_a/estop chair_interface/srv/EStop "{estop: True }"

When the estop button is pressed the chair is in ESTOP mode. When you release it it goes into manual mode. In manual mode you can command char_a using

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/chair_a/commanded_vel



