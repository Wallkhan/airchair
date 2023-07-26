# airchair
Air Chair project

The chair project is a colcon 'pure python' ros2 package. The chair_interface provides an interface to 
the chair and contains the srv/msg.

A sample scenario can be found in chair_ws/scenario
Scenarios can be launched using

ros2 launch chair scenario.launch.py chairs:=one_chair.json  convoy:=one_chair_convoy.json

where the chairs and convoy files are appropriate paths to those files.

