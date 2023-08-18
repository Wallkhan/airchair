This repository contains code associated with the airchair project.

The goal here is to support N wheelchairs with different targets on them.
By default the chairs are named chair_a through chair_x. There is a launch file that just launches chair_a and another that launches a and b. Should be clear from those examples.

To build a fleet of chairs we need to first create the ArUco plate that goes behind each one. This plate is used to get the pose of the wheelchair that will be used in tracking and following. 
'aruco_board.py' creates an ArUco board using a given a marker seperation, marker length and specific ids. The current version creates a board with 0.01 m marker separation and each marker being 0.13 m in length. Once created, the necessary materials file must also be created and the '.png' board image must be placed with its '.materials' file in the urdf directory. These are then added to the 'scenario' json files.

The 'scenario' directory contains '.json' files that are used to create N wheelchairs with the required targets on their backs. Each 'scenario.json' file describes the number of chairs in the fleet, their respective ArUco board plates, their position in the world, a show video trigger and a camera tilting parameter. 
The file 'ten_chairs.json' contains arguments to create 10 wheelchairs each with their own targets. The convoy file 'ten_chair_convoy.json' contains the order the chairs are in the convoy. To launch the 10 chairs change into the 'scenario' directory and use 
'ros2 launch chair scenario.launch.py chairs:=scenario/ten_chairs.json convoy:=ten_chair_convoy.json' 
command.

To drive them around with the keyboard, use


ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=chair_a/cmd_vel 

