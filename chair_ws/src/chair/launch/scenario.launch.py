import os
import json
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    chair_file = 'chairs.json'
    convoy_file = 'convoy.json'
    board_file = 'board.json'
    control = 'manual'
    for arg in sys.argv: # there must be a better way...
        if arg.startswith('chairs:='):
           print(arg.split('chairs:=', 1)[1])
           chair_file = arg.split('chairs:=', 1)[1]
        elif arg.startswith('convoy:='):
           convoy_file = arg.split('convoy:=', 1)[1]
        elif arg.startswith('board:='):
            board_file = arg.split('board:=', 1)[1]
            print(f"board file is {board_file}")
        elif arg.startswith('control:='):
            control = arg.split('control:=', 1)[1]
            if ['manual', 'leader'].count(control) != 1:
                print(f'control type of {control} not known, using manual')
                control = 'manual'
            print(f"control type is {control}")
        elif ':=' in arg:
           print(f"Unknown argument in {arg}. Usage ros2 launch sceario.launch.py [chairs:=chairs.json] [convoy:=convoy.json] [board:=board.json] [control:=[manual|leader]")
           sys.exit(0)
    print(f"Launching scenario from chair file {chair_file}, convoy {convoy_file}, board {board_file} with {control} control")
    try:
        chairs = json.load(open(chair_file, 'r'))
    except Exception as e:
        print(f"Unable to read/parse {chair_file} {e}")
        sys.exit(0)
    try:
        convoy = json.load(open(convoy_file, 'r'))
    except Exception as e:
        print(f"Unable to read/parse {convoy_file} {e}")
        sys.exit(0)
    try:
        boards = json.load(open(board_file, 'r'))
    except Exception as e:
        print(f"Unable to read/parse {board_file} {e}")
        sys.exit(0)
    print(f"All configuration files loaded {chairs}")
    

    nodelist = []

    # fire up gazebo
    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    q = IncludeLaunchDescription(gazebo)
    nodelist.append(q)

    # process each chair
    urdf = os.path.join(get_package_share_directory('chair'), 'airchair.urdf.xacro')

    chair_list = convoy['chairs']
    for i in range(len(chair_list)):
        print(f"Processing chair {i} {chair_list}")
        chair = chair_list[i]
        print(f"Processing {chair}")
        data = chairs[chair]
        print(f"Looking up {chair} resulted in {data}")
        robot_desc = xacro.process_file(urdf, mappings={'name' : chair, 'target' : data['target'], 'show_video': data['show_video'], 'camera_tilt': data['camera_tilt']}).toxml()

        nodelist.append(
            Node(
                namespace = chair,
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                output='screen',
                arguments=["0", "0", "0", "0", "0", "0", "1", "map", f"{chair}/odom"])
            )
        nodelist.append(
            Node(
                namespace = chair,
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': False, 'robot_description': robot_desc, 'frame_prefix': chair + "/"}],
                arguments=[urdf])
            )
        nodelist.append(
            Node(
                namespace = chair,
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", "/" + chair + "/robot_description",  "-entity",  chair, "-x", data['x'], '-y', data['y'], '-Y', data['theta']])
            )
        nodelist.append(
            Node(
                namespace = chair,
                package = 'chair',
                executable = 'chair_controller',
                name = 'chair_controller',
                output='screen',
                parameters=[{'convoy_description': convoy_file, 'chair_descriptions': chair_file, 'chair_name' : chair}])
            )
        nodelist.append(
            Node(
                namespace = chair,
                package = 'chair',
                executable = 'chair_ui',
                name = 'chair_ui',
                output='screen',
                parameters=[{'chair-name': chair}])
            )

        if i > 0: # not for first chair
            print(f"Processing chair {i}")
            following = chair_list[i-1]
            try:
                board =  chairs[chair_list[i-1]]['target']
            except Exception as e:
                print(f"No target entry in chair definition {chair_list[i-1]}")
                sys.exit(0)
            try:
                target = boards[board]
            except Exception as e:
                print(f"No board entry for board {board}")
                sys.exit(0)
            aruco_defn = json.dumps(target)
            print(f"chair {chair_list[i]} is following {chair_list[i-1]} with target {aruco_defn}")
            nodelist.append(
                Node(
                    namespace = chair,
                    package = 'chair',
                    executable = 'aruco_board_detect',
                    name = 'aruco_board_detect',
                    output='screen',
                    parameters=[{'chair_name': chair_list[i], 'board_definition': aruco_defn}])
                )
            nodelist.append(
                Node(
                    namespace = chair,
                    package = 'chair',
                    executable = 'chair_follower',
                    name = 'chair_follower',
                    output='screen',
                    parameters=[{'chair_name': chair_list[i]}])
                )


    if control == 'leader':
        print(f'Adding leader controller')
        nodelist.append(
            Node(
                namespace = chair_list[0],
                package = 'chair_pose',
                executable = 'camera',
                name = 'camera',
                output='screen')
            )
        nodelist.append(
            Node(
                namespace = chair_list[0],
                package = 'chair_pose',
                executable = 'pose_node',
                name = 'pose_node',
                output='screen',
                parameters=[{'chair-name': chair_list[0]}])
            )
        nodelist.append(
            Node(
                namespace = 'convoy',
                package = 'chair',
                executable = 'leader_ui',
                output = 'screen',
                parameters=[{'convoy_description': convoy_file}])
            )
 
    return LaunchDescription(nodelist)
