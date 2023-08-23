import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    chairs = [
        {'name': 'leader_chair', 'target': 'board', 'x': '0', 'y': '0', 'theta': '0', 'show_video': 'false', 'camera_tilt': '0.25'},
        {'name': 'chair_a', 'target': 'apriltag_36h10_0', 'x': '-2', 'y': '0', 'theta': '0', 'show_video': 'false', 'camera_tilt': '0.25'}
        ]
    print(chairs)
    urdf = os.path.join(get_package_share_directory('chair'), 'airchair.urdf.xacro')


    nodelist = []
    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    print(gazebo)
    q = IncludeLaunchDescription(gazebo)
    print(q)
    nodelist.append(q)

    for chair in chairs:
        print(chair)
        robot_desc = xacro.process_file(urdf, mappings={'name' : chair['name'], 'target' : chair['target'], 'show_video': chair['show_video'], 'camera_tilt': chair['camera_tilt']}).toxml()
        nodelist.append(
            Node(
                namespace = chair['name'],
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': False, 'robot_description': robot_desc, 'frame_prefix': chair['name'] + "/"}],
                arguments=[urdf])
            )
        nodelist.append(
            Node(
                namespace = chair['name'],
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", "/" + chair['name'] + "/robot_description",  "-entity",  chair['name'], "-x", chair['x'], '-y', chair['y'], '-Y', chair['theta']])
            )
    return LaunchDescription(nodelist)
