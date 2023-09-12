from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chair_pose',
            executable='pose_node',
            name='pose_node',
        ),

        Node(
            package='chair_pose',
            executable='camera',
            name='camera'
        ),
    ])