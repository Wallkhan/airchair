from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chair',
            executable='openpose_node',
            name='openpose_node'
        ),

        Node(
            package='chair',
            executable='opencv_camera',
            name='opencv_camera'
        ),

        Node(
            package='chair',
            executable='openpose_view',
            name='openpose_view',
            output='screen'
        )
    ])