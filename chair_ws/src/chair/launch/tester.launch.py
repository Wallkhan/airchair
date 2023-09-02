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
    

    nodelist = []


    nodelist.append(
        Node(
            namespace = 'stupid',
            package = 'chair',
            executable = 'tester',
            name = 'tester',
            output='screen'
        )
    )
    return LaunchDescription(nodelist)
