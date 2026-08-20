from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_cmd',
            executable='cmd_robot_path',
            name='cmd_robot_path'
        ),
        Node(
            package='odom_cmd',
            executable='logger',
            name='logger'
        )
    ])
