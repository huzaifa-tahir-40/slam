from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ekf_pkg',
            executable='ekf_with_mapping',
            name='ekf_with_mapping'
        ),
        Node(
            package='ekf_pkg',
            executable='logger_map',
            name='logger_map'
        )
    ])
