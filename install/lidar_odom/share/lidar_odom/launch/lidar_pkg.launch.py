from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_odom',
            executable='filter_lidar',
            name='filter_lidar'
        ),
        Node(
            package='lidar_odom',
            executable='icp_localization_node',
            name='icp_localization_node'
        ),
        Node(
            package='lidar_odom',
            executable='map_loader',
            name='map_loader'
        ),
        Node(
            package='lidar_odom',
            executable='logger',
            name='logger'
        )
    ])
