from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('apf_pkg'),
        'config',
        'apf.yaml'
    )
    return LaunchDescription([
        Node(
            package='apf_pkg',
            executable='apf_node',
            name='apf_node',
            parameters=[config]
        ),
        Node(
            package='apf_pkg',
            executable='apf_controller',
            name='apf_controller',
            parameters=[config]
        ),
        Node(
            package='apf_pkg',
            executable='logger',
            name='logger',
            parameters=[config]
        )
    ])
