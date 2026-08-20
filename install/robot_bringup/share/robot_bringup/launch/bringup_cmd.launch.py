from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_odom'),
                'launch',
                'lidar_pkg.launch.py'
            )
        )
    )
    
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ekf_pkg'),
                'launch',
                'ekf_pkg.launch.py'
            )
        )
    )
    
    return LaunchDescription([
        lidar_launch,
        ekf_launch
    ])
