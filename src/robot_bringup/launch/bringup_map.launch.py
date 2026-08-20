from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    apf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('apf_pkg'),
                'launch',
                'apf_launch.launch.py'
            )
        )
    )
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_odom'),
                'launch',
                'lidar_pkg_np_icp.launch.py'
            )
        )
    )
    
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ekf_pkg'),
                'launch',
                'ekf_pkg_map.launch.py'
            )
        )
    )
    
    return LaunchDescription([
        apf_launch,
        lidar_launch,
        ekf_launch
    ])
