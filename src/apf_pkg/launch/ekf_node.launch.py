from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get package path
    pkg_path = FindPackageShare("apf_pkg")

    # EKF config file path
    ekf_config = PathJoinSubstitution(
        [pkg_path, "config", "ekf.yaml"]
    )

    # Robot Localization EKF Node
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": False}],
    )

    return LaunchDescription([
        robot_localization_node
    ])
