It has 5 packages,

`apf_pkg` starts the artificial potential field for navigation and obstacle avoidance.

`ros2 launch apf_pkg apf_launch.launch.py`

`lidar_pkg` starts lidar filtering, ICP algorithm for recursive map updates and pose estimation, and map loader for loading and updating the map.

`ros2 launch lidar_odom lidar_pkg.launch.py`

`odom_cmd` can command the robot to travel a path using either open-loop and closed-loop control. The following command will make robot start moving in a defined path (currently circle).

`ros2 launch odom_cmd odom_log.launch.py`

`ekf_pkg` starts Extended Kalman Filter localization, uses odometry, from /odom topic, and LiDAR reference for correction. There are two EKF nodes, one node uses pose reference from the ICP algorithm on the topic /icp/pose and the other EKF node compares LiDAR measurements in polar coordinates on a fixed frame with landmarks on the map in the correction step.

`ros2 launch ekf_pkg ekf_pkg.launch.py`

`ros2 launch ekf_pkg ekf_pkg_map.launch.py`

Because all packages work together to achieve the purpose, the robot_bringup package runs other packages simultaneously. Initial pose from RViz2 will need to be published to make things work.

`ros2 launch robot_bringup bringup.launch.py`

`ros2 launch robot_bringup bringup_map.launch.py`

`ros2 launch robot_bringup bringup_cmd.launch.py`

`ros2 launch robot_bringup bringup_cmd_no_icp.launch.py`

`ros2 launch robot_bringup start_cmd.launch.py`
