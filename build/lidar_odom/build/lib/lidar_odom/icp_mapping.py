import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import os
import datetime

def rot2d(theta: float):
    s = np.sin(theta)
    c = np.cos(theta)
    return np.array([ [c,-s],[s,c] ])

class ICP(Node):
    
    def __init__(self):
        super().__init__('icp_mapping_node')
        
        # LiDAR data
        self.ranges = None
        self.range_min = 0.0
        self.range_max = 0.0
        self.angles = None
        self.angle_increment = 0.0
        self.angle_min = 0.0
        self.angle_max = 0.0
        
        # LiDAR-frame points
        self.xl = None
        self.yl = None
        self.scan_l = None
        
        # World-frame points
        self.xw = None
        self.yw = None
        self.scan_w = None
        
        # Robot pose
        self.q_odom = np.array([0.0,0.0])
        self.yaw = 0.0
        self.robot_received = False
        
        # Accumulated map
        # self.map_points = []  # OLD: raw per-scan accumulation, no dedup

        # NEW: deduped map storage (voxel-grid based)
        self.voxel_size = 0.025  # 2.5cm resolution, tune to your needs
        self.map_voxels = set()      # occupied voxel keys
        self.map_points_list = []    # deduped (x, y) points
        
        # LiDAR subscriber
        self.lidar_data = self.create_subscription(
            LaserScan,
            '/scan_filtered',               # subscribe to filtered scan topic
            self.scan_callback,
            qos_profile_sensor_data
        )
        
        # Odometry subscriber
        self.odom_data = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # RViz PointCloud2 publisher
        self.map_pub = self.create_publisher(
            PointCloud2,
            '/map_points',
            10
        )
        
        self.get_logger().info("ICP mapping node started")
    
    def odom_callback(self, msg):
        self.q_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        _, _, self.yaw = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        self.robot_received = True

    # NEW: dedup insert helper
    def add_points_deduped(self, xw, yw):
        new_pts = []
        for x, y in zip(xw, yw):
            key = (round(x / self.voxel_size), round(y / self.voxel_size))
            if key not in self.map_voxels:
                self.map_voxels.add(key)
                new_pts.append((x, y))
        if new_pts:
            self.map_points_list.extend(new_pts)
    
    def scan_callback(self, msg):
        
        if not self.robot_received:
            return
            
        # Read Laser scan
        self.ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        
        # Angles of every range
        self.angles = self.angle_min + np.arange(len(self.ranges))*self.angle_increment
        
        # Remove invalid measurements
        valid = (
            np.isfinite(self.ranges) &
            (self.ranges >= msg.range_min) &
            (self.ranges <= msg.range_max)
        )
        
        self.ranges = self.ranges[valid]
        self.angles = self.angles[valid]
        
        # Convert polar coordinates to cartesian coordinates
        self.xl = self.ranges * np.cos(self.angles)
        self.yl = self.ranges * np.sin(self.angles)
        
        # Convert LiDAR frame to odometry frame (rotation + translation)
        pts_robot = np.array([self.xl, self.yl])          # shape (2, N)
        pts_world = rot2d(self.yaw) @ pts_robot + self.q_odom.reshape(2, 1)
        self.xw, self.yw = pts_world
        
        self.scan_l = np.column_stack((self.xl, self.yl))
        self.scan_w = np.column_stack((self.xw, self.yw))
        
        # OLD: add raw scan to accumulated map (no dedup)
        # self.map_points.append(self.scan_w)

        # NEW: add only new (non-redundant) points to the map
        self.add_points_deduped(self.xw, self.yw)
        
        # Publish accumulated map
        self.publish_map()
    
    def save_map(self, filename=None):
        # OLD: stacked per-scan arrays
        # all_points = np.vstack(self.map_points)
        # np.savetxt(filename, all_points, delimiter=',', header='x,y', comments='')
        # self.get_logger().info(f'Saved {len(all_points)} points to {filename}')
        
        maps_dir = os.path.expanduser('~/apf_ws/maps')
        os.makedirs(maps_dir, exist_ok=True)
        
        if filename is None:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'map_{timestamp}.csv'
        
        filepath = os.path.join(maps_dir, filename)

        # NEW: save deduped map points
        if not self.map_points_list:
            self.get_logger().info('No map points to save.')
            return
        all_points = np.array(self.map_points_list)
        
        np.savetxt(filepath, all_points, delimiter=',', header='x,y', comments='')
        self.get_logger().info(f'Saved {len(all_points)} points to {filename}')
    
    def publish_map(self):
        
        # OLD: combine per-scan list
        # if not self.map_points:
        #     return
        # all_points = np.vstack(self.map_points)

        # NEW: use deduped map list
        if not self.map_points_list:
            return
        all_points = np.array(self.map_points_list)
        
        # Get z-axis
        points = [
            (float(x), float(y), 0.0) for x, y, in all_points
        ]
        
        # PointCloud2 header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        
        # Because the points are expressed in odom coordinates
        # Change this when doing sensor fusion
        header.frame_id = 'odom'
        
        # Create PointCloud2
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header, points
        )
        
        # OLD: crashed on ragged scan lengths
        # self.get_logger().info(f"map_points shape: {np.asarray(self.map_points).shape}")

        # NEW: safe logging, no shape assumption
        self.get_logger().info(f"total map points: {all_points.shape[0]}")
        
        # Publish
        self.map_pub.publish(cloud_msg)
        
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ICP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

