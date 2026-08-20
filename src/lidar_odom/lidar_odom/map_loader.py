import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import csv
import os


class MapLoaderNode(Node):

    def __init__(self):
        super().__init__('map_loader_node')

        self.declare_parameter('csv_path', os.path.expanduser('~/apf_ws/map.csv'))
        self.declare_parameter('frame_id', 'odom')   # matches ICP node's save frame
        self.declare_parameter('has_header', True)   # ICP save_map writes 'x,y' header

        csv_path = self.get_parameter('csv_path').value
        self.frame_id = self.get_parameter('frame_id').value
        has_header = self.get_parameter('has_header').value

        self.points = self.load_csv(csv_path, has_header)
        self.original_points = self.points.copy()               # to transform the map
        self.get_logger().info(f'Loaded {len(self.points)} points from {csv_path}')
        
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 0.0
        self.init_received = False
        self.map_transformed = False
        
        self.point_x = 0.0
        self.point_y = 0.0
        self.point_yaw = 0.0

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        self.init_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.init_pose_callback,
            10
        )
        
        self.point_sub = self.create_subscription(
            TransformStamped,
            '/icp/point',
            self.icp_point_callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2, 
            '/map_pointcloud', 
            qos
        )

        self.publish_map()
        self.get_logger().info('Map published once (latched via TRANSIENT_LOCAL QoS)')

    def load_csv(self, path, has_header):
        points = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            rows = list(reader)
        if has_header:
            rows = rows[1:]
        for row in rows:
        
            if not row:
                continue
                
            x = float(row[0])
            y = float(row[1])
            
            points.append((x, y, 0.0))   # z padded to 0, same as ICP's publish_map
        
        return points
    
    def init_pose_callback(self, msg):
        self.init_x = msg.pose.pose.position.x
        self.init_y = msg.pose.pose.position.y
        _, _, self.init_yaw = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        
        self.init_received = True
        
        self.get_logger().info(
            f'Initial pose received: '
            f'x={self.init_x:.3f}, '
            f'y={self.init_y:.3f}, '
            f'yaw={self.init_yaw:.3f}'
        )
        
        R = np.array([
                    [np.cos(self.init_yaw), -np.sin(self.init_yaw)],
                    [np.sin(self.init_yaw),  np.cos(self.init_yaw)]
                ])
        t = np.array([
            self.init_x, self.init_y
        ])
        
        # Transform ORIGINAL map
        transformed_points = []
        
        for x, y, z in self.original_points:
            p = np.array([x, y])
            p_transformed = R.T @ (p - t)
            
            transformed_points.append((p_transformed[0], p_transformed[1], 0.0))
        
        self.points = transformed_points
        self.publish_map()
        
        self.get_logger().info(
            'Map transformed according to initial pose and republished'
        )
        self.map_transformed = True
    
    def icp_point_callback(self, msg):
    
        if self.init_received and self.map_transformed:
            self.point_x = msg.transform.translation.x
            self.point_y = msg.transform.translation.y
            self.point_yaw = 2.0 * math.atan2(msg.transform.rotation.z, msg.transform.rotation.w)
            
            R = np.array([
                [np.cos(self.point_yaw), -np.sin(self.point_yaw)],
                [np.sin(self.point_yaw),  np.cos(self.point_yaw)]
            ])
            
            t = np.array([self.point_x, self.point_y])
            
            self.get_logger().info(
                f'Map pose received: '
                f'x={self.point_x:.3f}, '
                f'y={self.point_y:.3f}, '
                f'yaw={self.point_yaw:.3f}'
            )
            
            # Transform ORIGINAL map
            transformed_points = []
            
            for x, y, z in self.original_points:
                p = np.array([x, y])
                p_transformed = t + R @ p
                
                transformed_points.append((p_transformed[0], p_transformed[1], 0.0))
            
            self.points = transformed_points
            self.publish_map()
            
            self.get_logger().info(
                'Map transformed according to map pose and republished'
            )

    def publish_map(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        cloud_msg = point_cloud2.create_cloud_xyz32(header, self.points)
        self.pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapLoaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

