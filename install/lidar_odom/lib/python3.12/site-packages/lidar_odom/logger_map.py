import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import csv
import os
import numpy as np
import datetime

class Logger(Node):
    def __init__(self):
        super().__init__('logger')
        self.q_odom = np.array([0.0,0.0])
        self.yaw_odom = 0.0

        self.robot_received = False
        
        # Subscribe to Odometry to get current Pose
        self.odom_sub = self.create_subscription(                   # subscribe to odometry
            Odometry,
            '/odom',
            self.odom_callback,
            100
        )
        
        logs_dir = os.path.expanduser('~/apf_ws/my_logs/lidar_odom_logs/')
        os.makedirs(logs_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'lidar_odom_log_{timestamp}.csv'
        filepath = os.path.join(logs_dir, filename)

        self.file = open(filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time',
            'x_odom', 'y_odom', 'yaw_odom',
        ])

        self.timer = self.create_timer(
            0.05,   # 20 Hz
            self.log_data
        )
    
    def odom_callback(self, msg):
    
        self.q_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        _, _, self.yaw_odom = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        self.robot_received = True

    def log_data(self):
        if not (
            self.robot_received
        ):
            return

        time = self.get_clock().now().nanoseconds / 1e9

        self.writer.writerow([
            time,
            self.q_odom[0], self.q_odom[1], self.yaw_odom
        ])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
