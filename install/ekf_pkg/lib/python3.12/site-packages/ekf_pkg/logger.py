import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import csv
import os
import datetime
import numpy as np

class Logger(Node):
    def __init__(self):
        super().__init__('logger')
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.cov = None        
        self.robot_received = False
        
        self.K = None
        self.innovation = None
        self.param_received = False
        
        self.v = 0.0
        self.omega = 0.0
        
        # Subscribe to ekf to get current Pose
        self.ekf_sub = self.create_subscription(                   # subscribe to ekf
            PoseWithCovarianceStamped,
            '/ekf_pred',
            self.ekf_callback,
            100
        )
        
        # Subscribe to ekf parameters
        self.ekf_param_sub = self.create_subscription(
            Float64MultiArray,
            '/ekf_param',
            self.ekf_param_callback,
            10
        )
        
        # Subscribe to velocities
        self.ekf_vel_sub = self.create_subscription(
            Vector3,
            '/ekf_vel',
            self.ekf_vel_callback,
            10
        )
        
        logs_dir = os.path.expanduser('~/apf_ws/my_logs/ekf_logs')
        os.makedirs(logs_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'ekg_log_{timestamp}.csv'
        filepath = os.path.join(logs_dir, filename)
        
        self.file = open(filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time',
            'x', 'y', 'yaw', 'cov_prop', 'K_gain', 'innovation', 'v_ekf', 'omega_ekf'
        ])

        self.timer = self.create_timer(
            0.05,   # 20 Hz
            self.log_data
        )

    
    def ekf_callback(self, msg):
    
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.yaw = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        self.cov = msg.pose.covariance
        self.robot_received = True
    
    def ekf_param_callback(self, msg):
    
        data = np.array(msg.data)
        self.K = data[:4].reshape(2,2)
        self.innovation = data[9:12]
        self.param_received = True
    
    def ekf_vel_callback(self, msg):
        
        self.v = msg.x
        self.omega = msg.y
        
    def log_data(self):
        if not (self.robot_received and self.param_received):
            return

        time = self.get_clock().now().nanoseconds / 1e9

        self.writer.writerow([
            time,
            self.x, self.y, self.yaw, self.cov, self.K, self.innovation, self.v, self.omega
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
