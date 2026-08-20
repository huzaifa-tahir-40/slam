import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import csv
import os
import numpy as np

class Logger(Node):
    def __init__(self):
        super().__init__('logger')
        self.q = np.array([0.0,0.0])
        self.yaw = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.imu_ang_vel_covariance = None
        self.imu_linear_acc_covariance = None
        self.odom_pose_covariance = None
        self.odom_twist_covariance = None
        
        self.cmd_vel = TwistStamped()
        self.cmd_vel_received = False
        self.robot_received = False

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to Odometry to get current Pose
        self.odom_sub = self.create_subscription(                   # subscribe to odometry
            Odometry,
            '/odom',
            self.odom_callback,
            100
        )
        
        # Subscribe to joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # Subscribe to imu
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.file = open('odom_log_sim.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time',
            'v', 'omega', 'x', 'y', 'yaw', 'wL', 'wR', 'odom_pose_cov', 'odom_twist_cov',
            'imu_vx', 'imu_vy', 'imu_vz', 'imu_ax', 'imu_ay', 'imu_az',
            'imu_cov_vel', 'imu_cov_acc'
        ])

        self.timer = self.create_timer(
            0.05,   # 20 Hz
            self.log_data
        )

    def cmd_vel_callback(self, msg):
        if not self.cmd_vel_received:
            self.get_logger().info('DEBUG: first /cmd_vel message received')
        self.cmd_vel = msg
        self.cmd_vel_received = True
    
    def odom_callback(self, msg):
    
        self.q = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        _, _, self.yaw = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        self.odom_pose_covariance = msg.pose.covariance
        self.odom_twist_covariance = msg.twist.covariance
        self.robot_received = True
    
    def joint_states_callback(self, msg):
        
        left_idx = msg.name.index("wheel_left_joint")
        right_idx = msg.name.index("wheel_right_joint")
        self.left_wheel_vel = msg.velocity[left_idx]
        self.right_wheel_vel = msg.velocity[right_idx]
    
    def imu_callback(self, msg):
    
        self.vx = msg.angular_velocity.x
        self.vy = msg.angular_velocity.y
        self.vz = msg.angular_velocity.z
        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.linear_acceleration.z
        self.imu_ang_vel_covariance = msg.angular_velocity_covariance
        self.imu_linear_acc_covariance = msg.linear_acceleration_covariance

    def log_data(self):
        if not (
            self.cmd_vel_received and
            self.robot_received
        ):
            self.get_logger().info(
                f'DEBUG: waiting on -> '
                f'cmd_vel={self.cmd_vel_received}',
                throttle_duration_sec=1.0
            )
            return

        time = self.get_clock().now().nanoseconds / 1e9
        v = self.cmd_vel.twist.linear.x
        omega = self.cmd_vel.twist.angular.z

        self.writer.writerow([
            time,
            v, omega, self.q[0], self.q[1], self.yaw, 
            self.left_wheel_vel, self.right_wheel_vel, self.odom_pose_covariance, self.odom_twist_covariance,
            self.vx, self.vy, self.vz, self.ax, self.ay, self.az,
            self.imu_ang_vel_covariance, self.imu_linear_acc_covariance
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
