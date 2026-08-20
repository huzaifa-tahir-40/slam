import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
import math
import numpy as np
from nav_msgs.msg import Odometry

class ForceToCmdVel(Node):

    def __init__(self):
        super().__init__('force_to_cmd_vel')
        self.x_data = []
        self.y_data = []
        
        # APF forces
        self.U_att = np.array([0.0, 0.0])
        self.U_rep = np.array([0.0, 0.0])
        
        # Robot pose
        self.q = np.array([0.0, 0.0])
        self.yaw = 0.0
        # Subscribe to attractive forces
        self.att_sub = self.create_subscription(
            Vector3,
            '/U_att',
            self.att_callback,
            10
        )
        
        # Subscribe to repulsive forces
        self.rep_sub = self.create_subscription(
            Vector3,
            '/U_rep',
            self.rep_callback,
            10
        )
        
        # Subscribe to Odometry to get current Pose
        self.odom_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            '/ekf_pred',                            # to change to /ekf_pred
            self.odom_callback,
            10
        )
        # Controller parameters
        self.declare_parameter('Kv', 1.0)
        self.declare_parameter('Kw', 1.0)
        self.declare_parameter('alpha', 0.1)
        self.declare_parameter('move', False)
        self.Kv = self.get_parameter('Kv').value
        self.Kw = self.get_parameter('Kw').value
        self.alpha = self.get_parameter('alpha').value
        self.move = self.get_parameter('move').value
        self.Kd = 2.0
        self.prev_err_theta = 0.0
        self.err_theta_dot_filt = 0.0
        self.deriv_filt_alpha = 0.2  # low-pass factor for derivative term
        self.stop_threshold = 1e-6
        self.resume_threshold = 5e-6  # hysteresis band above stop_threshold
        self.stopped = False
        self.prev_time = self.get_clock().now()
        
        # Publish the required robot velocity
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        self.timer = self.create_timer(
            0.05,   # 20 Hz
            self.publish_cmd_vel
        )
    def odom_callback(self, msg):
        self.q = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        _, _, self.yaw = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
    
    def att_callback(self, msg):
        self.U_att = np.array([msg.x, msg.y])
    def rep_callback(self, msg):
        self.U_rep = np.array([msg.x, msg.y])
    def publish_cmd_vel(self):
        # Resultant APF force
        F = -(self.U_att + self.U_rep)
        force_mag = np.linalg.norm(F)

        if self.stopped:
            if force_mag < self.resume_threshold:
                cmd = TwistStamped()
                cmd.twist.linear.x = 0.0
                cmd.twist.linear.y = 0.0
                cmd.twist.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return
            else:
                self.stopped = False
        elif force_mag < self.stop_threshold:
            self.stopped = True
            cmd = TwistStamped()
            cmd.twist.linear.x = 0.0
            cmd.twist.linear.y = 0.0
            cmd.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.prev_err_theta = 0.0
            self.err_theta_dot_filt = 0.0
            return

        theta_goal = np.arctan2(F[1], F[0])
        err_theta = theta_goal - self.yaw
        err_theta = np.arctan2(np.sin(err_theta), np.cos(err_theta))
        # derivative term
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        dt = max(dt, 1e-3)  # avoid div-by-zero on first call / jitter
        err_theta_dot_raw = (err_theta - self.prev_err_theta) / dt
        # low-pass filter the derivative: raw atan2(F) near the goal is
        # extremely noise-sensitive as force_mag -> 0, and an unfiltered
        # derivative term feeds that noise straight into omega
        self.err_theta_dot_filt = (
            self.deriv_filt_alpha * err_theta_dot_raw
            + (1 - self.deriv_filt_alpha) * self.err_theta_dot_filt
        )
        v = min(self.Kv * force_mag, 0.2)
        # omega = np.clip(self.Kw * err_theta + self.Kd * self.err_theta_dot_filt, -5.0, 5.0)    # with Kd
        omega = np.clip(self.Kw * err_theta, -5.0, 5.0)         # without Kd
        self.prev_err_theta = err_theta
        self.prev_time = now
        cmd = TwistStamped()
        cmd.twist.linear.x = v
        cmd.twist.linear.y = 0.0
        cmd.twist.angular.z = omega
        self.cmd_pub.publish(cmd)
    
def main(args=None):
    rclpy.init(args=args)
    node = ForceToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

