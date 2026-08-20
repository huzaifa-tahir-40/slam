import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import os

def wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

class EKF(Node):
    
    def __init__(self):
        super().__init__("ekf_node")
        
        self.previous_time = None
        
        self.x_init = 0.0
        self.y_init = 0.0
        self.yaw_init = 0.0
        
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.yaw_odom = 0.0
        self.v_odom = 0.0
        self.omega_odom = 0.0
        
        self.ranges = None
        self.angles = None
        self.d = 0.0                # LiDAR offset from robot center
        
        # Nearest-neighbor threshold
        self.max_association_distance = 1.0
        
        # Map landmarks
        map_path = os.path.expanduser("~/apf_ws/maps/map.csv")
        self.landmarks = np.loadtxt(map_path, delimiter=",",skiprows=1)
        self.get_logger().info(f"loaded {len(self.landmarks)} map landmarks.")
        
        self.z = None           # actual measurement
        self.z_hat = None       # predicted measurement
        
        self.phi_L = 0.0
        self.phi_R = 0.0
        self.R = np.array([
            [0.04,0.0],
            [0.0,0.0006046]
        ])
        
        self.rL = 0.033
        self.rR = 0.033
        self.b  = 0.16
        
        self.Q = 1e-3 * np.array([[0.6417,0.1600],[0.1600,0.4150]])
        self.f_est = np.array([0.0,0.0,0.0])
        self.sigma = np.eye(3) * 1e-2
        
        self.measurements = []
        self.new_measurement = False            # set True in lidar_callback, used in filter_step
        
        self.innovation = None
        self.K = None
        self.C = None
        self.M = None
        
        self.v = 0.0
        self.omega = 0.0
        
        self.init_received = False
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.lidar_callback,
            qos_profile_sensor_data
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joints_callback,
            10
        )
        
        self.init_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.init_pose_callback,
            10
        )
        
        self.ekf_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/ekf_pred_map',
            10
        )
        
        self.ekf_param_pub = self.create_publisher(
            Float64MultiArray,
            '/ekf_param',
            10
        )
        
        self.ekf_vel_pub = self.create_publisher(
            Vector3,
            '/ekf_vel',
            10
        )
        
        # Filter loop
        self.filter_rate_hz = 50.0
        self.timer = self.create_timer(
            1.0 / self.filter_rate_hz,
            self.filter_step
        )
        
        self.get_logger().info("EKF node started.")
    
    def odom_callback(self, msg):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        _, _, self.yaw_odom = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        
        self.v_odom = msg.twist.twist.linear.x
        self.omega_odom = msg.twist.twist.angular.z
    
    def lidar_callback(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # Keep valid measurements only
        valid = (np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max))
        self.ranges = ranges[valid]
        self.angles = angles[valid]
        
        if len(self.ranges) == 0:
            return
        
        # Convert to cartesian
        lidar_points = np.column_stack([
            self.ranges * np.cos(self.angles),
            self.ranges * np.sin(self.angles)
        ])
        
        c = np.cos(self.f_est[2])
        s = np.sin(self.f_est[2])
        R = np.array([
            [c,-s],
            [s, c]
        ])
        
        lidar_positions = np.array([
            self.f_est[0] + self.d*c,
            self.f_est[1] + self.d*s
        ])
        
        map_points = lidar_positions + lidar_points @ R.T
        
        scan_measurements = []              # local list built fresh each scan
        
        # Nearest-neighbor data association
        for i, map_point in enumerate(map_points):
        
            distances = np.linalg.norm(
                self.landmarks - map_point,
                axis=1
            )
            
            landmark_idx = np.argmin(distances)
            nearest_distance = distances[landmark_idx]
            
            # Reject poor associations
            if nearest_distance > self.max_association_distance:
                continue
            
            landmark = self.landmarks[landmark_idx]
            
            # Measurement model
            dx = landmark[0] - (self.f_est[0] + self.d * np.cos(self.f_est[2]))
            dy = landmark[1] - (self.f_est[1] + self.d * np.sin(self.f_est[2]))
            predicted_range = np.sqrt(dx**2 + dy**2)
            
            predicted_bearing = np.arctan2(dy,dx) - self.f_est[2]
            predicted_bearing = wrap_angle(predicted_bearing)
            
            # Actual measurement
            self.z = np.array([
                self.ranges[i],
                self.angles[i]
            ])
            
            # Predicted measurement
            self.z_hat = np.array([
                predicted_range,
                predicted_bearing
            ])
            
            # innovation
            innovation = self.z - self.z_hat
            innovation[1] = wrap_angle(innovation[1])
            
            # jacobian of measurement w.r.t state
            C = np.array([
                [-dx/predicted_range, -dy/predicted_range, 0],
                [dy/predicted_range**2, -dx/predicted_range**2, -1]
            ])
            
            scan_measurements.append((innovation,C))
        
        if scan_measurements:
            self.measurements = scan_measurements
            self.new_measurement = True
            
        self.M = np.eye(2)
            
    def joints_callback(self, msg):
        left_idx = msg.name.index("wheel_left_joint")
        right_idx = msg.name.index("wheel_right_joint")
        self.phi_L = msg.velocity[left_idx]
        self.phi_R = msg.velocity[right_idx]
    
    def init_pose_callback(self, msg):
        self.x_init = msg.pose.pose.position.x
        self.y_init = msg.pose.pose.position.y
        _, _, self.yaw_init = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        self.init_received = True
        
        self.previous_time = None
    
    def ekf_prediction(self, dt):
    
        # Jacobians
        yaw = wrap_angle(self.f_est[2])
        a = self.phi_L*self.rL*np.sin(yaw)/2 + self.phi_R*self.rR*np.sin(yaw)/2
        b = self.phi_L*self.rL*np.cos(yaw)/2 + self.phi_R*self.rR*np.cos(yaw)/2
        
        Fk = np.array([
                    [1.0,0.0,-dt*a],
                    [0.0,1.0, dt*b],
                    [0.0,0.0, 1.0]
                ])
        Gk = np.array([
                    [dt*self.rR*np.cos(yaw)/2.0, dt*self.rL*np.cos(yaw)/2.0],
                    [dt*self.rR*np.sin(yaw)/2.0, dt*self.rL*np.sin(yaw)/2.0],
                    # [dt*self.rR/(2.0*self.b), -dt*self.rL/(2.0*self.b)]
                    [dt*self.rR/(self.b), -dt*self.rL/(self.b)]
                ])
        
        # state prediction
        self.v = (self.phi_L * self.rL + self.phi_R * self.rR) / 2.0
        self.omega = (self.phi_R * self.rR - self.phi_L * self.rL) / self.b
        
        # self.f_est[0] += self.v * np.cos(yaw) * dt
        # self.f_est[1] += self.v * np.sin(yaw) * dt
        # self.f_est[2] += self.omega * dt
        # self.f_est[0] = self.x_odom
        # self.f_est[1] = self.y_odom
        # self.f_est[2] = self.yaw_odom
        self.f_est[0] += self.v_odom * np.cos(yaw) * dt
        self.f_est[1] += self.v_odom * np.sin(yaw) * dt
        self.f_est[2] += self.omega_odom * dt
        self.f_est[2] = wrap_angle(self.f_est[2])
        
        # covariance prediction
        self.sigma = Fk @ self.sigma @ Fk.T + Gk @ self.Q @ Gk.T
    
    def ekf_correction(self):
        
        for innovation, C in self.measurements:
        
            S = C @ self.sigma @ C.T + self.M @ self.R @ self.M.T
            # Kalman gain
            K = self.sigma @ C.T @ np.linalg.inv(S)
            # Update the state
            self.f_est = self.f_est + K @ (innovation)
            self.f_est[2] = wrap_angle(self.f_est[2])
            # Update the covariance
            self.sigma = (np.eye(3) - K @ C) @ self.sigma
            
            self.K = K
            self.innovation = innovation
        
        self.measurements = []
        
    # ======================
    # Main Filter Loop
    #=======================
    
    def filter_step(self):
        
        current_time = self.get_clock().now()
        
        # First iteration
        if self.previous_time is None:
            
            if not self.init_received:
                return
            
            self.previous_time = current_time
            
            # Initialize EKF state
            self.f_est = np.array([
                self.x_init,
                self.y_init,
                self.yaw_init
            ])
            
            self.init_received = False
            
            return
        
        # Calculate dt
        dt = (current_time - self.previous_time).nanoseconds * 1e-9
        self.previous_time = current_time
        
        # Reject invalid timestep
        if dt <= 0.0:
            return
        
        # Prediction
        self.ekf_prediction(dt)
        if self.new_measurement:
            # Correction
            self.ekf_correction()
            self.new_measurement = False
        # Publish
        self.publish_pose()
    
    def publish_pose(self):
        
        # Publish Pose
        msg = PoseWithCovarianceStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.pose.pose.position.x = self.f_est[0]
        msg.pose.pose.position.y = self.f_est[1]
        msg.pose.pose.position.z = 0.0
        
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.f_est[2])
        
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        cov = np.zeros((6,6))
        cov[0,0] = self.sigma[0,0]          # x-x
        cov[0,1] = self.sigma[0,1]          # x-y
        cov[0,5] = self.sigma[0,2]          # x-yaw
        
        cov[1,0] = self.sigma[1,0]          # y-x
        cov[1,1] = self.sigma[1,1]          # y-y
        cov[1,5] = self.sigma[1,2]          # y-yaw
        
        cov[5,0] = self.sigma[2,0]          # yaw-x
        cov[5,1] = self.sigma[2,1]          # yaw-y
        cov[5,5] = self.sigma[2,2]          # yaw-yaw
        
        msg.pose.covariance = cov.flatten().tolist()
        
        self.ekf_pose_pub.publish(msg)
        
        if self.K is not None and self.innovation is not None:
            msg = Float64MultiArray()
            msg.data = np.concatenate([
                self.K.flatten(),
                self.innovation
            ]).tolist()
            self.ekf_param_pub.publish(msg)
        
        msg = Vector3()
        msg.x = self.v
        msg.y = self.omega
        msg.z = 0.0
        self.ekf_vel_pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = EKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
