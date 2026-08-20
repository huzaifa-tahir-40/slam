import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class APFNode(Node):
    
    def __init__(self):
        super().__init__('apf_node')
        
        self.robot_received = False
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 0.0
        self.q = np.array([0.0, 0.0])
        self.yaw = 0.0
        self.init_received = False
        
        # Read target pose from command line parameters
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.q_goal = np.array([self.goal_x, self.goal_y])
        self.goal_received = False
        
        # APF Parameters
        self.declare_parameter('K_att', 1.0)
        self.declare_parameter('K_rep', 0.5)
        self.declare_parameter('Rho_knot', 0.3)
        self.k_att = self.get_parameter('K_att').value
        self.k_rep = self.get_parameter('K_rep').value
        self.rho_knot = self.get_parameter('Rho_knot').value        # region of influence
        
        # Set parameters on run-time
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Subscribers
        self.min_distance_sub = self.create_subscription(           # from lidar
            LaserScan,
            '/scan_filtered',
            self.min_distance_callback,
            qos_profile_sensor_data
        )
        
        # Publishers
        self.u_att_pub = self.create_publisher(                     # to publish attractive potential gradient
            Vector3,
            '/U_att',
            10
        )
        
        self.u_rep_pub = self.create_publisher(                     # to publish repulsive potential gradient
            Vector3,
            '/U_rep',
            10
        )
        
        self.err_pub = self.create_publisher(
            Vector3,
            '/err_msg',
            10
        )
        
        # Subscribe to Odometry to get current Pose
        self.odom_sub = self.create_subscription(                   # subscribe to odometry
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscribe to goal-position
        self.goal_p = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Subscribe to initial-position
        self.init_p = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.init_pose_callback,
            10
        )
        
        self.lidar_data = None
        self.distance = 0.0
        self.angle = 0.0
        self.angle_increment = 0
        
        self.apply_init_pose = False
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'K_att':
                self.k_att = param.value
            elif param.name == 'K_rep':
                self.k_rep = param.value
            elif param.name == 'Rho_knot':
                self.rho_knot = param.value

        return SetParametersResult(successful=True)
    
    def min_distance_callback(self, msg):               # taking minimal obstacle distance in body frame
        ranges = list(msg.ranges)
        self.lidar_data = ranges
        valid_readings = [
            (distance, i) for i, distance in enumerate(ranges) if math.isfinite(distance)
        ]
        if not valid_readings:
            return
        # Minimum distance
        min_distance, min_index = min(valid_readings)
        self.angle_increment = msg.angle_increment
        self.distance = min_distance
        self.angle = msg.angle_min
    
    def odom_callback(self, msg):
        
        if self.apply_init_pose and self.init_received:
            self.q = np.array([self.init_x, self.init_y])
            self.yaw = self.init_yaw
            self.apply_init_pose = False
            self.robot_received = True
            self.calculate_apf()
            return
            
        self.q = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        _, _, self.yaw = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        self.calculate_apf()
    
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        _, _, self.goal_yaw = euler_from_quaternion([ msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w ])
        self.goal_received = True
        self.calculate_apf()
    
    def init_pose_callback(self, msg):
        self.init_x = msg.pose.pose.position.x
        self.init_y = msg.pose.pose.position.y
        _, _, self.init_yaw = euler_from_quaternion([ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        self.apply_init_pose = True
        self.init_received = True
    
    def attractive_potential(self):
    
        self.q_goal = np.array([self.goal_x, self.goal_y])
        # Distance from current pose to goal pose
        rho = np.linalg.norm(self.q - self.q_goal)
        # Goal threshold
        d = 0.05                                # Set to 0.15m away from goal
        # Attractive potential gradient
        if rho <= d:
            U_att_grad = self.k_att * (self.q - self.q_goal)
        elif rho > d:
            U_att_grad = d * self.k_att * (self.q - self.q_goal) / rho
            
        return U_att_grad
    
    def repulsive_potential(self):
    
        if self.lidar_data is None:
            return np.zeros(2)
        
        U_rep = np.zeros(2)

        for i, distance in enumerate(self.lidar_data):

            # Ignore invalid measurements
            if not math.isfinite(distance):
                continue

            # Ignore zero/negative measurements
            if distance <= 0.0:
                continue

            # Only obstacles inside the influence region contribute
            if distance > self.rho_knot:
                continue

            # LiDAR angle in robot/body frame
            ang = self.angle + i * self.angle_increment

            # Obstacle position relative to robot, body frame
            obstacle_x_b = distance * math.cos(ang)
            obstacle_y_b = distance * math.sin(ang)

            # Transform obstacle vector into world frame
            obstacle_x = (
                obstacle_x_b * math.cos(self.yaw)
                - obstacle_y_b * math.sin(self.yaw)
            )

            obstacle_y = (
                obstacle_x_b * math.sin(self.yaw)
                + obstacle_y_b * math.cos(self.yaw)
            )

            # Unit vector pointing from obstacle -> robot
            rho_grad = np.array([
                obstacle_x,
                obstacle_y
            ]) / distance

            # Repulsive force magnitude
            magnitude = (
                self.k_rep
                * (1.0 / distance - 1.0 / self.rho_knot)
                / (distance ** 2)
            )

            # Contribution of this LiDAR ray
            U_rep += magnitude * rho_grad
            
        return U_rep        
    
    def calculate_apf(self):
    
        if not self.robot_received or not self.goal_received:
            if not self.robot_received:
                self.get_logger().warn("Waiting for /initialpose before running APF", throttle_duration_sec=5.0)
            return

        U_att = self.attractive_potential()
        U_rep = self.repulsive_potential()
        
        # Publish attractive force
        att_msg = Vector3()
        att_msg.x = U_att[0]
        att_msg.y = U_att[1]
        att_msg.z = 0.0
        
        self.u_att_pub.publish(att_msg)
        
        # Publish repulsive force
        rep_msg = Vector3()
        rep_msg.x = U_rep[0]
        rep_msg.y = U_rep[1]
        rep_msg.z = 0.0
        
        self.u_rep_pub.publish(rep_msg)
        
        self.get_logger().info(
            f"Attractive: ({U_att[0]:.2f},{U_att[1]:.2f}) "
            f"Repulsive: ({U_rep[0]:.2f},{U_rep[1]:.2f}) "
            f"Goal: ({self.goal_x}, {self.goal_y}) "
            f"Robot: ({self.q[0]:.3f}, {self.q[1]:.3f})"
        )
        
        err_x = self.q[0] - self.goal_x
        err_y = self.q[1] - self.goal_y
        err_msg = Vector3()
        err_msg.x = err_x
        err_msg.y = err_y
        err_msg.z = 0.0
        self.err_pub.publish(err_msg)
    
def main(args=None):

    rclpy.init(args=args)
    node = APFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
