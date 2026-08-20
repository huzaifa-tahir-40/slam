import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion

class SquareController(Node):

    def __init__(self):
        super().__init__("cmd_robot")
        
        self.num_squares = 20

        self.Kv = 0.5
        self.Kw = 0.7

        self.max_linear = 0.2
        self.max_angular = 0.7

        self.goal_tol = 0.02
        self.heading_tol = math.radians(1.5)

        # Publishers/Subscribers 
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        # Robot State

        self.pose_received = False
        self.initialized = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Mission

        self.current_corner = 0
        self.completed_squares = 0

        self.state = "ROTATE"

    @staticmethod
    def wrap(angle):
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.pose_received = True
        
        if not self.initialized:
            # Use absolute coordinates for the square
            self.goals = [
                (0.5, 0.0),    # Corner 1
                (0.5, 0.5),    # Corner 2
                (0.0, 0.5),    # Corner 3
                (0.0, 0.0)     # Corner 4 (back to start)
            ]
            
            self.initialized = True
            self.get_logger().info("Square initialized with absolute coordinates.")

    def publish_cmd(self, v, w):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = v
        msg.twist.angular.z = w

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def control_loop(self):

        if not self.pose_received or not self.initialized:
            return

        if self.completed_squares >= self.num_squares:
            self.stop_robot()
            return
        

        goal_x, goal_y = self.goals[self.current_corner]

        dx = goal_x - self.x
        dy = goal_y - self.y

        rho = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        alpha = self.wrap(desired_heading - self.yaw)

        # ---------------- ROTATE ----------------
        if self.state == "ROTATE":

            if abs(alpha) > self.heading_tol:

                w = self.Kw * alpha
                w = max(-self.max_angular, min(self.max_angular, w))

                self.publish_cmd(0.0, w)

            else:

                self.stop_robot()
                self.state = "MOVE"

                self.get_logger().info(
                    f"Moving to corner {self.current_corner + 1}"
                )

        elif self.state == "MOVE":

            # If heading error becomes too large, rotate first
            if abs(alpha) > math.radians(10.0):
                self.stop_robot()
                self.state = "ROTATE"
                return

            if rho > self.goal_tol:

                v = self.Kv * rho
                v = min(v, self.max_linear)

                # Proportional heading correction
                w = self.Kw * alpha
                w = max(-self.max_angular, min(self.max_angular, w))

                self.publish_cmd(v, w)

            else:

                self.stop_robot()

                self.current_corner += 1

                if self.current_corner >= len(self.goals):

                    self.current_corner = 0
                    self.completed_squares += 1

                    self.get_logger().info(
                        f"Completed square "
                        f"{self.completed_squares}/{self.num_squares}"
                    )

                    if self.completed_squares >= self.num_squares:
                        self.stop_robot()
                        self.get_logger().info("Mission Complete.")
                        return

                self.state = "ROTATE"

def main(args=None):

    rclpy.init(args=args)

    node = SquareController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
