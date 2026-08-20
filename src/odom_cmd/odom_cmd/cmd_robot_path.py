import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class ClosedLoopDriver(Node):

    def __init__(self):
        super().__init__('closed_loop_driver')
        # Parameters
        self.declare_parameter('shape', 'circle')            # 'square' or 'circle'
        self.declare_parameter('num_loops', 5)
        self.declare_parameter('linear_speed', 0.15)          # m/s
        self.declare_parameter('angular_speed', 0.5)          # rad/s, used for square corner turns
        self.declare_parameter('side_length', 1.0)            # m, square only
        self.declare_parameter('radius', 0.5)                 # m, circle only
        self.declare_parameter('control_rate', 20.0)          # Hz
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('pause_between_segments', 0.0) # s, optional dwell (e.g. to let wheel jerk settle)
        self.declare_parameter('shutdown_when_done', False)

        self.shape = self.get_parameter('shape').value
        self.num_loops = int(self.get_parameter('num_loops').value)
        self.v = float(self.get_parameter('linear_speed').value)
        self.w_turn = float(self.get_parameter('angular_speed').value)
        self.side_length = float(self.get_parameter('side_length').value)
        self.radius = float(self.get_parameter('radius').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        # self.frame_id = self.get_parameter('frame_id').value
        self.pause_between = float(self.get_parameter('pause_between_segments').value)
        self.shutdown_when_done = bool(self.get_parameter('shutdown_when_done').value)

        if self.shape not in ('square', 'circle'):
            self.get_logger().warn(
                f"Unknown shape '{self.shape}', defaulting to 'square'."
            )
            self.shape = 'square'

        # Publisher
        self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # Build the segment plan 
        # Each segment: (linear_x, angular_z, duration_seconds)
        self.segments = self._build_segments()
        self.total_time = sum(seg[2] for seg in self.segments)
        self.get_logger().info(
            f"Planned {len(self.segments)} segments, shape='{self.shape}', "
            f"num_loops={self.num_loops}, total commanded time={self.total_time:.2f}s"
        )

        # Timer bookkeeping 
        self.dt = 1.0 / self.control_rate
        self.seg_index = 0
        self.seg_elapsed = 0.0
        self.done = False
        self.timer = self.create_timer(self.dt, self._tick)

    # ------------------------------------------------------------------
    def _build_segments(self):
        segments = []

        if self.shape == 'square':
            straight_duration = self.side_length / self.v
            turn_duration = (math.pi / 2.0) / self.w_turn
            for _ in range(self.num_loops):
                for _ in range(4):
                    segments.append((self.v, 0.0, straight_duration))
                    if self.pause_between > 0.0:
                        segments.append((0.0, 0.0, self.pause_between))
                    segments.append((0.0, self.w_turn, turn_duration))
                    if self.pause_between > 0.0:
                        segments.append((0.0, 0.0, self.pause_between))

        elif self.shape == 'circle':
            w_circle = self.v / self.radius
            loop_duration = (2.0 * math.pi * self.radius) / self.v * 2.0
            for _ in range(self.num_loops):
                segments.append((self.v, w_circle, loop_duration))
                if self.pause_between > 0.0:
                    segments.append((0.0, 0.0, self.pause_between))

        return segments

    # ------------------------------------------------------------------
    def _tick(self):
        if self.done:
            return

        if self.seg_index >= len(self.segments):
            self._publish(0.0, 0.0)
            self.get_logger().info("Closed-path trajectory complete. Robot stopped.")
            self.done = True
            if self.shutdown_when_done:
                self.timer.cancel()
                rclpy.shutdown()
            return

        v, w, duration = self.segments[self.seg_index]
        self._publish(v, w)

        self.seg_elapsed += self.dt
        if self.seg_elapsed >= duration:
            self.seg_index += 1
            self.seg_elapsed = 0.0

    # ------------------------------------------------------------------
    def _publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = self.frame_id
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Publish a final zero-velocity command on Ctrl+C so the robot
        # doesn't keep coasting on the last nonzero command.
        node._publish(0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# ----------------------------------------------------------------------
# Adapting to independent per-wheel TwistStamped topics
# ----------------------------------------------------------------------
# If your setup actually wants two separate topics (e.g. /left_wheel/cmd_vel
# and /right_wheel/cmd_vel, each a TwistStamped whose linear.x carries wheel
# surface speed) rather than a single body-twist topic:
#
#   1. Create two publishers instead of self.pub:
#         self.pub_left  = self.create_publisher(TwistStamped, 'left_wheel/cmd_vel', 10)
#         self.pub_right = self.create_publisher(TwistStamped, 'right_wheel/cmd_vel', 10)
#   2. In _publish(), convert (v, w) to wheel speeds using your track width L
#      and wheel radius r:
#         v_l = v - (w * L / 2.0)
#         v_r = v + (w * L / 2.0)
#      then publish v_l / v_r (or v_l/r, v_r/r if the topic expects angular
#      wheel speed) in msg.twist.linear.x for each publisher respectively.
#   3. Add `wheel_base` and `wheel_radius` as declared parameters.
#
# Say the word if this is actually what you need and I'll rewrite the
# publishing section accordingly.

