import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

import csv
import math


class LidarLogger(Node):

    def __init__(self):
        super().__init__("lidar_logger")

        self.raw_file = open("scan.csv", "w", newline="")
        self.filtered_file = open("scan_filtered.csv", "w", newline="")

        self.raw_writer = csv.writer(self.raw_file)
        self.filtered_writer = csv.writer(self.filtered_file)

        self.raw_writer.writerow(["x", "y"])
        self.filtered_writer.writerow(["x", "y"])

        self.raw_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.raw_callback,
            qos_profile_sensor_data
        )

        self.filtered_sub = self.create_subscription(
            LaserScan,
            "/scan_filtered",
            self.filtered_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("LiDAR logger started.")

    def scan_to_xy(self, msg):

        points = []

        for i, r in enumerate(msg.ranges):

            # Reject invalid measurements
            if not math.isfinite(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            if r <= 0.0:
                continue

            theta = msg.angle_min + i * msg.angle_increment

            x = r * math.cos(theta)
            y = r * math.sin(theta)

            points.append((x, y))

        return points

    def raw_callback(self, msg):

        points = self.scan_to_xy(msg)

        for x, y in points:
            self.raw_writer.writerow([x, y])

        self.raw_file.flush()

    def filtered_callback(self, msg):

        points = self.scan_to_xy(msg)

        for x, y in points:
            self.filtered_writer.writerow([x, y])

        self.filtered_file.flush()

    def destroy_node(self):

        self.raw_file.close()
        self.filtered_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = LidarLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

