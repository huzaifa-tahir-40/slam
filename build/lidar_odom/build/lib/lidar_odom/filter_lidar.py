import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class FilterLiDAR(Node):

    def __init__(self):
        super().__init__("filter_lidar")

        self.window_size = 5
        self.mad_threshold = 1.0

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.lidar_pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            qos_profile_sensor_data
        )

    def lidar_callback(self, msg):

        # Keep complete raw LiDAR scan
        ranges = np.asarray(msg.ranges, dtype=float)

        filtered_ranges = ranges.copy()

        half_window = self.window_size // 2

        for i in range(len(ranges)):

            # Current measurement
            x = ranges[i]

            # Invalid raw measurement
            if not np.isfinite(x):
                filtered_ranges[i] = np.nan
                continue

            # Window boundaries
            start = max(0, i - half_window)
            end = min(len(ranges), i + half_window + 1)

            window = ranges[start:end]

            # Only use finite measurements for statistics
            window = window[np.isfinite(window)]

            if len(window) < 3:
                filtered_ranges[i] = np.nan
                continue

            # Median
            median = np.median(window)

            # Median Absolute Deviation
            mad = np.median(np.abs(window - median))

            # Avoid division/problem when all measurements
            # in the window are almost identical
            if mad < 1e-6:
                threshold = 0.05
            else:
                threshold = self.mad_threshold * 1.4826 * mad

            # Statistical outlier rejection
            if abs(x - median) > threshold:
                filtered_ranges[i] = np.nan

            # Reject negative values
            elif x <= 0.0:
                filtered_ranges[i] = np.nan

            # Reject values outside sensor limits
            elif x < msg.range_min or x > msg.range_max:
                filtered_ranges[i] = np.nan

        # --------------------------------------------------
        # Publish full LiDAR scan
        # --------------------------------------------------

        filtered_msg = LaserScan()

        filtered_msg.header = msg.header

        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment

        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time

        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        # FULL ORIGINAL INDEXING
        filtered_msg.ranges = filtered_ranges.tolist()

        filtered_msg.intensities = msg.intensities

        self.lidar_pub.publish(filtered_msg)


def main(args=None):

    rclpy.init(args=args)

    node = FilterLiDAR()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# Classical IIR High Pass filter was not working
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math

class FilterLiDAR(Node):
    
    def __init__(self):
        super().__init__("filter_lidar")
        
        self.ranges = None
        self.previous_ranges = None
        self.angles = None
        self.cutoff_frequency = 1.0
        self.previous_time = None
        self.previous_filtered = None
        self.previous_valid = None
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )
        
        self.lidar_pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            qos_profile_sensor_data
        )
    
    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # Angles of every range
        self.angles = angle_min + np.arange(len(ranges))*angle_increment
        
        # Remove invalid measurements
        valid = (
            np.isfinite(ranges) &
            (ranges >= msg.range_min) &
            (ranges <= msg.range_max)
        )
        
        ranges[~valid] = np.nan
        
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # First scan
        if self.previous_time is None:
            self.previous_time = current_time
            self.previous_ranges = ranges.copy()
            # self.previous_filtered = np.zeros_like(self.ranges)
            self.previous_filtered = np.full(len(ranges), np.nan, dtype=float)
            self.previous_valid = valid.copy()
            return
        
        # Calculate actual dt
        dt = current_time - self.previous_time
        self.previous_time = current_time
        
        if dt <= 0.0:
            return
        
        self.lidar_filter(msg, ranges, valid, dt)
    
    def lidar_filter(self, msg, ranges, valid, dt):
        
        # First-order IIR high-pass filter
        tau = 1.0 / (2.0 * np.pi * self.cutoff_frequency)
        
        alpha = tau / (tau + dt)
        
        # filtered_ranges = np.full_like(
        #     self.ranges,
        #     np.nan,
        #     dtype=float
        # )
        filtered_ranges = np.full(
            len(ranges), np.nan, dtype=float
        )
        
        # Use beam if BOTH previous and current measurements are valid at the SAME index
        filter_valid = (valid & self.previous_valid & np.isfinite(self.previous_ranges))
        
        # High-pass filter
        filtered_ranges[filter_valid] = alpha * (
            self.previous_filtered[filter_valid] + ranges[filter_valid] - self.previous_ranges[filter_valid]
        )
        
        # Reject negative values and values beyond range_max
        invalid_filtered = (
            ~np.isfinite(filtered_ranges) | (filtered_ranges <= 0.0) | (filtered_ranges > msg.range_max)
        )
        
        filtered_ranges[invalid_filtered] = np.nan
        
        # Filter only beams that are valid in both scans
        # filter_valid = valid & self.previous_valid
        
        # filtered_ranges[filter_valid] = alpha * (
        #     self.previous_filtered[filter_valid] + self.ranges[filter_valid] - self.previous_ranges[filter_valid]
        # )
        
        # filtered_ranges = alpha * (self.previous_filtered + self.ranges - self.previous_ranges)
        
        # Reject invalid filtered measurements
        # filtered_ranges[(filtered_ranges <= 0.0) | (filtered_ranges > msg.range_max)] = np.nan
        
        # Update filter states
        # self.previous_ranges[valid] = self.ranges[valid]
        # self.previous_valid = valid.copy()
        self.previous_ranges = ranges.copy()
        self.previous_valid = valid.copy()
        
        # self.previous_filtered[filter_valid] = filtered_ranges[filter_valid]
        self.previous_filtered = filtered_ranges.copy()
        
        # Create output LaserScan
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        
        # Preserve original beam indexing
        filtered_msg.ranges = filtered_ranges.tolist()
        filtered_msg.intensities = msg.intensities
        
        self.lidar_pub.publish(filtered_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = FilterLiDAR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
