import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from collections import deque


class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')

        # ---- parameters ----
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_filtered')
        self.declare_parameter('range_min_override', -1.0)
        self.declare_parameter('range_max_override', -1.0)
        self.declare_parameter('disable_range_gate', True)
        self.declare_parameter('mad_window', 5)
        self.declare_parameter('mad_k', 3.5)
        self.declare_parameter('isolation_radius', 0.06)
        self.declare_parameter('isolation_min_neighbors', 1)
        self.declare_parameter('isolation_arc_factor', 3.0)
        self.declare_parameter('use_temporal_filter', False)
        self.declare_parameter('temporal_window', 3)
        self.declare_parameter('temporal_max_disagree', 2)

        p = self.get_parameter
        self.input_topic = p('input_topic').value
        self.output_topic = p('output_topic').value
        self.range_min_override = p('range_min_override').value
        self.range_max_override = p('range_max_override').value
        self.disable_range_gate = bool(p('disable_range_gate').value)
        self.mad_window = int(p('mad_window').value)
        self.mad_k = float(p('mad_k').value)
        self.isolation_radius = float(p('isolation_radius').value)
        self.isolation_min_neighbors = int(p('isolation_min_neighbors').value)
        self.isolation_arc_factor = float(p('isolation_arc_factor').value)
        self.use_temporal_filter = bool(p('use_temporal_filter').value)
        self.temporal_window = int(p('temporal_window').value)
        self.temporal_max_disagree = int(p('temporal_max_disagree').value)

        # rolling buffer of previous validity-masked range arrays (aligned
        # to fixed angle indices) for the temporal filter
        self._history = deque(maxlen=self.temporal_window)

        # LiDAR (and most sensor streams in ROS2) publish best-effort;
        # matching QoS avoids silently dropping the subscription
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(LaserScan, self.input_topic, self.scan_cb, qos)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)

        self.get_logger().info(
            f"lidar_filter_node up: '{self.input_topic}' -> '{self.output_topic}' "
            f"(mad_window={self.mad_window}, mad_k={self.mad_k}, "
            f"isolation_radius={self.isolation_radius}, temporal={self.use_temporal_filter})"
        )

    # ------------------------------------------------------------------
    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float64)
        n = ranges.shape[0]

        rmin = self.range_min_override if self.range_min_override > 0.0 else msg.range_min
        rmax = self.range_max_override if self.range_max_override > 0.0 else msg.range_max

        # ---- stage 1: validity gate ----
        if self.disable_range_gate:
            # open-area mode: don't cap at msg.range_max at all, only drop
            # non-physical readings (inf/nan/<=0). Genuine max-range returns
            # from an open area stay in.
            valid = np.isfinite(ranges) & (ranges > 0.0)
        else:
            valid = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
        c1 = int(valid.sum())

        # ---- stage 2: windowed MAD outlier rejection ----
        valid = self._mad_filter(ranges, valid)
        c2 = int(valid.sum())

        # ---- stage 3: isolated-point removal (Cartesian neighbor check) ----
        valid = self._isolation_filter(ranges, valid, msg.angle_min, msg.angle_increment)
        c3 = int(valid.sum())

        # ---- stage 4: optional temporal consistency check ----
        if self.use_temporal_filter:
            valid = self._temporal_filter(valid)
        else:
            # still push into history so the buffer is warm if toggled on later
            self._history.append(valid.copy())
        c4 = int(valid.sum())

        self.get_logger().debug(
            f"points valid: raw_gate={c1} after_mad={c2} after_isolation={c3} after_temporal={c4} (of {n})"
        )
        if c1 > 0 and c3 == 0:
            self.get_logger().warn(
                "isolation filter dropped every point that survived the gate/MAD stages - "
                "isolation_radius/isolation_arc_factor is likely too strict for this scan's range"
            )

        filtered = ranges.copy()
        filtered[~valid] = float('inf')

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = filtered.tolist()
        # intensities array must match length if present; pass through untouched
        out.intensities = list(msg.intensities) if msg.intensities else []

        self.pub.publish(out)

    # ------------------------------------------------------------------
    def _mad_filter(self, ranges: np.ndarray, valid: np.ndarray) -> np.ndarray:
        """Reject points whose range deviates too far from the median of a
        local sliding window, scaled by that window's MAD. Using a local
        window (rather than one MAD over the whole scan) lets the threshold
        adapt as expected range magnitude changes with viewing angle."""
        n = ranges.shape[0]
        w = self.mad_window
        if n == 0 or w <= 0:
            return valid

        out = valid.copy()
        idx_valid = np.where(valid)[0]
        # Work on the array of valid ranges only, with padding by edge
        # replication so points near the scan boundary still get a window.
        padded = np.pad(ranges, (w, w), mode='edge')
        padded_valid = np.pad(valid, (w, w), mode='edge')

        for i in idx_valid:
            lo = i  # i - w + w (offset by pad width w)
            window = padded[lo:lo + 2 * w + 1]
            window_mask = padded_valid[lo:lo + 2 * w + 1]
            wv = window[window_mask]
            if wv.size < 3:
                continue  # not enough neighbors to judge, leave as-is
            med = np.median(wv)
            mad = np.median(np.abs(wv - med))
            # 0.6745 makes MAD a consistent estimator of std under normality
            sigma = mad / 0.6745 if mad > 1e-9 else 1e-6
            if abs(ranges[i] - med) > self.mad_k * sigma:
                out[i] = False
        return out

    # ------------------------------------------------------------------
    def _isolation_filter(self, ranges: np.ndarray, valid: np.ndarray,
                           angle_min: float, angle_increment: float) -> np.ndarray:
        """Reject a point if it has too few Cartesian neighbors within an
        adaptive radius among its immediate angular neighbors. Catches
        single-ray spurious returns (dust, specular reflections) that pass
        a range-only check but are clearly not part of a real surface.

        The radius is NOT fixed: consecutive rays are angle_increment apart,
        so their Cartesian (arc) spacing grows with range (spacing ~=
        range * angle_increment). A fixed radius tuned for close-range noise
        would flag every legitimate far-away wall point as 'isolated', so
        the per-point radius is the larger of the configured floor and a
        multiple of that point's own expected arc spacing."""
        idx = np.where(valid)[0]
        if idx.size == 0:
            return valid

        angles = angle_min + idx.astype(np.float64) * angle_increment
        r = ranges[idx]
        xs = r * np.cos(angles)
        ys = r * np.sin(angles)
        # per-point adaptive radius: max(floor, arc_factor * expected spacing)
        radii = np.maximum(self.isolation_radius,
                            self.isolation_arc_factor * angle_increment * r)

        out = valid.copy()
        k = 3  # check up to 3 rays on each side (cheap local check, O(n))
        m = idx.size
        for j in range(m):
            neighbor_count = 0
            for d in range(1, k + 1):
                for jj in (j - d, j + d):
                    if 0 <= jj < m:
                        dist = np.hypot(xs[j] - xs[jj], ys[j] - ys[jj])
                        if dist <= radii[j]:
                            neighbor_count += 1
            if neighbor_count < self.isolation_min_neighbors:
                out[idx[j]] = False
        return out

    # ------------------------------------------------------------------
    def _temporal_filter(self, valid: np.ndarray) -> np.ndarray:
        """Only meaningful once a few frames have accumulated; if a ray was
        invalid in >= temporal_max_disagree of the recent frames, cut it
        even if this frame's checks passed it (guards against flicker)."""
        self._history.append(valid.copy())
        if len(self._history) < self._history.maxlen:
            return valid  # not enough history yet, don't over-filter early on

        out = valid.copy()
        stacked = np.stack(self._history, axis=0)  # (frames, n)
        disagree_count = np.sum(~stacked, axis=0)
        out &= disagree_count < self.temporal_max_disagree
        return out


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



'''
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
    
'''

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
