"""
Sensor processor node (pipeline node 1 of 5).

Subscribes to raw LiDAR scans, applies lightweight preprocessing suitable for
real-time use on a small delivery robot: invalid range handling and optional
temporal smoothing. Publishes a filtered ``sensor_msgs/LaserScan`` for the
obstacle tracker and Nav2 costmaps.

This isolates sensor quirks from downstream perception so those nodes can assume
consistent range bounds and finite values where possible.
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SensorProcessorNode(Node):
    """
    ROS 2 node: filter ``LaserScan`` and republish for the rest of the stack.

    Parameters
    ----------
    scan_topic : str
        Input scan topic (default ``scan``).
    scan_filtered_topic : str
        Output filtered scan topic (default ``scan_filtered``).
    median_window : int
        Odd number of bins for optional median filter across the ring (0 disables).
    replace_inf_with_max : bool
        If true, replace ``+inf`` with ``range_max`` for downstream algorithms that
        do not handle infinite ranges.
    """

    def __init__(self) -> None:
        super().__init__("sensor_processor_node")

        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("scan_filtered_topic", "scan_filtered")
        self.declare_parameter("median_window", 0)
        # By default we preserve +inf as NaN so downstream mappers can treat
        # cells beyond sensor range as unknown/uncharted unless the user
        # explicitly requests replacement with range_max.
        self.declare_parameter("replace_inf_with_max", False)

        self._scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self._out_topic = self.get_parameter("scan_filtered_topic").get_parameter_value().string_value
        self._median_window = max(0, self.get_parameter("median_window").get_parameter_value().integer_value)
        self._replace_inf = self.get_parameter("replace_inf_with_max").get_parameter_value().bool_value

        self._sub = self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self._pub = self.create_publisher(LaserScan, self._out_topic, 10)

        self.get_logger().info(
            f"SensorProcessorNode: '{self._scan_topic}' -> '{self._out_topic}' "
            f"(median_window={self._median_window})"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        """Copy ranges, sanitize, optionally smooth, and publish."""
        ranges = list(msg.ranges)
        n = len(ranges)
        if n == 0:
            return

        # Replace non-finite and out-of-bounds values.
        # If replace_inf is False we preserve +inf / out-of-range as NaN so
        # mappers that raycast can treat those cells as unknown. If True,
        # replace +inf with the sensor range (common for algorithms that
        # expect finite ranges).
        lo = msg.range_min
        hi = msg.range_max
        cleaned: List[float] = []
        for r in ranges:
            if math.isnan(r):
                # keep as NaN
                cleaned.append(float("nan"))
            elif math.isinf(r):
                # positive infinity indicates no return; respect configuration
                if self._replace_inf and r > 0:
                    cleaned.append(hi)
                else:
                    cleaned.append(float("nan"))
            elif r < lo or r > hi:
                # out-of-bounds -> unknown
                cleaned.append(float("nan"))
            else:
                cleaned.append(r)

        if self._median_window >= 3 and self._median_window % 2 == 1:
            cleaned = self._median_ring(cleaned, self._median_window)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = cleaned
        out.intensities = list(msg.intensities) if msg.intensities else []
        self._pub.publish(out)

    @staticmethod
    def _median_ring(ranges: List[float], window: int) -> List[float]:
        """Apply 1D median filter on the circular range array."""
        n = len(ranges)
        half = window // 2
        extended = ranges[-half:] + ranges + ranges[:half]
        out = list(ranges)
        for i in range(n):
            chunk = sorted(extended[i : i + window])
            out[i] = chunk[half]
        return out


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SensorProcessorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
