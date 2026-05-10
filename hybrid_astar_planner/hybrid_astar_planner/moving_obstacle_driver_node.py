"""Constant-velocity driver for the warehouse moving_box obstacle.

The Gazebo VelocityControl plugin on the moving_box model subscribes to
``/moving_box/cmd_vel`` (bridged to ``/model/moving_box/cmd_vel`` on the
Gazebo side). This node publishes a constant linear velocity along the y
axis and flips the sign once the box, observed via the bridged
``/moving_box/odometry`` topic, comes within a configurable distance of the
upcoming waypoint.

This replaces the earlier TrajectoryFollower-based motion: the latter is
force-based and, combined with Coulomb friction, produces unbounded
acceleration along the corridor and significant overshoot at the
waypoints. A direct kinematic command is more appropriate for a simple
moving obstacle whose only role is to interact with the lidar/costmap.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MovingObstacleDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("moving_obstacle_driver_node")

        self.declare_parameter("cmd_vel_topic", "/moving_box/cmd_vel")
        self.declare_parameter("odometry_topic", "/moving_box/odometry")
        self.declare_parameter("speed_mps", 0.5)
        self.declare_parameter("waypoint_y_min", -8.0)
        self.declare_parameter("waypoint_y_max", 8.0)
        self.declare_parameter("reverse_distance_m", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("initial_direction", 1)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._odom_topic = str(self.get_parameter("odometry_topic").value)
        self._speed_mps = float(self.get_parameter("speed_mps").value)
        self._wp_y_min = float(self.get_parameter("waypoint_y_min").value)
        self._wp_y_max = float(self.get_parameter("waypoint_y_max").value)
        self._reverse_dist = max(0.05, float(self.get_parameter("reverse_distance_m").value))
        self._publish_rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        initial_dir = int(self.get_parameter("initial_direction").value)
        self._direction: int = 1 if initial_dir >= 0 else -1

        self._latest_y: Optional[float] = None

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odometry, 10)
        self.create_timer(1.0 / self._publish_rate, self._publish_cmd_vel)

        self.get_logger().info(
            "moving_obstacle_driver active "
            f"(cmd_vel='{self._cmd_vel_topic}', odom='{self._odom_topic}', "
            f"speed={self._speed_mps:.2f}mps, "
            f"y_range=[{self._wp_y_min:.1f},{self._wp_y_max:.1f}], "
            f"reverse_within={self._reverse_dist:.2f}m)"
        )

    def _on_odometry(self, msg: Odometry) -> None:
        self._latest_y = float(msg.pose.pose.position.y)
        self._maybe_flip_direction()

    def _maybe_flip_direction(self) -> None:
        if self._latest_y is None:
            return
        # Heading +y, approaching upper waypoint.
        if self._direction > 0 and self._latest_y >= (self._wp_y_max - self._reverse_dist):
            self._direction = -1
            self.get_logger().info(
                f"moving_obstacle_driver reverse -> -y (y={self._latest_y:.2f})"
            )
        # Heading -y, approaching lower waypoint.
        elif self._direction < 0 and self._latest_y <= (self._wp_y_min + self._reverse_dist):
            self._direction = 1
            self.get_logger().info(
                f"moving_obstacle_driver reverse -> +y (y={self._latest_y:.2f})"
            )

    def _publish_cmd_vel(self) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = self._direction * self._speed_mps
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self._cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MovingObstacleDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
