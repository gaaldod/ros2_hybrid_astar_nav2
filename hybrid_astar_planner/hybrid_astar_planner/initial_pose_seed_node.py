from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPoseSeedNode(Node):
    """Publish an initial pose a few times so AMCL can initialize reliably."""

    def __init__(self) -> None:
        super().__init__("initial_pose_seed_node")
        self.declare_parameter("pose_x", -8.0)
        self.declare_parameter("pose_y", -8.0)
        self.declare_parameter("pose_yaw", 0.0)
        self.declare_parameter("publish_count", 20)
        # Publish initial pose less frequently to avoid TF extrapolation warnings
        self.declare_parameter("publish_period_s", 2.0)

        self._pose_x = float(self.get_parameter("pose_x").value)
        self._pose_y = float(self.get_parameter("pose_y").value)
        self._pose_yaw = float(self.get_parameter("pose_yaw").value)
        self._publish_count = int(self.get_parameter("publish_count").value)
        period = float(self.get_parameter("publish_period_s").value)

        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._sent = 0
        self._timer = self.create_timer(period, self._publish_initial_pose)

    def _publish_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        # Use stamp=0 so TF resolves against latest available transform and
        # avoids startup extrapolation races in simulated time.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self._pose_x
        msg.pose.pose.position.y = self._pose_y
        msg.pose.pose.orientation.z = math.sin(self._pose_yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self._pose_yaw * 0.5)
        # Conservative covariance for startup localization.
        msg.pose.covariance[0] = 0.5
        msg.pose.covariance[7] = 0.5
        msg.pose.covariance[35] = 0.3
        self._pub.publish(msg)

        self._sent += 1
        if self._sent >= self._publish_count:
            self.get_logger().info("Initial pose seeding complete.")
            self._timer.cancel()
            self.destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InitialPoseSeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow Ctrl-C during local testing
        pass
    except ExternalShutdownException:
        # External ROS shutdown requested (e.g., from a launch manager)
        # Treat this as a clean shutdown, do not propagate the exception.
        pass
    finally:
        # Attempt best-effort cleanup.
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
