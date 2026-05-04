from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBridgeNode(Node):
    """Publish TF odom->base_link from odometry for Nav2 compatibility."""

    def __init__(self) -> None:
        super().__init__("odom_tf_bridge_node")
        self.declare_parameter("odom_topic", "odom_combined")
        self.declare_parameter("tf_parent_frame", "odom")
        self.declare_parameter("tf_child_frame", "base_link")
        # Prefer using the odometry message timestamp so TF history aligns with
        # sensor message stamps (especially when using simulated /clock).
        self.declare_parameter("use_latest_stamp", False)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._parent = str(self.get_parameter("tf_parent_frame").value)
        self._child = str(self.get_parameter("tf_child_frame").value)
        self._use_latest_stamp = bool(self.get_parameter("use_latest_stamp").value)

        self._tf_pub = TransformBroadcaster(self)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.get_logger().info(
            f"OdomTfBridgeNode: '{self._odom_topic}' -> TF '{self._parent}' -> '{self._child}'"
        )

    def _on_odom(self, msg: Odometry) -> None:
        tf = TransformStamped()
        # Publishing with the node's current clock avoids small lag-induced
        # extrapolation errors at startup when consumers request "latest".
        if self._use_latest_stamp:
            tf.header.stamp = self.get_clock().now().to_msg()
        else:
            tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self._parent
        tf.child_frame_id = self._child
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf_pub.sendTransform(tf)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = OdomTfBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

