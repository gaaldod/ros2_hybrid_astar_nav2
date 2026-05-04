from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MotionReasonListener(Node):
    """
    Lightweight runtime listener to explain robot motion decisions.

    It does not replace planner/controller internals; it infers likely causes
    from the most relevant public topics.
    """

    def __init__(self) -> None:
        super().__init__("motion_reason_listener")

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("cmd_safe_topic", "/cmd_vel_safe")
        self.declare_parameter("cmd_nav_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_smoothed_topic", "/cmd_vel_smoothed")
        self.declare_parameter("speed_scale_topic", "/nav_speed_scale")
        self.declare_parameter("emergency_stop_topic", "/nav_emergency_stop")
        self.declare_parameter("report_period_s", 1.0)

        path_topic = str(self.get_parameter("path_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        cmd_safe_topic = str(self.get_parameter("cmd_safe_topic").value)
        cmd_nav_topic = str(self.get_parameter("cmd_nav_topic").value)
        cmd_smoothed_topic = str(self.get_parameter("cmd_smoothed_topic").value)
        scale_topic = str(self.get_parameter("speed_scale_topic").value)
        e_stop_topic = str(self.get_parameter("emergency_stop_topic").value)
        period = float(self.get_parameter("report_period_s").value)

        self._path: Optional[Path] = None
        self._odom: Optional[Odometry] = None
        self._cmd: Optional[Twist] = None
        self._cmd_safe: Optional[Twist] = None
        self._cmd_nav: Optional[Twist] = None
        self._cmd_smoothed: Optional[Twist] = None
        self._speed_scale: float = 1.0
        self._e_stop: bool = False

        self.create_subscription(Path, path_topic, self._on_path, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(Twist, cmd_topic, self._on_cmd, 20)
        self.create_subscription(Twist, cmd_safe_topic, self._on_cmd_safe, 20)
        self.create_subscription(Twist, cmd_nav_topic, self._on_cmd_nav, 20)
        self.create_subscription(Twist, cmd_smoothed_topic, self._on_cmd_smoothed, 20)
        self.create_subscription(Float32, scale_topic, self._on_scale, 20)
        self.create_subscription(Bool, e_stop_topic, self._on_estop, 20)
        self.create_timer(max(0.2, period), self._report)

        self.get_logger().info(
            f"MotionReasonListener active. Listening path='{path_topic}', odom='{odom_topic}', "
            f"cmd='{cmd_topic}', cmd_safe='{cmd_safe_topic}', cmd_nav='{cmd_nav_topic}', "
            f"cmd_smoothed='{cmd_smoothed_topic}'."
        )

    def _on_path(self, msg: Path) -> None:
        self._path = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = msg

    def _on_cmd_safe(self, msg: Twist) -> None:
        self._cmd_safe = msg

    def _on_cmd_nav(self, msg: Twist) -> None:
        self._cmd_nav = msg

    def _on_cmd_smoothed(self, msg: Twist) -> None:
        self._cmd_smoothed = msg

    def _on_scale(self, msg: Float32) -> None:
        self._speed_scale = float(msg.data)

    def _on_estop(self, msg: Bool) -> None:
        self._e_stop = bool(msg.data)

    def _report(self) -> None:
        if self._odom is None:
            self.get_logger().info("reason=waiting_for_odom")
            return
        if self._path is None or not self._path.poses:
            self.get_logger().info("reason=waiting_for_path")
            return

        cmd = (
            self._cmd_safe
            if self._cmd_safe is not None
            else self._cmd_smoothed
            if self._cmd_smoothed is not None
            else self._cmd_nav
            if self._cmd_nav is not None
            else self._cmd
        )
        if cmd is None:
            self.get_logger().info("reason=waiting_for_cmd")
            return

        x = self._odom.pose.pose.position.x
        y = self._odom.pose.pose.position.y
        yaw = _yaw_from_quat(self._odom.pose.pose.orientation)

        # Find nearest path point and a lookahead target.
        nearest_i = 0
        nearest_d2 = float("inf")
        for i, ps in enumerate(self._path.poses):
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            d2 = dx * dx + dy * dy
            if d2 < nearest_d2:
                nearest_d2 = d2
                nearest_i = i
        target_i = min(nearest_i + 6, len(self._path.poses) - 1)
        tx = self._path.poses[target_i].pose.position.x
        ty = self._path.poses[target_i].pose.position.y
        target_heading = math.atan2(ty - y, tx - x)
        heading_err = _norm_angle(target_heading - yaw)

        v = float(cmd.linear.x)
        w = float(cmd.angular.z)
        dist_to_path = math.sqrt(nearest_d2)

        if self._e_stop:
            reason = "emergency_stop_active"
        elif self._speed_scale < 0.05:
            reason = "speed_scale_zero_or_blocked"
        elif v < -0.02:
            reason = "reversing_for_path_alignment_or_recovery"
        elif abs(v) < 0.02 and abs(w) > 0.10:
            reason = "rotate_to_align_with_path"
        elif abs(heading_err) > 0.8:
            reason = "large_heading_error_turning_to_path"
        elif self._speed_scale < 0.5:
            reason = "slowed_by_local_obstacle_context"
        elif dist_to_path > 1.0:
            reason = "recovering_back_to_path"
        else:
            reason = "tracking_global_path_normally"

        self.get_logger().info(
            "reason=%s v=%.3f w=%.3f heading_err=%.2f dist_to_path=%.2f speed_scale=%.2f e_stop=%s"
            % (reason, v, w, heading_err, dist_to_path, self._speed_scale, self._e_stop)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionReasonListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
