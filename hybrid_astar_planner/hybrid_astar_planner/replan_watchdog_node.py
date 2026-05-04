from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node


class ReplanWatchdogNode(Node):
    """
    Re-issue the latest goal to trigger Nav2 replanning.

    Triggers:
    - periodic refresh while a goal exists;
    - sustained zero-velocity stall detection.
    """

    def __init__(self) -> None:
        super().__init__("replan_watchdog_node")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("tick_hz", 1.0)
        self.declare_parameter("periodic_replan_sec", 2.5)
        self.declare_parameter("enable_periodic_replan", True)
        self.declare_parameter("zero_speed_linear_eps", 0.02)
        self.declare_parameter("zero_speed_angular_eps", 0.05)
        self.declare_parameter("zero_streak_threshold", 6)
        self.declare_parameter("replan_cooldown_sec", 8.0)
        self.declare_parameter("path_stale_for_stall_sec", 4.0)
        self.declare_parameter("goal_freshness_timeout_sec", 300.0)
        self.declare_parameter("log_debug", True)

        self._goal_topic = str(self.get_parameter("goal_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_topic").value)
        self._path_topic = str(self.get_parameter("path_topic").value)
        self._tick_hz = max(0.5, float(self.get_parameter("tick_hz").value))
        self._periodic_replan_sec = float(self.get_parameter("periodic_replan_sec").value)
        self._enable_periodic = bool(self.get_parameter("enable_periodic_replan").value)
        self._lin_eps = float(self.get_parameter("zero_speed_linear_eps").value)
        self._ang_eps = float(self.get_parameter("zero_speed_angular_eps").value)
        self._zero_streak_threshold = int(self.get_parameter("zero_streak_threshold").value)
        self._cooldown_sec = float(self.get_parameter("replan_cooldown_sec").value)
        self._path_stale_for_stall_sec = float(self.get_parameter("path_stale_for_stall_sec").value)
        self._goal_freshness_timeout = float(self.get_parameter("goal_freshness_timeout_sec").value)
        self._log_debug = bool(self.get_parameter("log_debug").value)

        self._last_goal: Optional[PoseStamped] = None
        self._last_goal_walltime: float = 0.0
        self._last_cmd: Optional[Twist] = None
        self._last_path: Optional[Path] = None
        self._last_path_walltime: float = 0.0
        self._had_motion_since_goal: bool = False
        self._zero_streak: int = 0
        self._last_replan_walltime: float = 0.0
        self._last_periodic_walltime: float = 0.0

        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal, 20)
        self.create_subscription(Twist, self._cmd_topic, self._on_cmd, 20)
        self.create_subscription(Path, self._path_topic, self._on_path, 20)
        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self.create_timer(1.0 / self._tick_hz, self._tick)

        self.get_logger().info(
            f"ReplanWatchdogNode active. goal='{self._goal_topic}' cmd='{self._cmd_topic}' path='{self._path_topic}' "
            f"periodic={self._enable_periodic} every={self._periodic_replan_sec:.1f}s "
            f"stall_threshold={self._zero_streak_threshold}"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        self._last_goal = msg
        self._last_goal_walltime = time.monotonic()
        self._had_motion_since_goal = False
        self._zero_streak = 0

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg
        if abs(msg.linear.x) > self._lin_eps or abs(msg.angular.z) > self._ang_eps:
            self._had_motion_since_goal = True

    def _on_path(self, msg: Path) -> None:
        self._last_path = msg
        self._last_path_walltime = time.monotonic()

    def _goal_is_fresh(self, now: float) -> bool:
        if self._last_goal is None:
            return False
        return (now - self._last_goal_walltime) <= self._goal_freshness_timeout

    def _emit_replan(self, reason: str, now: float) -> None:
        if self._last_goal is None:
            return
        goal = PoseStamped()
        goal.header.frame_id = self._last_goal.header.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = self._last_goal.pose
        self._goal_pub.publish(goal)
        self._last_replan_walltime = now
        if reason == "periodic":
            self._last_periodic_walltime = now
        self.get_logger().warn(
            f"replan_watchdog_triggered reason={reason} zero_streak={self._zero_streak}"
        )

    def _tick(self) -> None:
        now = time.monotonic()
        if not self._goal_is_fresh(now):
            return

        if self._enable_periodic and (now - self._last_periodic_walltime) >= self._periodic_replan_sec:
            if (now - self._last_replan_walltime) >= self._cooldown_sec:
                self._emit_replan("periodic", now)

        if self._last_cmd is None:
            return

        is_zero = abs(self._last_cmd.linear.x) < self._lin_eps and abs(self._last_cmd.angular.z) < self._ang_eps
        if is_zero:
            self._zero_streak += 1
        else:
            self._zero_streak = 0

        if not self._had_motion_since_goal:
            return

        path_is_stale = self._last_path_walltime <= 0.0 or (now - self._last_path_walltime) >= self._path_stale_for_stall_sec
        if (
            self._zero_streak >= self._zero_streak_threshold
            and path_is_stale
            and (now - self._last_replan_walltime) >= self._cooldown_sec
        ):
            self._emit_replan("zero_speed_stall", now)
            self._zero_streak = 0
        elif self._log_debug and self._zero_streak > 0 and self._zero_streak % 5 == 0:
            self.get_logger().info(
                f"replan_watchdog_pending zero_streak={self._zero_streak}/{self._zero_streak_threshold} "
                f"path_stale={path_is_stale}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReplanWatchdogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
