from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import Bool


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ReverseRecoveryNode(Node):
    """
    Trigger a short reverse arc when Ackermann navigation stalls at sharp turns.
    """

    def __init__(self) -> None:
        super().__init__("reverse_recovery_node")

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("cmd_nav_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_safe_topic", "/cmd_vel_safe")
        self.declare_parameter("recovery_active_topic", "/nav_recovery_active")
        self.declare_parameter("recovery_cmd_topic", "/cmd_vel_recovery")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("pre_replan_enabled", False)
        self.declare_parameter("allow_goal_republish_pre_replan", False)
        self.declare_parameter("pre_replan_wait_sec", 1.2)
        self.declare_parameter("pre_replan_wait_zero_speed_sec", 3.0)
        self.declare_parameter("pre_replan_cooldown_sec", 2.0)
        self.declare_parameter("pre_replan_min_path_poses", 2)
        self.declare_parameter("pre_replan_fresh_path_age_sec", 2.5)
        self.declare_parameter("dist_to_path_trigger", 1.2)
        self.declare_parameter("heading_error_trigger", 0.8)
        self.declare_parameter("rotate_cmd_linear_eps", 0.02)
        self.declare_parameter("rotate_cmd_angular_eps", 0.10)
        self.declare_parameter("reverse_linear_speed", 0.18)
        self.declare_parameter("reverse_angular_speed", 0.35)
        self.declare_parameter("reverse_angular_speed_far", 0.12)
        self.declare_parameter("reverse_far_dist_threshold", 1.8)
        self.declare_parameter("maneuver_duration_sec", 1.4)
        self.declare_parameter("max_maneuver_duration_sec", 2.6)
        self.declare_parameter("maneuver_duration_heading_gain", 0.35)
        self.declare_parameter("trigger_cooldown_sec", 8.0)
        self.declare_parameter("stop_duration_sec", 0.5)
        self.declare_parameter("stuck_zero_speed_streak_threshold", 25)
        self.declare_parameter("stuck_dist_to_path_trigger", 0.8)
        self.declare_parameter("stuck_heading_error_trigger", 0.9)
        self.declare_parameter("goal_start_grace_sec", 10.0)
        self.declare_parameter("heading_stall_trigger", 1.10)
        self.declare_parameter("heading_stall_odom_speed_eps", 0.03)
        self.declare_parameter("heading_stall_streak_threshold", 18)
        self.declare_parameter("collision_warning_threshold", 2)
        self.declare_parameter("collision_window_sec", 3.0)
        self.declare_parameter("collision_immediate_trigger", True)
        self.declare_parameter("collision_immediate_window_sec", 0.4)
        self.declare_parameter(
            "collision_warning_substring", "RegulatedPurePursuitController detected collision ahead!"
        )
        self.declare_parameter("planner_abort_threshold", 2)
        self.declare_parameter("planner_abort_window_sec", 20.0)
        self.declare_parameter("planner_abort_substring", "compute_path_to_pose abort: planner_returned_none")
        self.declare_parameter("planner_logger_name", "nav2_hybrid_astar_server")
        self.declare_parameter("controller_logger_name", "controller_server")
        self.declare_parameter(
            "clear_local_costmap_service", "/local_costmap/clear_entirely_local_costmap"
        )
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("log_debug", True)
        self.declare_parameter("require_goal_before_trigger", True)
        self.declare_parameter("startup_grace_sec", 12.0)
        self.declare_parameter("reverse_min_duration_sec", 0.8)
        self.declare_parameter("reverse_hard_max_duration_sec", 12.0)
        self.declare_parameter("reverse_clear_dist_to_path_m", 0.65)
        self.declare_parameter("reverse_clear_heading_error_rad", 0.65)
        self.declare_parameter("reverse_collision_quiet_sec", 1.2)

        self._path_topic = str(self.get_parameter("path_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_nav_topic = str(self.get_parameter("cmd_nav_topic").value)
        self._cmd_safe_topic = str(self.get_parameter("cmd_safe_topic").value)
        self._active_topic = str(self.get_parameter("recovery_active_topic").value)
        self._cmd_topic = str(self.get_parameter("recovery_cmd_topic").value)
        self._goal_topic = str(self.get_parameter("goal_topic").value)
        self._pre_replan_enabled = bool(self.get_parameter("pre_replan_enabled").value)
        self._allow_goal_republish_pre_replan = bool(
            self.get_parameter("allow_goal_republish_pre_replan").value
        )
        self._pre_replan_wait_sec = max(0.1, float(self.get_parameter("pre_replan_wait_sec").value))
        self._pre_replan_wait_zero_speed_sec = max(
            self._pre_replan_wait_sec,
            float(self.get_parameter("pre_replan_wait_zero_speed_sec").value),
        )
        self._pre_replan_cooldown_sec = max(
            0.0, float(self.get_parameter("pre_replan_cooldown_sec").value)
        )
        self._pre_replan_min_path_poses = max(
            2, int(self.get_parameter("pre_replan_min_path_poses").value)
        )
        self._pre_replan_fresh_path_age_sec = max(
            0.1, float(self.get_parameter("pre_replan_fresh_path_age_sec").value)
        )
        self._dist_trigger = float(self.get_parameter("dist_to_path_trigger").value)
        self._heading_trigger = float(self.get_parameter("heading_error_trigger").value)
        self._lin_eps = float(self.get_parameter("rotate_cmd_linear_eps").value)
        self._ang_eps = float(self.get_parameter("rotate_cmd_angular_eps").value)
        self._rev_v = abs(float(self.get_parameter("reverse_linear_speed").value))
        self._rev_w = abs(float(self.get_parameter("reverse_angular_speed").value))
        self._rev_w_far = abs(float(self.get_parameter("reverse_angular_speed_far").value))
        self._reverse_far_dist_threshold = abs(
            float(self.get_parameter("reverse_far_dist_threshold").value)
        )
        self._maneuver_dt = float(self.get_parameter("maneuver_duration_sec").value)
        self._max_maneuver_dt = max(
            self._maneuver_dt, float(self.get_parameter("max_maneuver_duration_sec").value)
        )
        self._maneuver_heading_gain = max(
            0.0, float(self.get_parameter("maneuver_duration_heading_gain").value)
        )
        self._cooldown_dt = float(self.get_parameter("trigger_cooldown_sec").value)
        self._stop_dt = float(self.get_parameter("stop_duration_sec").value)
        self._stuck_zero_speed_streak_threshold = int(
            self.get_parameter("stuck_zero_speed_streak_threshold").value
        )
        self._stuck_dist_trigger = float(self.get_parameter("stuck_dist_to_path_trigger").value)
        self._stuck_heading_trigger = float(self.get_parameter("stuck_heading_error_trigger").value)
        self._goal_start_grace_sec = max(0.0, float(self.get_parameter("goal_start_grace_sec").value))
        self._heading_stall_trigger = float(self.get_parameter("heading_stall_trigger").value)
        self._heading_stall_odom_speed_eps = float(
            self.get_parameter("heading_stall_odom_speed_eps").value
        )
        self._heading_stall_streak_threshold = int(
            self.get_parameter("heading_stall_streak_threshold").value
        )
        self._collision_threshold = int(self.get_parameter("collision_warning_threshold").value)
        self._collision_window_dt = float(self.get_parameter("collision_window_sec").value)
        self._collision_immediate_trigger = bool(
            self.get_parameter("collision_immediate_trigger").value
        )
        self._collision_immediate_window_sec = max(
            0.05, float(self.get_parameter("collision_immediate_window_sec").value)
        )
        self._collision_substr = str(self.get_parameter("collision_warning_substring").value)
        self._planner_abort_threshold = int(self.get_parameter("planner_abort_threshold").value)
        self._planner_abort_window_dt = float(self.get_parameter("planner_abort_window_sec").value)
        self._planner_abort_substr = str(self.get_parameter("planner_abort_substring").value)
        self._planner_logger = str(self.get_parameter("planner_logger_name").value)
        self._controller_logger = str(self.get_parameter("controller_logger_name").value)
        self._clear_local_costmap_service = str(self.get_parameter("clear_local_costmap_service").value)
        tick_hz = max(5.0, float(self.get_parameter("tick_hz").value))
        self._log_debug = bool(self.get_parameter("log_debug").value)
        self._require_goal_before_trigger = bool(
            self.get_parameter("require_goal_before_trigger").value
        )
        self._startup_grace_sec = max(0.0, float(self.get_parameter("startup_grace_sec").value))
        self._reverse_min_duration_sec = max(
            0.0, float(self.get_parameter("reverse_min_duration_sec").value)
        )
        self._reverse_hard_max_duration_sec = max(
            self._reverse_min_duration_sec,
            float(self.get_parameter("reverse_hard_max_duration_sec").value),
        )
        self._reverse_clear_dist_to_path_m = max(
            0.05, float(self.get_parameter("reverse_clear_dist_to_path_m").value)
        )
        self._reverse_clear_heading_error_rad = max(
            0.05, float(self.get_parameter("reverse_clear_heading_error_rad").value)
        )
        self._reverse_collision_quiet_sec = max(
            0.05, float(self.get_parameter("reverse_collision_quiet_sec").value)
        )

        self._path: Optional[Path] = None
        self._odom: Optional[Odometry] = None
        self._cmd_nav: Optional[Twist] = None
        self._cmd_safe: Optional[Twist] = None
        self._active = False
        self._mode = "idle"
        self._active_until = 0.0
        self._stop_until = 0.0
        self._cooldown_until = 0.0
        self._pre_replan_until = 0.0
        self._pre_replan_cooldown_until = 0.0
        self._pre_replan_reason: Optional[str] = None
        self._pre_replan_path_walltime_at_request = 0.0
        self._last_heading_err = 0.0
        self._collision_hits = 0
        self._collision_window_start = 0.0
        self._planner_abort_hits = 0
        self._planner_abort_window_start = 0.0
        self._zero_speed_streak = 0
        self._heading_stall_streak = 0
        self._last_goal: Optional[PoseStamped] = None
        self._last_goal_walltime: float = 0.0
        self._last_path_walltime: float = 0.0
        self._node_start_walltime: float = time.monotonic()
        self._last_collision_walltime: float = 0.0
        self._reverse_started_walltime: float = 0.0

        self.create_subscription(Path, self._path_topic, self._on_path, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.create_subscription(Twist, self._cmd_nav_topic, self._on_cmd_nav, 20)
        self.create_subscription(Twist, self._cmd_safe_topic, self._on_cmd_safe, 20)
        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal, 20)
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)
        self._active_pub = self.create_publisher(Bool, self._active_topic, 10)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, self._clear_local_costmap_service
        )
        self.create_timer(1.0 / tick_hz, self._tick)

        self.get_logger().info(
            f"ReverseRecoveryNode active. odom='{self._odom_topic}' path='{self._path_topic}' "
            f"cmd_nav='{self._cmd_nav_topic}' cmd_safe='{self._cmd_safe_topic}' "
            f"goal='{self._goal_topic}' -> active='{self._active_topic}' cmd='{self._cmd_topic}'"
        )

    def _on_rosout(self, msg: Log) -> None:
        # Trigger recovery when controller reports repeated collision-ahead warnings
        # or planner reports repeated no-path aborts.
        if int(msg.level) < 30:
            return
        now = time.monotonic()
        if self._controller_logger in msg.name and self._collision_substr in msg.msg:
            self._last_collision_walltime = now
            if self._collision_window_start == 0.0 or (now - self._collision_window_start) > self._collision_window_dt:
                self._collision_window_start = now
                self._collision_hits = 1
            else:
                self._collision_hits += 1
            return
        if self._planner_logger not in msg.name:
            return
        if self._planner_abort_substr not in msg.msg:
            return
        if (
            self._planner_abort_window_start == 0.0
            or (now - self._planner_abort_window_start) > self._planner_abort_window_dt
        ):
            self._planner_abort_window_start = now
            self._planner_abort_hits = 1
        else:
            self._planner_abort_hits += 1

    def _on_path(self, msg: Path) -> None:
        self._path = msg
        self._last_path_walltime = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        self._last_goal = msg
        self._last_goal_walltime = time.monotonic()

    def _on_cmd_nav(self, msg: Twist) -> None:
        self._cmd_nav = msg

    def _on_cmd_safe(self, msg: Twist) -> None:
        self._cmd_safe = msg

    def _publish_active(self, active: bool) -> None:
        self._active_pub.publish(Bool(data=active))

    def _nearest_path_metrics(self) -> Optional[tuple[float, float]]:
        if self._odom is None or self._path is None or len(self._path.poses) < 2:
            return None
        x = self._odom.pose.pose.position.x
        y = self._odom.pose.pose.position.y
        yaw = _yaw_from_quat(self._odom.pose.pose.orientation)

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
        return (math.sqrt(nearest_d2), heading_err)

    def _start_maneuver(self, now: float, reason: str) -> None:
        heading_mag = abs(self._last_heading_err)
        reverse_dt = min(
            self._max_maneuver_dt,
            self._maneuver_dt + self._maneuver_heading_gain * max(0.0, heading_mag - self._heading_trigger),
        )
        self._active = True
        self._mode = "stop"
        self._stop_until = now + self._stop_dt
        # _active_until marks the end of the reverse phase (after the stop phase).
        self._active_until = self._stop_until + reverse_dt
        self._reverse_started_walltime = 0.0
        self._cooldown_until = self._active_until + self._cooldown_dt
        self._collision_hits = 0
        self._collision_window_start = 0.0
        self._planner_abort_hits = 0
        self._planner_abort_window_start = 0.0
        self._zero_speed_streak = 0
        self._heading_stall_streak = 0
        self.get_logger().warn(
            f"reverse_recovery_triggered reason={reason} "
            f"dist_to_path={self._current_dist:.2f} heading_err={self._last_heading_err:.2f} "
            f"stop={self._stop_dt:.1f}s reverse={reverse_dt:.1f}s cooldown={self._cooldown_dt:.1f}s"
        )

    def _start_pre_replan(self, now: float, reason: str) -> bool:
        if not self._pre_replan_enabled:
            return False
        if not self._allow_goal_republish_pre_replan:
            return False
        # Never preempt active Nav2 goals for rotate-stall recovery. In practice
        # this causes goal preemption loops at the worst moment.
        if reason == "rotate_stall_far_from_path":
            return False
        if now < self._pre_replan_cooldown_until:
            return False

        goal = PoseStamped()
        if self._last_goal is not None:
            goal.header.frame_id = self._last_goal.header.frame_id
            goal.pose = self._last_goal.pose
        elif self._path is not None and len(self._path.poses) >= self._pre_replan_min_path_poses:
            # Fallback: use current planned path endpoint when goal topic was not seen.
            end_pose = self._path.poses[-1]
            goal.header.frame_id = (
                self._path.header.frame_id if self._path.header.frame_id else "map"
            )
            goal.pose = end_pose.pose
        else:
            return False
        goal.header.stamp = self.get_clock().now().to_msg()
        self._goal_pub.publish(goal)
        self._mode = "pre_replan"
        wait_sec = (
            self._pre_replan_wait_zero_speed_sec
            if reason == "zero_speed_stuck_threshold"
            else self._pre_replan_wait_sec
        )
        self._pre_replan_until = now + wait_sec
        self._pre_replan_reason = reason
        self._pre_replan_path_walltime_at_request = self._last_path_walltime
        self.get_logger().warn(
            f"reverse_recovery_pre_replan reason={reason} wait={wait_sec:.1f}s"
        )
        return True

    def _request_local_costmap_clear(self) -> None:
        if not self._clear_local_costmap_client.service_is_ready():
            if self._log_debug:
                self.get_logger().info(
                    "reverse_recovery: local costmap clear service not ready"
                )
            return
        req = ClearEntireCostmap.Request()
        self._clear_local_costmap_client.call_async(req)
        self.get_logger().info("reverse_recovery: requested local costmap clear")

    def _tick(self) -> None:
        now = time.monotonic()
        if (now - self._node_start_walltime) < self._startup_grace_sec:
            self._publish_active(False)
            return

        if self._require_goal_before_trigger and self._last_goal_walltime <= 0.0:
            self._publish_active(False)
            return

        metrics = self._nearest_path_metrics()
        if metrics is None or self._cmd_nav is None:
            self._publish_active(False)
            return
        self._current_dist, self._last_heading_err = metrics

        speed_src = self._cmd_safe if self._cmd_safe is not None else self._cmd_nav
        if speed_src is not None and abs(speed_src.linear.x) < self._lin_eps and abs(speed_src.angular.z) < self._ang_eps:
            self._zero_speed_streak += 1
        else:
            self._zero_speed_streak = 0

        if self._active:
            if self._mode == "stop" and now < self._stop_until:
                self._publish_active(True)
                self._cmd_pub.publish(Twist())
                return
            if self._mode == "stop":
                self._mode = "reverse"
                self._reverse_started_walltime = now

            reverse_elapsed = (
                now - self._reverse_started_walltime
                if self._reverse_started_walltime > 0.0
                else 0.0
            )
            clear_by_path = (
                self._current_dist <= self._reverse_clear_dist_to_path_m
                and abs(self._last_heading_err) <= self._reverse_clear_heading_error_rad
            )
            collision_quiet = (now - self._last_collision_walltime) >= self._reverse_collision_quiet_sec
            clear_to_exit = (
                reverse_elapsed >= self._reverse_min_duration_sec
                and clear_by_path
                and collision_quiet
            )
            if clear_to_exit or (reverse_elapsed >= self._reverse_hard_max_duration_sec):
                if reverse_elapsed >= self._reverse_hard_max_duration_sec:
                    self.get_logger().warn(
                        "reverse_recovery_hard_stop reached "
                        f"(elapsed={reverse_elapsed:.1f}s, dist={self._current_dist:.2f}, "
                        f"heading={abs(self._last_heading_err):.2f})"
                    )
                self._active = False
                self._mode = "idle"
                self._publish_active(False)
                self.get_logger().info(
                    "reverse_recovery_complete "
                    f"(elapsed={reverse_elapsed:.2f}s, dist={self._current_dist:.2f}, "
                    f"heading={abs(self._last_heading_err):.2f}, "
                    f"collision_quiet={collision_quiet})"
                )
                self._request_local_costmap_clear()
                return

            cmd = Twist()
            # Reverse arc: for positive heading error, steer right (negative yaw rate)
            # so the vehicle front swings left while backing.
            cmd.linear.x = -self._rev_v
            use_far_reverse = self._current_dist >= self._reverse_far_dist_threshold
            reverse_w = self._rev_w_far if use_far_reverse else self._rev_w
            cmd.angular.z = -reverse_w if self._last_heading_err > 0.0 else reverse_w
            self._publish_active(True)
            self._cmd_pub.publish(cmd)
            return

        self._publish_active(False)
        if self._mode == "pre_replan":
            path_age = now - self._last_path_walltime if self._last_path_walltime > 0.0 else float("inf")
            has_fresh_path = (
                self._path is not None
                and len(self._path.poses) >= self._pre_replan_min_path_poses
                and (
                    self._last_path_walltime > self._pre_replan_path_walltime_at_request
                    or path_age <= self._pre_replan_fresh_path_age_sec
                )
            )
            if has_fresh_path:
                self._mode = "idle"
                self._pre_replan_cooldown_until = now + self._pre_replan_cooldown_sec
                self.get_logger().info(
                    "reverse_recovery_pre_replan_succeeded fresh_path_received"
                )
                return
            if now < self._pre_replan_until:
                return
            reason = self._pre_replan_reason or "unknown"
            self._mode = "idle"
            self._start_maneuver(now, reason=f"{reason}_after_pre_replan_timeout")
            return

        if now < self._cooldown_until:
            return

        nav_v = float(self._cmd_nav.linear.x)
        nav_w = float(self._cmd_nav.angular.z)
        is_rotate_like = abs(nav_v) < self._lin_eps and abs(nav_w) > self._ang_eps
        far_from_path = self._current_dist > self._dist_trigger
        big_heading_err = abs(self._last_heading_err) > self._heading_trigger

        odom_vx = float(self._odom.twist.twist.linear.x) if self._odom is not None else 0.0
        odom_vy = float(self._odom.twist.twist.linear.y) if self._odom is not None else 0.0
        odom_speed = math.hypot(odom_vx, odom_vy)
        very_big_heading_err = abs(self._last_heading_err) > self._heading_stall_trigger
        low_odom_speed = odom_speed < self._heading_stall_odom_speed_eps
        if very_big_heading_err and low_odom_speed:
            self._heading_stall_streak += 1
        else:
            self._heading_stall_streak = 0

        if self._collision_hits >= self._collision_threshold:
            if not self._start_pre_replan(now, reason="collision_warning_threshold"):
                self._start_maneuver(now, reason="collision_warning_threshold")
            return
        if (
            self._collision_immediate_trigger
            and (now - self._last_collision_walltime) <= self._collision_immediate_window_sec
        ):
            if not self._start_pre_replan(now, reason="collision_warning_immediate"):
                self._start_maneuver(now, reason="collision_warning_immediate")
            return
        if self._planner_abort_hits >= self._planner_abort_threshold:
            if not self._start_pre_replan(now, reason="planner_abort_threshold"):
                self._start_maneuver(now, reason="planner_abort_threshold")
            return
        within_goal_grace = (
            self._last_goal_walltime > 0.0
            and (now - self._last_goal_walltime) < self._goal_start_grace_sec
        )
        # Global startup grace: suppress all autonomous recovery triggers for a
        # short period after receiving goal, avoiding immediate reverse/backup
        # while planner/controller are still converging.
        if within_goal_grace:
            return

        if (
            self._zero_speed_streak >= self._stuck_zero_speed_streak_threshold
            and self._current_dist > self._stuck_dist_trigger
            and abs(self._last_heading_err) > self._stuck_heading_trigger
        ):
            if not self._start_pre_replan(now, reason="zero_speed_stuck_threshold"):
                self._start_maneuver(now, reason="zero_speed_stuck_threshold")
            return
        if self._heading_stall_streak >= self._heading_stall_streak_threshold:
            if not self._start_pre_replan(now, reason="heading_stall_low_odom_speed"):
                self._start_maneuver(now, reason="heading_stall_low_odom_speed")
            return

        if is_rotate_like and far_from_path and big_heading_err:
            if not self._start_pre_replan(now, reason="rotate_stall_far_from_path"):
                self._start_maneuver(now, reason="rotate_stall_far_from_path")
        elif self._log_debug and is_rotate_like and (far_from_path or big_heading_err):
            self.get_logger().info(
                "reverse_recovery_not_triggered "
                f"dist={self._current_dist:.2f}/{self._dist_trigger:.2f} "
                f"heading={abs(self._last_heading_err):.2f}/{self._heading_trigger:.2f} "
                f"odom_speed={odom_speed:.3f}/{self._heading_stall_odom_speed_eps:.3f} "
                f"h_streak={self._heading_stall_streak}/{self._heading_stall_streak_threshold}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReverseRecoveryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

