from __future__ import annotations

import math
import os
import time
from datetime import datetime
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from rosgraph_msgs.msg import Clock


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class TfEventLoggerNode(Node):
    """Detect TF/clock discontinuities that can rotate visual frames."""

    def __init__(self) -> None:
        super().__init__("tf_event_logger_node")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("yaw_jump_warn_deg", 8.0)
        self.declare_parameter("xy_jump_warn_m", 0.35)
        self.declare_parameter("clock_back_jump_warn_sec", 0.3)
        self.declare_parameter("clock_gap_warn_sec", 1.5)
        self.declare_parameter("tf_stale_warn_sec", 0.35)
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("controller_logger_name", "controller_server")
        self.declare_parameter("controller_plan_zero_substring", "Resulting plan has 0 poses in it.")
        self.declare_parameter("amcl_jump_warn_m", 0.5)
        self.declare_parameter("amcl_yaw_jump_warn_deg", 12.0)
        self.declare_parameter("context_window_sec", 2.0)
        self.declare_parameter("log_dir", "/home/dominik/ros2_ws/logs/nav2_runs")

        self._global = str(self.get_parameter("global_frame").value)
        self._odom = str(self.get_parameter("odom_frame").value)
        self._base = str(self.get_parameter("base_frame").value)
        self._tick_hz = max(1.0, float(self.get_parameter("tick_hz").value))
        self._yaw_jump_warn_rad = math.radians(
            max(1.0, float(self.get_parameter("yaw_jump_warn_deg").value))
        )
        self._xy_jump_warn_m = max(0.05, float(self.get_parameter("xy_jump_warn_m").value))
        self._clock_back_jump_warn_sec = max(
            0.01, float(self.get_parameter("clock_back_jump_warn_sec").value)
        )
        self._clock_gap_warn_sec = max(0.1, float(self.get_parameter("clock_gap_warn_sec").value))
        self._tf_stale_warn_sec = max(0.05, float(self.get_parameter("tf_stale_warn_sec").value))
        self._amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._controller_logger_name = str(self.get_parameter("controller_logger_name").value)
        self._controller_plan_zero_substring = str(
            self.get_parameter("controller_plan_zero_substring").value
        )
        self._amcl_jump_warn_m = max(0.05, float(self.get_parameter("amcl_jump_warn_m").value))
        self._amcl_yaw_jump_warn_rad = math.radians(
            max(1.0, float(self.get_parameter("amcl_yaw_jump_warn_deg").value))
        )
        self._context_window_sec = max(0.2, float(self.get_parameter("context_window_sec").value))
        self._log_dir = str(self.get_parameter("log_dir").value)

        os.makedirs(self._log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(self._log_dir, f"tf_event_{ts}.log")

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Clock, "/clock", self._on_clock, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, self._amcl_pose_topic, self._on_amcl_pose, 20
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 20)
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)
        self.create_timer(1.0 / self._tick_hz, self._tick)

        self._last_clock: Optional[float] = None
        self._last_map_odom: Optional[Tuple[float, float, float, float]] = None
        self._last_odom_base: Optional[Tuple[float, float, float, float]] = None
        self._last_amcl: Optional[Tuple[float, float, float, float]] = None
        self._last_odom_speed: float = 0.0
        self._last_cmd_linear: float = 0.0
        self._last_cmd_angular: float = 0.0
        self._last_controller_plan_zero_walltime: float = 0.0

        self._write(
            f"tf_event_logger started global={self._global} odom={self._odom} base={self._base} "
            f"tick_hz={self._tick_hz:.1f}"
        )
        self.get_logger().warn(f"tf_event_logger active -> {self._log_path}")

    def _write(self, msg: str) -> None:
        stamp = time.time()
        line = f"[{stamp:.3f}] {msg}\n"
        with open(self._log_path, "a", encoding="utf-8") as f:
            f.write(line)

    def _now_sec(self) -> float:
        now_msg = self.get_clock().now().to_msg()
        return now_msg.sec + now_msg.nanosec * 1e-9

    @staticmethod
    def _msg_time_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_clock(self, msg: Clock) -> None:
        t = self._msg_time_sec(msg.clock)
        if self._last_clock is not None:
            dt = t - self._last_clock
            if dt < -self._clock_back_jump_warn_sec:
                txt = f"CLOCK_BACK_JUMP dt={dt:.3f}s new={t:.3f} old={self._last_clock:.3f}"
                self.get_logger().warn(txt)
                self._write(txt)
            elif dt > self._clock_gap_warn_sec:
                txt = f"CLOCK_GAP dt={dt:.3f}s new={t:.3f} old={self._last_clock:.3f}"
                self.get_logger().warn(txt)
                self._write(txt)
        self._last_clock = t

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        t = self._msg_time_sec(msg.header.stamp)
        cur = (x, y, yaw, t)
        if self._last_amcl is not None:
            dx = cur[0] - self._last_amcl[0]
            dy = cur[1] - self._last_amcl[1]
            dxy = math.hypot(dx, dy)
            dyaw = abs(_norm_angle(cur[2] - self._last_amcl[2]))
            if dxy > self._amcl_jump_warn_m or dyaw > self._amcl_yaw_jump_warn_rad:
                txt = (
                    f"AMCL_JUMP dxy={dxy:.3f}m dyaw_deg={math.degrees(dyaw):.1f} "
                    f"frame={msg.header.frame_id} stamp={t:.3f}{self._context_suffix()}"
                )
                self.get_logger().warn(txt)
                self._write(txt)
        self._last_amcl = cur

    def _on_odom(self, msg: Odometry) -> None:
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self._last_odom_speed = math.hypot(vx, vy)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_linear = float(msg.linear.x)
        self._last_cmd_angular = float(msg.angular.z)

    def _on_rosout(self, msg: Log) -> None:
        if int(msg.level) < 30:
            return
        if self._controller_logger_name not in msg.name:
            return
        if self._controller_plan_zero_substring not in msg.msg:
            return
        self._last_controller_plan_zero_walltime = time.perf_counter()

    def _context_suffix(self) -> str:
        now = time.perf_counter()
        dt_zero = (
            now - self._last_controller_plan_zero_walltime
            if self._last_controller_plan_zero_walltime > 0.0
            else float("inf")
        )
        recent_zero = dt_zero <= self._context_window_sec
        dt_zero_txt = f"{dt_zero:.2f}" if math.isfinite(dt_zero) else "inf"
        return (
            f" odom_speed={self._last_odom_speed:.3f}mps"
            f" cmd=({self._last_cmd_linear:.3f},{self._last_cmd_angular:.3f})"
            f" recent_plan_zero={recent_zero}"
            f" dt_plan_zero={dt_zero_txt}s"
        )

    def _lookup_xy_yaw(self, parent: str, child: str) -> Optional[Tuple[float, float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = _yaw_from_quat(tf.transform.rotation)
        tf_stamp = self._msg_time_sec(tf.header.stamp)
        return (x, y, yaw, tf_stamp)

    def _check_jump(
        self,
        name: str,
        prev: Optional[Tuple[float, float, float, float]],
        cur: Optional[Tuple[float, float, float, float]],
    ) -> Optional[Tuple[float, float, float, float]]:
        if cur is None:
            return prev
        if prev is None:
            return cur
        now_sec = self._now_sec()
        tf_age = now_sec - cur[3]
        if tf_age > self._tf_stale_warn_sec:
            txt = f"TF_STALE {name} age={tf_age:.3f}s tf_stamp={cur[3]:.3f} now={now_sec:.3f}"
            self.get_logger().warn(txt)
            self._write(txt)
        if cur[3] + 1e-6 < prev[3]:
            txt = f"TF_STAMP_BACKWARD {name} prev={prev[3]:.3f} cur={cur[3]:.3f}"
            self.get_logger().warn(txt)
            self._write(txt)
        dx = cur[0] - prev[0]
        dy = cur[1] - prev[1]
        dxy = math.hypot(dx, dy)
        dyaw = abs(_norm_angle(cur[2] - prev[2]))
        if dxy > self._xy_jump_warn_m or dyaw > self._yaw_jump_warn_rad:
            txt = (
                f"TF_JUMP {name} dxy={dxy:.3f}m dyaw_deg={math.degrees(dyaw):.1f} "
                f"from=({prev[0]:.2f},{prev[1]:.2f},{prev[2]:.2f}) "
                f"to=({cur[0]:.2f},{cur[1]:.2f},{cur[2]:.2f}) "
                f"tf_stamp={cur[3]:.3f} tf_age={tf_age:.3f}s{self._context_suffix()}"
            )
            self.get_logger().warn(txt)
            self._write(txt)
        return cur

    def _tick(self) -> None:
        map_odom = self._lookup_xy_yaw(self._global, self._odom)
        odom_base = self._lookup_xy_yaw(self._odom, self._base)
        self._last_map_odom = self._check_jump("map->odom", self._last_map_odom, map_odom)
        self._last_odom_base = self._check_jump("odom->base", self._last_odom_base, odom_base)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfEventLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

