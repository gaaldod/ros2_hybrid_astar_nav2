from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class AnchorFrameGuardNode(Node):
    """Anchor AMCL to odometry-based reference transform in simulation."""

    def __init__(self) -> None:
        super().__init__("anchor_frame_guard_node")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("drift_warn_m", 0.8)
        self.declare_parameter("drift_warn_yaw_deg", 18.0)
        self.declare_parameter("reseed_enabled", True)
        self.declare_parameter("reseed_trigger_m", 1.8)
        self.declare_parameter("reseed_trigger_yaw_deg", 30.0)
        self.declare_parameter("reseed_sustained_trigger_m", 1.2)
        self.declare_parameter("reseed_sustained_trigger_yaw_deg", 14.0)
        self.declare_parameter("reseed_sustained_sec", 4.0)
        self.declare_parameter("reseed_cooldown_sec", 20.0)
        self.declare_parameter("reseed_max_speed_mps", 0.08)
        self.declare_parameter("max_reseed_xy_step_m", 1.5)
        self.declare_parameter("max_reseed_yaw_step_deg", 25.0)
        self.declare_parameter("persistent_drift_stop_enabled", True)
        self.declare_parameter("persistent_drift_stop_yaw_deg", 5.0)
        self.declare_parameter("persistent_drift_stop_dist_m", 0.0)
        self.declare_parameter("persistent_drift_stop_sec", 5.0)
        self.declare_parameter("persistent_drift_stop_topic", "nav_anchor_drift_stop")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("goal_republish_after_reseed", True)
        self.declare_parameter("goal_republish_delay_sec", 0.4)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self._initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self._drift_warn_m = max(0.05, float(self.get_parameter("drift_warn_m").value))
        self._drift_warn_yaw_rad = math.radians(
            max(1.0, float(self.get_parameter("drift_warn_yaw_deg").value))
        )
        self._reseed_enabled = bool(self.get_parameter("reseed_enabled").value)
        self._reseed_trigger_m = max(0.1, float(self.get_parameter("reseed_trigger_m").value))
        self._reseed_trigger_yaw_rad = math.radians(
            max(1.0, float(self.get_parameter("reseed_trigger_yaw_deg").value))
        )
        self._reseed_sustained_trigger_m = max(
            0.05, float(self.get_parameter("reseed_sustained_trigger_m").value)
        )
        self._reseed_sustained_trigger_yaw_rad = math.radians(
            max(1.0, float(self.get_parameter("reseed_sustained_trigger_yaw_deg").value))
        )
        self._reseed_sustained_sec = max(
            0.2, float(self.get_parameter("reseed_sustained_sec").value)
        )
        self._reseed_cooldown_sec = max(1.0, float(self.get_parameter("reseed_cooldown_sec").value))
        self._reseed_max_speed_mps = max(
            0.0, float(self.get_parameter("reseed_max_speed_mps").value)
        )
        self._max_reseed_xy_step_m = max(0.05, float(self.get_parameter("max_reseed_xy_step_m").value))
        self._max_reseed_yaw_step_rad = math.radians(
            max(1.0, float(self.get_parameter("max_reseed_yaw_step_deg").value))
        )
        self._persistent_drift_stop_enabled = bool(
            self.get_parameter("persistent_drift_stop_enabled").value
        )
        self._persistent_drift_stop_yaw_rad = math.radians(
            max(0.1, float(self.get_parameter("persistent_drift_stop_yaw_deg").value))
        )
        self._persistent_drift_stop_dist_m = max(
            0.0, float(self.get_parameter("persistent_drift_stop_dist_m").value)
        )
        self._persistent_drift_stop_sec = max(
            0.2, float(self.get_parameter("persistent_drift_stop_sec").value)
        )
        self._persistent_drift_stop_topic = str(
            self.get_parameter("persistent_drift_stop_topic").value
        )
        self._goal_topic = str(self.get_parameter("goal_topic").value)
        self._goal_republish_after_reseed = bool(
            self.get_parameter("goal_republish_after_reseed").value
        )
        self._goal_republish_delay_sec = max(
            0.0, float(self.get_parameter("goal_republish_delay_sec").value)
        )

        self._last_odom: Optional[Tuple[float, float, float]] = None
        self._last_amcl: Optional[Tuple[float, float, float]] = None
        # Fixed anchor transform map<-odom captured at first valid pair.
        self._anchor_tx: Optional[float] = None
        self._anchor_ty: Optional[float] = None
        self._anchor_yaw: Optional[float] = None
        self._last_warn_walltime: float = 0.0
        self._last_reseed_walltime: float = 0.0
        self._sustained_drift_start: float = 0.0
        self._last_speed_mps: float = 0.0
        self._persistent_drift_start: float = 0.0
        self._persistent_stop_active: bool = False
        self._was_persistent_stop_active: bool = False
        self._cooldown_warned: bool = False
        self._speed_gate_warned: bool = False
        self._last_goal: Optional[PoseStamped] = None
        self._goal_republish_due_walltime: float = 0.0

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, self._amcl_pose_topic, self._on_amcl, 20)
        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal, 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, self._initialpose_topic, 10)
        self._drift_stop_pub = self.create_publisher(Bool, self._persistent_drift_stop_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self.create_timer(0.1, self._heartbeat)

        self.get_logger().warn(
            f"anchor_frame_guard active odom='{self._odom_topic}' amcl='{self._amcl_pose_topic}'"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            _yaw_from_quat(msg.pose.pose.orientation),
        )
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self._last_speed_mps = math.hypot(vx, vy)
        self._tick()

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_amcl = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            _yaw_from_quat(msg.pose.pose.orientation),
        )
        self._tick()

    def _on_goal(self, msg: PoseStamped) -> None:
        self._last_goal = msg

    def _heartbeat(self) -> None:
        stop_msg = Bool()
        stop_msg.data = self._persistent_stop_active
        self._drift_stop_pub.publish(stop_msg)
        if (
            self._goal_republish_due_walltime > 0.0
            and time.perf_counter() >= self._goal_republish_due_walltime
        ):
            self._goal_republish_due_walltime = 0.0
            if self._last_goal is None:
                self.get_logger().warn(
                    "anchor_frame_guard goal republish failed: no active goal cached"
                )
                return
            self._goal_pub.publish(self._last_goal)
            self.get_logger().warn(
                "anchor_frame_guard goal republished after reseed "
                f"(topic='{self._goal_topic}', x={self._last_goal.pose.position.x:.2f}, "
                f"y={self._last_goal.pose.position.y:.2f})"
            )

    def _tick(self) -> None:
        if self._last_odom is None or self._last_amcl is None:
            return
        ox, oy, oyaw = self._last_odom
        mx, my, myaw = self._last_amcl

        if self._anchor_tx is None:
            # T_map_odom := T_map_base * inv(T_odom_base) using first pair.
            self._anchor_yaw = _norm_angle(myaw - oyaw)
            c = math.cos(self._anchor_yaw)
            s = math.sin(self._anchor_yaw)
            self._anchor_tx = mx - (c * ox - s * oy)
            self._anchor_ty = my - (s * ox + c * oy)
            self.get_logger().warn(
                "anchor_frame_guard initialized "
                f"(tx={self._anchor_tx:.2f}, ty={self._anchor_ty:.2f}, yaw={self._anchor_yaw:.2f})"
            )
            return

        assert self._anchor_tx is not None
        assert self._anchor_ty is not None
        assert self._anchor_yaw is not None
        c = math.cos(self._anchor_yaw)
        s = math.sin(self._anchor_yaw)
        ex = self._anchor_tx + (c * ox - s * oy)
        ey = self._anchor_ty + (s * ox + c * oy)
        eyaw = _norm_angle(oyaw + self._anchor_yaw)

        dx = ex - mx
        dy = ey - my
        dxy = math.hypot(dx, dy)
        dyaw = abs(_norm_angle(eyaw - myaw))

        now = time.perf_counter()
        if dxy >= self._drift_warn_m or dyaw >= self._drift_warn_yaw_rad:
            if (now - self._last_warn_walltime) > 5.0:
                self.get_logger().warn(
                    "anchor_frame_guard drift "
                    f"(dxy={dxy:.3f}m, dyaw_deg={math.degrees(dyaw):.1f}, "
                    f"exp=({ex:.2f},{ey:.2f},{eyaw:.2f}) cur=({mx:.2f},{my:.2f},{myaw:.2f}))"
                )
                self._last_warn_walltime = now

        dist_trigger = (
            self._persistent_drift_stop_dist_m > 0.0 and dxy >= self._persistent_drift_stop_dist_m
        )
        persistent_now = (dyaw >= self._persistent_drift_stop_yaw_rad) or dist_trigger
        if self._persistent_drift_stop_enabled and persistent_now:
            if self._persistent_drift_start == 0.0:
                self._persistent_drift_start = now
            self._persistent_stop_active = (now - self._persistent_drift_start) >= self._persistent_drift_stop_sec
        else:
            self._persistent_drift_start = 0.0
            self._persistent_stop_active = False
        if self._persistent_stop_active and not self._was_persistent_stop_active:
            self.get_logger().warn(
                "anchor_frame_guard drift stop ENGAGED "
                f"(dxy={dxy:.3f}m, dyaw_deg={math.degrees(dyaw):.1f}, "
                f"sustained_for={(now - self._persistent_drift_start):.1f}s)"
            )
            self._was_persistent_stop_active = True
        elif not self._persistent_stop_active and self._was_persistent_stop_active:
            self.get_logger().warn(
                "anchor_frame_guard drift stop CLEARED "
                f"(dxy={dxy:.3f}m, dyaw_deg={math.degrees(dyaw):.1f})"
            )
            self._was_persistent_stop_active = False

        if not self._reseed_enabled:
            return
        hard_trigger = (dxy >= self._reseed_trigger_m) or (dyaw >= self._reseed_trigger_yaw_rad)
        sustained_now = (
            dxy >= self._reseed_sustained_trigger_m or dyaw >= self._reseed_sustained_trigger_yaw_rad
        )
        if sustained_now:
            if self._sustained_drift_start == 0.0:
                self._sustained_drift_start = now
        else:
            self._sustained_drift_start = 0.0
        sustained_trigger = (
            self._sustained_drift_start > 0.0
            and (now - self._sustained_drift_start) >= self._reseed_sustained_sec
        )
        persistent_trigger = self._persistent_stop_active
        if not (hard_trigger or sustained_trigger or persistent_trigger):
            return
        if (now - self._last_reseed_walltime) < self._reseed_cooldown_sec:
            if not self._cooldown_warned:
                self.get_logger().warn(
                    "anchor_frame_guard reseed gated by cooldown "
                    f"(remaining={self._reseed_cooldown_sec - (now - self._last_reseed_walltime):.1f}s, "
                    f"dxy={dxy:.3f}m, dyaw_deg={math.degrees(dyaw):.1f})"
                )
                self._cooldown_warned = True
            return
        self._cooldown_warned = False
        if self._last_speed_mps > self._reseed_max_speed_mps:
            if not self._speed_gate_warned:
                self.get_logger().warn(
                    "anchor_frame_guard reseed gated by motion "
                    f"(speed={self._last_speed_mps:.3f}mps, "
                    f"limit={self._reseed_max_speed_mps:.3f}mps, "
                    f"dxy={dxy:.3f}m, dyaw_deg={math.degrees(dyaw):.1f})"
                )
                self._speed_gate_warned = True
            return
        self._speed_gate_warned = False

        # Bound correction step to avoid violent map frame snaps.
        if dxy > self._max_reseed_xy_step_m:
            scale = self._max_reseed_xy_step_m / max(1e-6, dxy)
            tx = mx + dx * scale
            ty = my + dy * scale
        else:
            tx, ty = ex, ey
        yaw_err = _norm_angle(eyaw - myaw)
        if abs(yaw_err) > self._max_reseed_yaw_step_rad:
            tyaw = _norm_angle(myaw + math.copysign(self._max_reseed_yaw_step_rad, yaw_err))
        else:
            tyaw = eyaw

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # Stamp zero => AMCL uses latest available TF.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.pose.position.x = tx
        msg.pose.pose.position.y = ty
        msg.pose.pose.position.z = 0.0
        half = 0.5 * tyaw
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        msg.pose.covariance[0] = 0.20
        msg.pose.covariance[7] = 0.20
        msg.pose.covariance[35] = 0.35
        self._initialpose_pub.publish(msg)
        self._last_reseed_walltime = now
        self._sustained_drift_start = 0.0
        self._persistent_drift_start = 0.0
        self._persistent_stop_active = False
        self.get_logger().warn(
            "anchor_frame_guard reseed "
            f"(target=({tx:.2f},{ty:.2f},{tyaw:.2f}), "
            f"dxy={dxy:.2f}m, dyaw_deg={math.degrees(dyaw):.1f}, "
            f"hard_trigger={hard_trigger}, sustained_trigger={sustained_trigger}, "
            f"persistent_trigger={persistent_trigger}, "
            f"speed={self._last_speed_mps:.3f}mps)"
        )
        if self._goal_republish_after_reseed:
            self._goal_republish_due_walltime = now + self._goal_republish_delay_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnchorFrameGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

