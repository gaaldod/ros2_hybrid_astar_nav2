"""
Ackermann safety controller node (pipeline node 4 of 5).

Subscribes to the velocity command that Nav2's controller / velocity smoother
publishes (typically ``geometry_msgs/Twist`` on ``cmd_vel`` or
``cmd_vel_smoothed``), applies:

* optional **velocity limits** consistent with a small Ackermann platform;
* scaling from :class:`LocalPlannerNode` (``nav_speed_scale``);
* **emergency stop** from :class:`LocalPlannerNode` when a dynamic obstacle
  intrudes into the path corridor.

Publishes the final ``Twist`` to the driver / Gazebo bridge topic used in your
simulation (``cmd_vel`` in ``megoldas_sim24``, or ``roboworks/cmd_vel`` if you
remap the bridge that way).

This mirrors the previous project's pattern (``Twist`` on ``cmd_vel``) while
keeping Nav2 as the motion planner.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class AckermannSafetyControllerNode(Node):
    """
    Final command gate: limit and scale ``Twist`` before the robot driver.

    Parameters
    ----------
    cmd_vel_in_topic : str
        Input commands from Nav2 (e.g. ``cmd_vel_smoothed``).
    cmd_vel_out_topic : str
        Output to the robot or Gazebo bridge (e.g. ``cmd_vel``).
    speed_scale_topic : str
        ``Float32`` in ``[0, 1]`` from :class:`LocalPlannerNode`.
    emergency_stop_topic : str
        ``Bool``; when true, linear and angular commands are zeroed.
    max_linear_speed : float
        Absolute cap on ``linear.x`` (m/s).
    max_angular_speed : float
        Absolute cap on ``angular.z`` (rad/s).
    min_linear_when_turning : float
        Enforced crawl speed when there is meaningful angular command but
        commanded linear speed is near zero (Ackermann anti-spin deadlock).
    turning_angular_threshold : float
        Minimum ``|angular.z|`` to treat command as a turn.
    """

    def __init__(self) -> None:
        super().__init__("ackermann_safety_controller_node")

        self.declare_parameter("cmd_vel_in_topic", "cmd_vel")
        self.declare_parameter("cmd_vel_out_topic", "cmd_vel_safe")
        self.declare_parameter("speed_scale_topic", "nav_speed_scale")
        self.declare_parameter("emergency_stop_topic", "nav_emergency_stop")
        self.declare_parameter("aux_emergency_stop_topic", "nav_localization_jump_stop")
        self.declare_parameter("aux_emergency_stop_topic_2", "")
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("min_linear_when_turning", 0.12)
        self.declare_parameter("turning_angular_threshold", 0.12)
        self.declare_parameter("enforce_crawl_on_rotate_zero_cmd", True)
        self.declare_parameter("recovery_active_topic", "nav_recovery_active")
        self.declare_parameter("recovery_cmd_topic", "cmd_vel_recovery")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("path_topic", "planned_path")
        self.declare_parameter("odom_topic", "odom_combined")
        self.declare_parameter("path_assist_enabled", True)
        self.declare_parameter("path_assist_distance_m", 1.2)
        self.declare_parameter("path_assist_heading_deadband_rad", 0.10)
        self.declare_parameter("path_assist_min_angular_rad_s", 0.10)
        self.declare_parameter("path_assist_max_angular_rad_s", 0.55)
        self.declare_parameter("path_assist_lookahead_points", 6)
        self.declare_parameter("path_rejoin_countersteer_enabled", True)
        self.declare_parameter("path_rejoin_band_m", 0.90)
        self.declare_parameter("path_rejoin_countersteer_deadband_m", 0.08)
        self.declare_parameter("path_rejoin_heading_weight", 0.70)
        self.declare_parameter("path_rejoin_cross_track_gain", 1.10)
        self.declare_parameter("path_rejoin_speed_floor_mps", 0.10)
        self.declare_parameter("front_collision_stop_enabled", True)
        self.declare_parameter("front_collision_scan_topic", "scan_filtered")
        self.declare_parameter("front_collision_stop_distance_m", 0.40)
        self.declare_parameter("front_collision_half_angle_rad", 0.35)
        self.declare_parameter("front_collision_min_points", 3)

        self._in_topic = self.get_parameter("cmd_vel_in_topic").get_parameter_value().string_value
        self._out_topic = self.get_parameter("cmd_vel_out_topic").get_parameter_value().string_value
        self._scale_topic = self.get_parameter("speed_scale_topic").get_parameter_value().string_value
        self._emerg_topic = self.get_parameter("emergency_stop_topic").get_parameter_value().string_value
        self._aux_emerg_topic = (
            self.get_parameter("aux_emergency_stop_topic").get_parameter_value().string_value
        )
        self._aux_emerg_topic_2 = (
            self.get_parameter("aux_emergency_stop_topic_2").get_parameter_value().string_value
        )
        self._max_v = abs(self.get_parameter("max_linear_speed").get_parameter_value().double_value)
        self._max_w = abs(self.get_parameter("max_angular_speed").get_parameter_value().double_value)
        self._min_v_turn = abs(
            self.get_parameter("min_linear_when_turning").get_parameter_value().double_value
        )
        self._turn_w_thresh = abs(
            self.get_parameter("turning_angular_threshold").get_parameter_value().double_value
        )
        self._enforce_crawl_on_rotate_zero_cmd = bool(
            self.get_parameter("enforce_crawl_on_rotate_zero_cmd").get_parameter_value().bool_value
        )
        self._recovery_active_topic = (
            self.get_parameter("recovery_active_topic").get_parameter_value().string_value
        )
        self._recovery_cmd_topic = self.get_parameter("recovery_cmd_topic").get_parameter_value().string_value
        self._publish_rate_hz = max(
            5.0, self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        self._odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._path_assist_enabled = (
            self.get_parameter("path_assist_enabled").get_parameter_value().bool_value
        )
        self._path_assist_dist = abs(
            self.get_parameter("path_assist_distance_m").get_parameter_value().double_value
        )
        self._path_assist_heading_deadband = abs(
            self.get_parameter("path_assist_heading_deadband_rad").get_parameter_value().double_value
        )
        self._path_assist_min_w = abs(
            self.get_parameter("path_assist_min_angular_rad_s").get_parameter_value().double_value
        )
        self._path_assist_max_w = abs(
            self.get_parameter("path_assist_max_angular_rad_s").get_parameter_value().double_value
        )
        self._path_assist_lookahead_points = max(
            1, self.get_parameter("path_assist_lookahead_points").get_parameter_value().integer_value
        )
        self._path_rejoin_countersteer_enabled = (
            self.get_parameter("path_rejoin_countersteer_enabled").get_parameter_value().bool_value
        )
        self._path_rejoin_band_m = abs(
            self.get_parameter("path_rejoin_band_m").get_parameter_value().double_value
        )
        self._path_rejoin_countersteer_deadband_m = abs(
            self.get_parameter("path_rejoin_countersteer_deadband_m").get_parameter_value().double_value
        )
        self._path_rejoin_heading_weight = max(
            0.0,
            min(
                1.0,
                self.get_parameter("path_rejoin_heading_weight").get_parameter_value().double_value,
            ),
        )
        self._path_rejoin_cross_track_gain = max(
            0.0, self.get_parameter("path_rejoin_cross_track_gain").get_parameter_value().double_value
        )
        self._path_rejoin_speed_floor_mps = max(
            0.01, self.get_parameter("path_rejoin_speed_floor_mps").get_parameter_value().double_value
        )
        self._front_collision_stop_enabled = (
            self.get_parameter("front_collision_stop_enabled").get_parameter_value().bool_value
        )
        self._front_collision_scan_topic = (
            self.get_parameter("front_collision_scan_topic").get_parameter_value().string_value
        )
        self._front_collision_stop_distance_m = abs(
            self.get_parameter("front_collision_stop_distance_m").get_parameter_value().double_value
        )
        self._front_collision_half_angle_rad = abs(
            self.get_parameter("front_collision_half_angle_rad").get_parameter_value().double_value
        )
        self._front_collision_min_points = max(
            1, self.get_parameter("front_collision_min_points").get_parameter_value().integer_value
        )

        self._scale: float = 1.0
        self._emergency: bool = False
        self._aux_emergency: bool = False
        self._aux_emergency_2: bool = False
        self._last_cmd: Optional[Twist] = None
        self._recovery_active: bool = False
        self._recovery_cmd: Optional[Twist] = None
        self._path: Optional[Path] = None
        self._odom: Optional[Odometry] = None
        self._front_collision_blocked: bool = False

        self.create_subscription(Twist, self._in_topic, self._on_twist, 10)
        self.create_subscription(Float32, self._scale_topic, self._on_scale, 10)
        self.create_subscription(Bool, self._emerg_topic, self._on_emerg, 10)
        self.create_subscription(Bool, self._aux_emerg_topic, self._on_aux_emerg, 10)
        if self._aux_emerg_topic_2:
            self.create_subscription(Bool, self._aux_emerg_topic_2, self._on_aux_emerg_2, 10)
        self.create_subscription(Bool, self._recovery_active_topic, self._on_recovery_active, 10)
        self.create_subscription(Twist, self._recovery_cmd_topic, self._on_recovery_cmd, 10)
        self.create_subscription(Path, self._path_topic, self._on_path, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        if self._front_collision_stop_enabled:
            self.create_subscription(
                LaserScan, self._front_collision_scan_topic, self._on_scan, 20
            )
        self._pub = self.create_publisher(Twist, self._out_topic, 10)
        self.create_timer(1.0 / self._publish_rate_hz, self._publish_output)

        self.get_logger().info(
            f"AckermannSafetyControllerNode: '{self._in_topic}' -> '{self._out_topic}' "
            f"(scale='{self._scale_topic}', e_stop='{self._emerg_topic}')"
        )

    def _on_scale(self, msg: Float32) -> None:
        self._scale = max(0.0, min(1.0, float(msg.data)))

    def _on_emerg(self, msg: Bool) -> None:
        self._emergency = bool(msg.data)

    def _on_aux_emerg(self, msg: Bool) -> None:
        self._aux_emergency = bool(msg.data)

    def _on_aux_emerg_2(self, msg: Bool) -> None:
        self._aux_emergency_2 = bool(msg.data)

    def _on_recovery_active(self, msg: Bool) -> None:
        self._recovery_active = bool(msg.data)

    def _on_recovery_cmd(self, msg: Twist) -> None:
        self._recovery_cmd = msg

    def _on_path(self, msg: Path) -> None:
        self._path = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._front_collision_stop_enabled:
            self._front_collision_blocked = False
            return
        close_hits = 0
        a = msg.angle_min
        for r in msg.ranges:
            if abs(a) <= self._front_collision_half_angle_rad and math.isfinite(r):
                if msg.range_min <= r <= self._front_collision_stop_distance_m:
                    close_hits += 1
                    if close_hits >= self._front_collision_min_points:
                        self._front_collision_blocked = True
                        return
            a += msg.angle_increment
        self._front_collision_blocked = False

    def _path_tracking_metrics(self) -> Optional[tuple[float, float, float]]:
        if self._path is None or self._odom is None or len(self._path.poses) < 2:
            return None
        x = self._odom.pose.pose.position.x
        y = self._odom.pose.pose.position.y
        q = self._odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        nearest_i = 0
        nearest_d2 = float("inf")
        for i, ps in enumerate(self._path.poses):
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            d2 = dx * dx + dy * dy
            if d2 < nearest_d2:
                nearest_d2 = d2
                nearest_i = i
        seg_i0 = nearest_i
        seg_i1 = min(nearest_i + 1, len(self._path.poses) - 1)
        p0 = self._path.poses[seg_i0].pose.position
        p1 = self._path.poses[seg_i1].pose.position
        seg_dx = p1.x - p0.x
        seg_dy = p1.y - p0.y
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len > 1e-6:
            # Signed cross-track error wrt path segment left-normal.
            # Positive means robot is on left side of segment direction.
            cross_track = ((x - p0.x) * (-seg_dy) + (y - p0.y) * seg_dx) / seg_len
            path_heading = math.atan2(seg_dy, seg_dx)
        else:
            cross_track = 0.0
            path_heading = yaw
        target_i = min(nearest_i + self._path_assist_lookahead_points, len(self._path.poses) - 1)
        tx = self._path.poses[target_i].pose.position.x
        ty = self._path.poses[target_i].pose.position.y
        lookahead_heading = math.atan2(ty - y, tx - x)
        heading_target = math.atan2(
            math.sin(
                self._path_rejoin_heading_weight * path_heading
                + (1.0 - self._path_rejoin_heading_weight) * lookahead_heading
            ),
            math.cos(
                self._path_rejoin_heading_weight * path_heading
                + (1.0 - self._path_rejoin_heading_weight) * lookahead_heading
            ),
        )
        heading_err = math.atan2(math.sin(heading_target - yaw), math.cos(heading_target - yaw))
        return (math.sqrt(nearest_d2), heading_err, cross_track)

    def _on_twist(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._publish_output()

    def _publish_output(self) -> None:
        if self._last_cmd is None and not (
            self._emergency or (self._recovery_active and self._recovery_cmd is not None)
        ):
            return

        nav_cmd = self._last_cmd if self._last_cmd is not None else Twist()
        out = Twist()
        lin = max(-self._max_v, min(self._max_v, nav_cmd.linear.x))
        ang = max(-self._max_w, min(self._max_w, nav_cmd.angular.z))

        if self._emergency or self._aux_emergency or self._aux_emergency_2:
            out.linear.x = 0.0
            out.angular.z = 0.0
        elif self._recovery_active and self._recovery_cmd is not None:
            # Recovery override has priority over Nav2 command when active.
            out.linear.x = max(-self._max_v, min(self._max_v, self._recovery_cmd.linear.x))
            out.angular.z = max(-self._max_w, min(self._max_w, self._recovery_cmd.angular.z))
        else:
            s = self._scale
            out.linear.x = lin * s
            out.angular.z = ang * s

            # Ackermann platforms cannot execute in-place rotation robustly.
            # If Nav2 sends near-zero linear velocity with non-trivial yaw
            # rate, enforce a small forward crawl so turns become arcs.
            # Respect controller stop commands near obstacles. Only enforce crawl-turn
            # when the upstream command already requests forward motion.
            should_enforce_crawl = (
                abs(out.angular.z) > self._turn_w_thresh
                and abs(out.linear.x) < self._min_v_turn
            )
            if should_enforce_crawl and (lin > 0.0 or self._enforce_crawl_on_rotate_zero_cmd):
                out.linear.x = self._min_v_turn

            # Path-attractor assist: when close to the global path, ensure the
            # steering direction always points back toward the path heading.
            if self._path_assist_enabled and out.linear.x > 0.0:
                metrics = self._path_tracking_metrics()
                if metrics is not None:
                    dist_to_path, heading_err, cross_track = metrics
                    if (
                        dist_to_path <= self._path_assist_dist
                        and (
                            abs(heading_err) > self._path_assist_heading_deadband
                            or (
                                self._path_rejoin_countersteer_enabled
                                and abs(cross_track) > self._path_rejoin_countersteer_deadband_m
                            )
                        )
                    ):
                        desired_turn = heading_err
                        # Countersteer near/within rejoin band using signed cross-track.
                        # This damps "line crossing ping-pong" by reducing steer once
                        # we are very close to the global path centerline.
                        if (
                            self._path_rejoin_countersteer_enabled
                            and dist_to_path <= self._path_rejoin_band_m
                        ):
                            v_eff = max(self._path_rejoin_speed_floor_mps, abs(out.linear.x))
                            cross_term = math.atan2(
                                self._path_rejoin_cross_track_gain * cross_track, v_eff
                            )
                            desired_turn = heading_err - cross_term
                        desired_sign = 1.0 if desired_turn >= 0.0 else -1.0
                        desired_mag = max(
                            self._path_assist_min_w,
                            min(self._path_assist_max_w, abs(desired_turn)),
                        )
                        out.angular.z = desired_sign * desired_mag
            # Hard front-stop guard independent from planner/logging pathways.
            # If obstacle is too close in front scan cone, suppress forward motion.
            if self._front_collision_blocked and out.linear.x > 0.0:
                out.linear.x = 0.0
                out.angular.z = 0.0

        self._pub.publish(out)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = AckermannSafetyControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
