"""
Local planner / navigation supervisor node (pipeline node 3 of 5).

This node bridges **tracked dynamic obstacles** to Nav2-style consumers:

* Publishes a ``sensor_msgs/PointCloud2`` of predicted obstacle positions so an
  ``obstacle_layer`` (or ``voxel_layer``) in ``nav2_costmap_2d`` can inflate
  around *future* locations, not only the current laser sweep.
* Subscribes to the global path (``nav_msgs/Path``) when available and computes a
  conservative **speed scale** and **emergency stop** flag from the closest
  approach between the path corridor and predicted obstacles.

It does **not** replace Nav2's controller server; it adds obstacle-aware
slow-down and stop signals that the :class:`AckermannSafetyControllerNode` can
apply to ``cmd_vel``. This matches a typical small delivery-robot stack where
global routing stays in Nav2 while short-horizon safety is enforced in Python.
"""

from __future__ import annotations

import math
import struct
from typing import List, Optional, Tuple

import rclpy
import time
from rclpy.node import Node

from delivery_robot_msgs.msg import ObstacleTrackArray
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32, Header


def _make_pointcloud_xyz(header: Header, points: List[Tuple[float, float, float]]) -> PointCloud2:
    """Build a dense XYZ float32 ``PointCloud2`` without extra dependencies."""
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    buf = bytearray()
    for x, y, z in points:
        buf.extend(struct.pack("fff", x, y, z))
    cloud.data = bytes(buf)
    return cloud


def _dist_point_to_segment(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """2D distance from P to segment AB."""
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(px - qx, py - qy)


class LocalPlannerNode(Node):
    """
    Publish predicted obstacle clouds and path-based speed limits.

    Parameters
    ----------
    tracks_topic : str
        Input :class:`ObstacleTrackArray`.
    path_topic : str
        Global or local path from Nav2 (often ``plan`` or ``plan_smoothed``).
    obstacle_cloud_topic : str
        Output ``PointCloud2`` for costmap plugins.
    speed_scale_topic : str
        Output ``Float32`` in ``[0, 1]`` for downstream velocity scaling.
    emergency_stop_topic : str
        Output ``Bool`` when the robot should not move.
    global_frame : str
        Expected frame for path and tracks (``map``).
    prediction_horizon : float
        Seconds ahead to sample motion (default ``0.6``).
    prediction_samples : int
        Number of time steps ``0 .. horizon`` inclusive.
    robot_safety_radius : float
        Extra clearance around the path (m), combined with obstacle ``radius``.
    slow_clearance : float
        Distance (m) at which speed scale reaches ``1.0``.
    hard_clearance : float
        Distance (m) below which emergency stop is asserted.
    """

    def __init__(self) -> None:
        super().__init__("local_planner_node")

        self.declare_parameter("tracks_topic", "tracked_obstacles")
        # Nav2 often exposes ``plan``; our Python planner publishes ``planned_path`` by default.
        self.declare_parameter("path_topic", "planned_path")
        self.declare_parameter("obstacle_cloud_topic", "tracked_obstacles_cloud")
        self.declare_parameter("speed_scale_topic", "nav_speed_scale")
        self.declare_parameter("emergency_stop_topic", "nav_emergency_stop")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("prediction_horizon", 0.6)
        self.declare_parameter("prediction_samples", 4)
        self.declare_parameter("robot_safety_radius", 0.25)
        self.declare_parameter("slow_clearance", 1.2)
        self.declare_parameter("hard_clearance", 0.12)
        self.declare_parameter("path_safety_lookahead_m", 3.0)
        self.declare_parameter("dynamic_only_for_safety", True)
        self.declare_parameter("dynamic_track_speed_min", 0.08)
        self.declare_parameter("disable_emergency_stop", True) #temporary, testing
        self.declare_parameter("emergency_trigger_ticks", 3)
        self.declare_parameter("emergency_release_clearance", 0.22)
        self.declare_parameter("cloud_flush_interval", 0.5)
    # Replan request / in-progress handling to slow robot while planner replans
    self.declare_parameter("request_replan_topic", "nav_request_replan")
    self.declare_parameter("replan_in_progress_topic", "nav_replan_in_progress")
    self.declare_parameter("replan_request_clearance", 0.0)
    self.declare_parameter("replan_debounce_ticks", 2)
    self.declare_parameter("replan_rate_limit_s", 2.0)
    self.declare_parameter("replan_speed_floor", 0.4)

        self._global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self._horizon = max(0.0, self.get_parameter("prediction_horizon").get_parameter_value().double_value)
        self._samples = max(1, self.get_parameter("prediction_samples").get_parameter_value().integer_value)
        self._robot_rr = self.get_parameter("robot_safety_radius").get_parameter_value().double_value
        self._slow_d = max(0.05, self.get_parameter("slow_clearance").get_parameter_value().double_value)
        self._hard_d = max(0.0, self.get_parameter("hard_clearance").get_parameter_value().double_value)
        self._path_safety_lookahead_m = max(
            0.5, self.get_parameter("path_safety_lookahead_m").get_parameter_value().double_value
        )
        self._dynamic_only_for_safety = (
            self.get_parameter("dynamic_only_for_safety").get_parameter_value().bool_value
        )
        self._dynamic_track_speed_min = max(
            0.0, self.get_parameter("dynamic_track_speed_min").get_parameter_value().double_value
        )
        self._disable_emergency_stop = (
            self.get_parameter("disable_emergency_stop").get_parameter_value().bool_value
        )
        self._emergency_trigger_ticks = max(
            1, self.get_parameter("emergency_trigger_ticks").get_parameter_value().integer_value
        )
        self._emergency_release_clearance = max(
            self._hard_d,
            self.get_parameter("emergency_release_clearance").get_parameter_value().double_value,
        )
        self._cloud_flush_interval = max(0.0, float(self.get_parameter("cloud_flush_interval").get_parameter_value().double_value))

        tracks_topic = self.get_parameter("tracks_topic").get_parameter_value().string_value
        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        cloud_topic = self.get_parameter("obstacle_cloud_topic").get_parameter_value().string_value
        scale_topic = self.get_parameter("speed_scale_topic").get_parameter_value().string_value
        emerg_topic = self.get_parameter("emergency_stop_topic").get_parameter_value().string_value

    # Replan topics and state
    self._request_replan_topic = self.get_parameter("request_replan_topic").get_parameter_value().string_value
    self._replan_in_progress_topic = self.get_parameter("replan_in_progress_topic").get_parameter_value().string_value
    self._replan_request_clearance = float(self.get_parameter("replan_request_clearance").get_parameter_value().double_value)
    self._replan_debounce_ticks = int(self.get_parameter("replan_debounce_ticks").get_parameter_value().integer_value)
    self._replan_rate_limit_s = float(self.get_parameter("replan_rate_limit_s").get_parameter_value().double_value)
    self._replan_speed_floor = float(self.get_parameter("replan_speed_floor").get_parameter_value().double_value)
    self._replan_debounce = 0
    self._last_replan_request_time = 0.0
    self._replan_in_progress = False

        self._latest_path: Optional[Path] = None
        self._latest_tracks: Optional[ObstacleTrackArray] = None

        self.create_subscription(ObstacleTrackArray, tracks_topic, self._on_tracks, 10)
        self.create_subscription(Path, path_topic, self._on_path, 10)

        self._pub_cloud = self.create_publisher(PointCloud2, cloud_topic, 10)
        self._pub_scale = self.create_publisher(Float32, scale_topic, 10)
        self._pub_emerg = self.create_publisher(Bool, emerg_topic, 10)
    self._pub_request_replan = self.create_publisher(Bool, self._request_replan_topic, 1)
    # subscribe to replan in-progress flag so we can reduce speed while planner works
    self.create_subscription(Bool, self._replan_in_progress_topic, self._on_replan_in_progress, 10)

        self._timer = self.create_timer(0.05, self._tick)
        self._last_flush_walltime = 0.0
        self._emergency_counter = 0
        self._emergency_active = False

        self.get_logger().info(
            f"LocalPlannerNode: tracks='{tracks_topic}', path='{path_topic}', cloud='{cloud_topic}'"
        )

    def _on_path(self, msg: Path) -> None:
        self._latest_path = msg

    def _on_tracks(self, msg: ObstacleTrackArray) -> None:
        self._latest_tracks = msg

    def _on_replan_in_progress(self, msg: Bool) -> None:
        try:
            new_val = bool(msg.data)
            # Log only when the state changes to reduce console spam
            if new_val and not self._replan_in_progress:
                self.get_logger().info("Replan in progress: slowing robot until new plan is available")
            elif not new_val and self._replan_in_progress:
                self.get_logger().info("Replan complete: resuming normal speed")
            self._replan_in_progress = new_val
        except Exception:
            self._replan_in_progress = False

    def _tick(self) -> None:
        tracks = self._latest_tracks
        if tracks is None:
            return

        now = time.monotonic()
        # Periodically publish an empty cloud to encourage downstream costmaps
        # to forget previously-seen obstacle points before we publish the fresh
        # predicted obstacle cloud. This helps moving obstacles free space they
        # previously occupied so the planner can replan into cleared areas.
        if self._cloud_flush_interval > 0.0 and (now - self._last_flush_walltime) >= self._cloud_flush_interval:
            empty_header = Header()
            empty_header.stamp = tracks.header.stamp
            empty_header.frame_id = self._global_frame
            empty_cloud = PointCloud2()
            empty_cloud.header = empty_header
            empty_cloud.height = 1
            empty_cloud.width = 0
            empty_cloud.fields = []
            empty_cloud.is_bigendian = False
            empty_cloud.point_step = 0
            empty_cloud.row_step = 0
            empty_cloud.is_dense = True
            empty_cloud.data = b""
            self._pub_cloud.publish(empty_cloud)
            self._last_flush_walltime = now

        stamp = tracks.header.stamp
        header = Header()
        header.stamp = stamp
        header.frame_id = self._global_frame

        pts: List[Tuple[float, float, float]] = []
        for tr in tracks.tracks:
            for k in range(self._samples):
                t = self._horizon * k / max(1, self._samples - 1)
                x = tr.x + tr.vx * t
                y = tr.y + tr.vy * t
                pts.append((x, y, 0.0))
                # Ring around obstacle for inflation-friendly sampling
                r = max(0.05, tr.radius)
                for a in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                    pts.append((x + r * math.cos(a), y + r * math.sin(a), 0.0))

        self._pub_cloud.publish(_make_pointcloud_xyz(header, pts))

        path = self._latest_path
        if path is None or len(path.poses) < 2:
            self._pub_scale.publish(Float32(data=1.0))
            self._pub_emerg.publish(Bool(data=False))
            return

        # Path polyline in map frame
        segs: List[Tuple[float, float, float, float]] = []
        total_len = 0.0
        for i in range(len(path.poses) - 1):
            ax = path.poses[i].pose.position.x
            ay = path.poses[i].pose.position.y
            bx = path.poses[i + 1].pose.position.x
            by = path.poses[i + 1].pose.position.y
            segs.append((ax, ay, bx, by))
            total_len += math.hypot(bx - ax, by - ay)

        # Evaluate safety only on the near-term part of the path. Using the
        # entire global path can hold emergency stop high because of obstacles
        # far ahead, preventing any initial motion.
        segs_eval: List[Tuple[float, float, float, float]] = []
        accum = 0.0
        for ax, ay, bx, by in segs:
            seg_len = math.hypot(bx - ax, by - ay)
            if seg_len <= 1e-6:
                continue
            if accum >= self._path_safety_lookahead_m:
                break
            segs_eval.append((ax, ay, bx, by))
            accum += seg_len
        if not segs_eval and segs:
            segs_eval = [segs[0]]

        min_clearance = float("inf")
        for tr in tracks.tracks:
            if self._dynamic_only_for_safety:
                spd = math.hypot(tr.vx, tr.vy)
                if spd < self._dynamic_track_speed_min:
                    continue
            for k in range(self._samples):
                t = self._horizon * k / max(1, self._samples - 1)
                ox = tr.x + tr.vx * t
                oy = tr.y + tr.vy * t
                orad = max(0.05, tr.radius) + self._robot_rr
                for ax, ay, bx, by in segs_eval:
                    d = _dist_point_to_segment(ox, oy, ax, ay, bx, by) - orad
                    if d < min_clearance:
                        min_clearance = d

        if min_clearance == float("inf"):
            # No dynamic tracks considered hazardous right now.
            min_clearance = self._slow_d

        if min_clearance < self._hard_d:
            self._emergency_counter += 1
        else:
            self._emergency_counter = 0

        if self._emergency_active:
            # Hysteresis: once emergency stop is active, require a safer
            # clearance than the trigger threshold to release it.
            emerg = min_clearance < self._emergency_release_clearance
        else:
            emerg = self._emergency_counter >= self._emergency_trigger_ticks
        self._emergency_active = emerg
        if self._disable_emergency_stop:
            emerg = False
            self._emergency_active = False
            self._emergency_counter = 0
        if emerg:
            scale = 0.0
        elif min_clearance >= self._slow_d:
            scale = 1.0
        else:
            scale = max(0.0, min_clearance / self._slow_d)

        # Request replan if path is blocked (debounced + rate-limited)
        now = time.monotonic()
        if min_clearance < self._replan_request_clearance:
            self._replan_debounce += 1
        else:
            self._replan_debounce = 0
        if (
            self._replan_debounce >= self._replan_debounce_ticks
            and (now - self._last_replan_request_time) >= self._replan_rate_limit_s
        ):
            try:
                self._pub_request_replan.publish(Bool(data=True))
                self._last_replan_request_time = now
                self._replan_debounce = 0
            except Exception:
                pass

        # If a replan is in progress, reduce speed to the configured floor
        if self._replan_in_progress:
            scale = min(scale, self._replan_speed_floor)

        self._pub_scale.publish(Float32(data=float(scale)))
        self._pub_emerg.publish(Bool(data=emerg))


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
