"""
Obstacle tracker node (pipeline node 2 of 5).

Implements a lightweight **perception + prediction** stack suitable for a small
delivery robot:

1. Convert ``LaserScan`` hits to points in the map frame (via TF).
2. Segment the scan in polar space into contiguous free-space clusters (simple
   gap-based segmentation).
3. Associate cluster centers with persistent track IDs (greedy nearest-neighbour
   with prediction gate).
4. Estimate planar velocity with exponential smoothing (constant-velocity model).

Publishes :class:`delivery_robot_msgs.msg.ObstacleTrackArray` for consumers that
need explicit obstacle motion (local supervisor, costmap adapters, diagnostics).

This is not a full multi-hypothesis tracker; it trades accuracy for CPU use and
deterministic behaviour on dense 2D LiDAR.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

from delivery_robot_msgs.msg import ObstacleTrack, ObstacleTrackArray


@dataclass
class _Track:
    """Internal track state used only inside this node."""

    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    lost: int = 0


@dataclass
class _Cluster:
    """Polar-space cluster: mean angle/range in the laser frame."""

    cx: float
    cy: float
    r_eff: float


class ObstacleTrackerNode(Node):
    """
    Track moving obstacles from filtered laser scans.

    Parameters
    ----------
    scan_topic : str
        Typically ``scan_filtered`` from :class:`SensorProcessorNode`.
    tracks_topic : str
        Output topic for :class:`ObstacleTrackArray`.
    global_frame : str
        Frame for published obstacle positions (usually ``map``).
    laser_frame : str
        Frame id expected in incoming ``LaserScan`` headers (used if TF to map fails).
    max_range_jump : float
        Polar discontinuity threshold (m) to split segments.
    min_segment_points : int
        Minimum rays per segment to accept a cluster.
    association_radius : float
        Max distance (m) in map frame to match a detection to a predicted track.
    velocity_smoothing : float
        Low-pass factor in ``(0, 1]`` for velocity updates.
    max_lost_updates : int
        Remove a track after this many scans without a match.
    """

    def __init__(self) -> None:
        super().__init__("obstacle_tracker_node")

        self.declare_parameter("scan_topic", "scan_filtered")
        self.declare_parameter("tracks_topic", "tracked_obstacles")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("max_range_jump", 0.35)
        self.declare_parameter("min_segment_points", 3)
        self.declare_parameter("association_radius", 0.55)
        self.declare_parameter("velocity_smoothing", 0.35)
        self.declare_parameter("max_lost_updates", 6)
        self.declare_parameter("use_latest_tf_fallback", False)
        self.declare_parameter("max_tf_age_sec", 0.20)
        self.declare_parameter("tf_ignore_scan_timestamp", True)

        self._global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self._laser_frame_param = self.get_parameter("laser_frame").get_parameter_value().string_value
        self._max_jump = self.get_parameter("max_range_jump").get_parameter_value().double_value
        self._min_seg = max(1, self.get_parameter("min_segment_points").get_parameter_value().integer_value)
        self._assoc_r = self.get_parameter("association_radius").get_parameter_value().double_value
        self._vel_alpha = min(
            1.0,
            max(0.05, self.get_parameter("velocity_smoothing").get_parameter_value().double_value),
        )
        self._max_lost = max(1, self.get_parameter("max_lost_updates").get_parameter_value().integer_value)
        self._use_latest_tf_fallback = bool(
            self.get_parameter("use_latest_tf_fallback").get_parameter_value().bool_value
        )
        self._max_tf_age_sec = max(
            0.0, float(self.get_parameter("max_tf_age_sec").get_parameter_value().double_value)
        )
        self._tf_ignore_scan_timestamp = bool(
            self.get_parameter("tf_ignore_scan_timestamp").get_parameter_value().bool_value
        )

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        tracks_topic = self.get_parameter("tracks_topic").get_parameter_value().string_value

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._tracks: Dict[int, _Track] = {}
        self._next_id = 1
        self._prev_stamp: Optional[Time] = None
        self._scan_count = 0
        self.declare_parameter("diagnostic_log_every", 1)
        self.declare_parameter("strict_tf", False)
        self._diag_every = int(self.get_parameter("diagnostic_log_every").get_parameter_value().integer_value)
        self._strict_tf = bool(self.get_parameter("strict_tf").get_parameter_value().bool_value)

        self._sub = self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self._pub = self.create_publisher(ObstacleTrackArray, tracks_topic, 10)

        #self.get_logger().info(
        #    f"ObstacleTrackerNode: '{scan_topic}' -> '{tracks_topic}' (frame={self._global_frame})"
        #)

    def _on_scan(self, msg: LaserScan) -> None:
        stamp = Time.from_msg(msg.header.stamp)
        # allow changing diagnostic verbosity at runtime
        try:
            self._diag_every = int(self.get_parameter("diagnostic_log_every").get_parameter_value().integer_value)
            # allow switching strict TF behavior at runtime
            self._strict_tf = bool(self.get_parameter("strict_tf").get_parameter_value().bool_value)
            self._use_latest_tf_fallback = bool(
                self.get_parameter("use_latest_tf_fallback").get_parameter_value().bool_value
            )
            self._max_tf_age_sec = max(
                0.0, float(self.get_parameter("max_tf_age_sec").get_parameter_value().double_value)
            )
            self._tf_ignore_scan_timestamp = bool(
                self.get_parameter("tf_ignore_scan_timestamp").get_parameter_value().bool_value
            )
        except Exception:
            pass
        dt = 0.1
        if self._prev_stamp is not None:
            dt = max(1e-3, (stamp - self._prev_stamp).nanoseconds / 1e9)
        self._prev_stamp = stamp

        laser_frame = msg.header.frame_id or self._laser_frame_param
        clusters = self._segment_scan(msg)
        detections_map: List[Tuple[float, float, float]] = []
        for c in clusters:
            lm = self._transform_point(laser_frame, c.cx, c.cy, stamp)
            if lm is None:
                continue
            detections_map.append((lm[0], lm[1], c.r_eff))

        # Snapshot positions before motion prediction so velocity uses a consistent dt.
        prev_xy: Dict[int, Tuple[float, float]] = {tid: (tr.x, tr.y) for tid, tr in self._tracks.items()}
        self._predict_tracks(dt)
        self._associate_and_update(detections_map, dt, prev_xy)
        self._prune_tracks()

        out = ObstacleTrackArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._global_frame
        # Use the original scan timestamp for published tracks so downstream
        # consumers (costmaps, planners) see a consistent timebase.
        scan_stamp = msg.header.stamp
        for tr in self._tracks.values():
            ot = ObstacleTrack()
            ot.header.stamp = scan_stamp
            ot.header.frame_id = self._global_frame
            ot.id = tr.track_id
            ot.x = tr.x
            ot.y = tr.y
            ot.vx = tr.vx
            ot.vy = tr.vy
            ot.radius = tr.radius
            ot.confidence = max(0.0, 1.0 - 0.1 * tr.lost)
            out.tracks.append(ot)
        self._pub.publish(out)

        # Diagnostic logging (low-volume): report transform stamps and a sample
        # transformed point every N scans if enabled. This helps detect whether
        # points appear to rotate with the robot due to TF/timestamp issues.
        self._scan_count += 1
        if self._diag_every > 0 and (self._scan_count % self._diag_every) == 0:
            try:
                if len(msg.ranges) > 0:
                    # find first valid range index
                    for i, r in enumerate(msg.ranges):
                        if not (math.isnan(r) or r < msg.range_min or r > msg.range_max):
                            ang = msg.angle_min + i * msg.angle_increment
                            lx = r * math.cos(ang)
                            ly = r * math.sin(ang)
                            # attempt to lookup transform used earlier
                            try:
                                # Reuse the same lookup logic as _transform_point: try
                                # an exact-time lookup and fall back to the latest
                                # available transform. Report which stamp was used so
                                # diagnostics reflect actual behavior.
                                try:
                                    tf = self._tf_buffer.lookup_transform(
                                        self._global_frame,
                                        msg.header.frame_id or self._laser_frame_param,
                                        Time.from_msg(msg.header.stamp),
                                        timeout=Duration(seconds=0.15),
                                    )
                                    used = "exact"
                                except TransformException as ex1:
                                    try:
                                        tf = self._tf_buffer.lookup_transform(
                                            self._global_frame,
                                            msg.header.frame_id or self._laser_frame_param,
                                            Time(),
                                            timeout=Duration(seconds=0.15),
                                        )
                                        used = "latest"
                                    except TransformException as ex2:
                                        raise ex2 from ex1

                                rot = tf.transform.rotation
                                # yaw from quaternion
                                yaw = math.atan2(
                                    2.0 * (rot.w * rot.z + rot.x * rot.y),
                                    1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z),
                                )
                                ###
                                #self.get_logger().info(
                                #    f"diag_scan idx={i} range={r:.3f} laser_pt=({lx:.3f},{ly:.3f}) "
                                #    f"tf_used={used} tf_stamp={tf.header.stamp.sec}.{tf.header.stamp.nanosec:09d} tf_yaw={yaw:.3f} "
                                #    f"scan_stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
                                #)
                            except Exception as ex:
                                #self.get_logger().info(f"diag_scan TF lookup failed: {ex}")
                                pass
                            break
            except Exception:
                pass

    def _segment_scan(self, msg: LaserScan) -> List[_Cluster]:
        """Split the scan into polar segments; return cluster centroids in laser frame."""
        ranges = msg.ranges
        n = len(ranges)
        if n < 3:
            return []

        angles = [msg.angle_min + i * msg.angle_increment for i in range(n)]
        clusters: List[_Cluster] = []
        i = 0
        while i < n:
            r = ranges[i]
            if math.isnan(r) or r < msg.range_min or r > msg.range_max:
                i += 1
                continue
            j = i
            seg_r: List[float] = []
            seg_a: List[float] = []
            while j < n:
                rj = ranges[j]
                if math.isnan(rj) or rj < msg.range_min or rj > msg.range_max:
                    break
                if not seg_r:
                    seg_r.append(rj)
                    seg_a.append(angles[j])
                    j += 1
                    continue
                if abs(rj - seg_r[-1]) > self._max_jump:
                    break
                seg_r.append(rj)
                seg_a.append(angles[j])
                j += 1

            if len(seg_r) >= self._min_seg:
                mean_a = sum(seg_a) / len(seg_a)
                mean_r = sum(seg_r) / len(seg_r)
                cx = mean_r * math.cos(mean_a)
                cy = mean_r * math.sin(mean_a)
                span = max(seg_r) - min(seg_r)
                clusters.append(_Cluster(cx=cx, cy=cy, r_eff=max(0.05, 0.5 * span)))
            i = j if j > i else i + 1

        return clusters

    def _transform_point(
        self, laser_frame: str, x: float, y: float, stamp: Time
    ) -> Optional[Tuple[float, float, float]]:
        """Return (map_x, map_y, radius) or None if TF fails."""
        ps = PointStamped()
        ps.header.frame_id = laser_frame
        ps.header.stamp = stamp.to_msg()
        ps.point.x = x
        ps.point.y = y
        ps.point.z = 0.0
        # In simulation, exact scan-time TF often causes avoidable extrapolation
        # drops and makes obstacle positions appear unstable. When enabled,
        # ignore scan timestamp and use the latest available transform on arrival.
        if self._tf_ignore_scan_timestamp:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._global_frame,
                    laser_frame,
                    Time(),
                    timeout=Duration(seconds=0.15),
                )
                out = tf2_geometry_msgs.do_transform_point(ps, tf)
                return (out.point.x, out.point.y, 0.12)
            except TransformException:
                if self._strict_tf:
                    raise RuntimeError(
                        f"Strict TF mode: failed latest transform {laser_frame} -> {self._global_frame}"
                    )
                return None

        # Exact-time transform mode (legacy behavior).
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                laser_frame,
                stamp,
                timeout=Duration(seconds=0.15),
            )
            tf_stamp = Time.from_msg(tf.header.stamp)
            tf_age = abs((stamp - tf_stamp).nanoseconds) / 1e9
            if tf_age > self._max_tf_age_sec:
                return None
        except TransformException as ex:
            if not self._use_latest_tf_fallback:
                if self._strict_tf:
                    raise RuntimeError(
                        f"Strict TF mode: failed exact-time transform {laser_frame} -> {self._global_frame} at {stamp.to_msg()}: {ex}"
                    )
                return None
            # try latest available transform as a fallback
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._global_frame,
                    laser_frame,
                    Time(),
                    timeout=Duration(seconds=0.15),
                )
                tf_stamp = Time.from_msg(tf.header.stamp)
                tf_age = abs((stamp - tf_stamp).nanoseconds) / 1e9
                if tf_age > self._max_tf_age_sec:
                    return None
            except TransformException as ex2:
                if self._strict_tf:
                    raise RuntimeError(
                        f"Strict TF mode: failed to lookup transform {laser_frame} -> {self._global_frame} at {stamp.to_msg()}"
                    )
                return None

        out = tf2_geometry_msgs.do_transform_point(ps, tf)
        return (out.point.x, out.point.y, 0.12)

    def _predict_tracks(self, dt: float) -> None:
        for tr in self._tracks.values():
            tr.x += tr.vx * dt
            tr.y += tr.vy * dt

    def _associate_and_update(
        self,
        detections: List[Tuple[float, float, float]],
        dt: float,
        prev_xy: Dict[int, Tuple[float, float]],
    ) -> None:
        """Greedy assignment of detections to nearest predicted tracks."""
        used_det: set[int] = set()
        used_tracks: set[int] = set()

        det_list = list(enumerate(detections))
        tr_list = [(tid, tr) for tid, tr in self._tracks.items()]

        pairs: List[Tuple[float, int, int]] = []
        for di, (dx, dy, dr) in det_list:
            for tid, tr in tr_list:
                d = math.hypot(dx - tr.x, dy - tr.y)
                if d <= self._assoc_r:
                    pairs.append((d, di, tid))

        pairs.sort(key=lambda x: x[0])
        for _, di, tid in pairs:
            if di in used_det or tid in used_tracks:
                continue
            used_det.add(di)
            used_tracks.add(tid)
            dx, dy, dr = detections[di]
            tr = self._tracks[tid]
            px, py = prev_xy.get(tid, (tr.x, tr.y))
            vx_meas = (dx - px) / dt if dt > 1e-6 else 0.0
            vy_meas = (dy - py) / dt if dt > 1e-6 else 0.0
            tr.x = dx
            tr.y = dy
            tr.radius = max(tr.radius * 0.8 + 0.2 * dr, dr)
            a = self._vel_alpha
            tr.vx = (1 - a) * tr.vx + a * vx_meas
            tr.vy = (1 - a) * tr.vy + a * vy_meas
            tr.lost = 0

        for di, (dx, dy, dr) in enumerate(detections):
            if di in used_det:
                continue
            tid = self._next_id
            self._next_id += 1
            self._tracks[tid] = _Track(
                track_id=tid,
                x=dx,
                y=dy,
                vx=0.0,
                vy=0.0,
                radius=dr,
                lost=0,
            )

        for tid, tr in self._tracks.items():
            if tid not in used_tracks:
                tr.lost += 1

    def _prune_tracks(self) -> None:
        dead = [tid for tid, tr in self._tracks.items() if tr.lost > self._max_lost]
        for tid in dead:
            del self._tracks[tid]


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ObstacleTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
