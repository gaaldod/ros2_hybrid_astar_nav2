from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
import json
import math
import os
import re
import struct
import threading
import time
from typing import Optional

import rclpy
from rcl_interfaces.msg import Log
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformException, TransformListener

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .grid_map import OccupancyGridMap
from .hybrid_astar import HybridAStarPlanner
from .types import GridInfo, PlanResult, Pose2D


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _pose_to_pose2d(ps: PoseStamped) -> Pose2D:
    return Pose2D(
        x=ps.pose.position.x,
        y=ps.pose.position.y,
        yaw=_yaw_from_quaternion(ps.pose.orientation),
    )


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _lerp_angle(a0: float, a1: float, t: float) -> float:
    t_clamped = max(0.0, min(1.0, t))
    return _norm_angle(a0 + _norm_angle(a1 - a0) * t_clamped)


class Nav2HybridAStarServer(Node):
    """
    Nav2-compatible planning action servers implemented in Python.

    Provides:
    - `compute_path_to_pose` (nav2_msgs/action/ComputePathToPose)
    - `compute_path_through_poses` (nav2_msgs/action/ComputePathThroughPoses)

    This lets you run Nav2 BT Navigator without the default `planner_server`.
    """

    def __init__(self) -> None:
        super().__init__("nav2_hybrid_astar_server")

        self.declare_parameter("map_topic", "map")
        self.declare_parameter("path_topic", "planned_path")
        self.declare_parameter("expansions_topic", "search_expansions")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("minimum_turning_radius", 1.35)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("angle_quantization_bins", 72)
        self.declare_parameter("allow_primitive_interpolation", True)
        self.declare_parameter("reverse_penalty", 2.0)
        self.declare_parameter("non_straight_penalty", 0.6)
        self.declare_parameter("direction_change_penalty", 0.3)
        self.declare_parameter("steering_change_penalty", 0.1)
        self.declare_parameter("planner_max_iterations", 25000)
        self.declare_parameter("planner_timeout_sec", 3.0)
        self.declare_parameter("plan_fallback_ttl_sec", 1.5)
        self.declare_parameter("plan_fallback_keepalive_dist_to_global_m", 0.8)
        self.declare_parameter("plan_fallback_max_global_age_sec", 20.0)
        self.declare_parameter("plan_fallback_max_dist_for_age_m", 1.6)
        self.declare_parameter("wall_memory_enabled", True)
        self.declare_parameter("wall_memory_path", "/tmp/hybrid_astar_wall_memory/walls.json")
        self.declare_parameter("wall_memory_inflation_radius_m", 0.7)
        self.declare_parameter("wall_memory_max_points", 300)
        self.declare_parameter("wall_memory_min_separation_m", 0.6)
        self.declare_parameter("wall_memory_capture_cooldown_sec", 0.75)
        self.declare_parameter("wall_memory_collision_substring", "RegulatedPurePursuitController detected collision ahead!")
        self.declare_parameter("wall_memory_controller_logger_name", "controller_server")
        self.declare_parameter("live_obstacle_cloud_enabled", True)
        self.declare_parameter("live_obstacle_cloud_topic", "tracked_obstacles_cloud")
        self.declare_parameter("live_obstacle_cloud_timeout_sec", 1.5)
        self.declare_parameter("live_obstacle_inflation_radius_m", 0.8)
        self.declare_parameter("live_obstacle_ignore_near_start_m", 0.8)
        self.declare_parameter("path_hold_enabled", True)
        self.declare_parameter("path_hold_goal_tolerance_m", 0.6)
        self.declare_parameter("path_hold_goal_tolerance_yaw_rad", 0.6)
        self.declare_parameter("path_hold_obstacle_ahead_radius_m", 2.5)
        self.declare_parameter("path_hold_obstacle_ahead_half_angle_rad", 0.75)
        self.declare_parameter("path_hold_cloud_block_min_drift_m", 0.7)
        self.declare_parameter("path_live_obstacle_min_clearance_m", 1.0)
        self.declare_parameter("path_live_obstacle_soft_accept_min_clearance_m", 0.45)
        self.declare_parameter("path_clearance_retry_count", 2)
        self.declare_parameter("path_clearance_retry_inflation_step_m", 0.3)
        self.declare_parameter("path_none_retry_relax_live_overlay", True)
        self.declare_parameter("path_none_retry_min_inflation_m", 0.35)
        self.declare_parameter("planning_grid_static_inflation_radius_m", 0.35)
        self.declare_parameter("path_segment_obstacle_clearance_m", 0.9)
        self.declare_parameter("path_segment_sample_step_m", 0.10)
        self.declare_parameter("segment_treat_unknown_as_occupied", False)
        self.declare_parameter("planner_treat_unknown_as_occupied", False)
        self.declare_parameter("rejected_path_block_radius_m", 0.6)
        self.declare_parameter("rejected_path_memory_ttl_sec", 8.0)
        # If False, segment-grid-only rejects are not stored in rejected-path memory.
        # This avoids over-blocking narrow map corridors when no live obstacles exist.
        self.declare_parameter("remember_segment_grid_rejects", False)
        # Block only around live obstacle points that are close to rejected paths.
        self.declare_parameter("rejected_obstacle_near_path_radius_m", 1.0)
        self.declare_parameter("rejected_obstacle_block_radius_m", 1.0)
        self.declare_parameter("path_clearance_start_grace_m", 1.2)
        self.declare_parameter("start_progress_projection_ratio", 0.20)
        self.declare_parameter("start_progress_projection_max_m", 0.50)
        self.declare_parameter("start_heading_to_goal_blend", 0.20)
        self.declare_parameter("target_heading_relax_distance_m", 3.0)
        self.declare_parameter("target_heading_relax_blend", 0.20)
        self.declare_parameter("disallow_initial_reverse", True)
        self.declare_parameter("allow_initial_reverse_when_goal_behind_deg", 100.0)
        self.declare_parameter("allow_initial_reverse_when_obstacle_ahead", True)
        self.declare_parameter("allow_initial_reverse_obstacle_ahead_radius_m", 2.5)
        # Local takeover mode: when global path is good but obstacle appears ahead,
        # publish short-horizon micro-paths at a fixed rate until rejoined.
        self.declare_parameter("local_takeover_enabled", True)
        self.declare_parameter("local_takeover_rate_hz", 2.5)
        self.declare_parameter("local_takeover_trigger_obstacle_ahead_radius_m", 2.5)
        self.declare_parameter("local_takeover_path_proximity_m", 1.5)
        self.declare_parameter("local_takeover_min_drift_without_collision_hint_m", 0.6)
        self.declare_parameter("local_takeover_horizon_m", 4.0)
        self.declare_parameter("local_takeover_min_horizon_m", 1.5)
        self.declare_parameter("local_takeover_anchor_advance_points", 8)
        self.declare_parameter("local_takeover_forward_only", True)
        self.declare_parameter("local_takeover_start_from_front_m", 0.35)
        self.declare_parameter("local_takeover_min_forward_progress_m", 0.08)
        self.declare_parameter("local_takeover_retry_backoff_sec", 0.8)
        self.declare_parameter("local_takeover_suppressed_backoff_sec", 1.0)
        self.declare_parameter("local_takeover_microplan_timeout_sec", 2.5)
        self.declare_parameter("local_takeover_microplan_iterations_ratio", 0.75)
        self.declare_parameter("local_takeover_bridge_min_poses", 5)
        self.declare_parameter("local_takeover_bridge_min_point_spacing_m", 0.20)
        self.declare_parameter("local_takeover_post_jump_block_sec", 6.0)
        self.declare_parameter("local_takeover_keepalive_sec", 25.0)
        self.declare_parameter("local_takeover_fast_fallback_window_sec", 1.5)
        self.declare_parameter(
            "local_takeover_collision_substring",
            "RegulatedPurePursuitController detected collision ahead!",
        )
        self.declare_parameter("local_takeover_collision_logger_name", "controller_server")
        self.declare_parameter(
            "local_takeover_controller_stall_substring", "Resulting plan has 0 poses in it."
        )
        self.declare_parameter("local_takeover_collision_hint_sec", 2.5)
        # Acceptable cone (degrees) in front of the vehicle for the first
        # planning waypoint. If the first motion in the plan falls outside
        # this cone the planner will attempt a forward-only replan.
        # For Ackermann vehicles this approximates checking that the first
        # primitive is generally in front of the vehicle. Typical values: 30-45.
        self.declare_parameter("initial_forward_cone_deg", 45.0)
        self.declare_parameter("localization_jump_guard_enabled", True)
        self.declare_parameter("localization_jump_amcl_topic", "/amcl_pose")
        self.declare_parameter("localization_jump_dist_m", 0.9)
        self.declare_parameter("localization_jump_yaw_deg", 20.0)
        self.declare_parameter("localization_jump_guard_sec", 2.0)
        self.declare_parameter("localization_jump_emergency_stop_enabled", True)
        self.declare_parameter("localization_jump_emergency_stop_topic", "nav_localization_jump_stop")
        self.declare_parameter("localization_jump_burst_window_sec", 3.0)
        self.declare_parameter("localization_jump_burst_count_threshold", 3)
        self.declare_parameter("localization_jump_burst_guard_sec", 8.0)
        self.declare_parameter("localization_reseed_on_deathloop_enabled", True)
        self.declare_parameter("localization_reseed_plan_zero_threshold", 8)
        self.declare_parameter("localization_reseed_plan_zero_window_sec", 2.0)
        self.declare_parameter("localization_reseed_cooldown_sec", 20.0)
        self.declare_parameter("localization_reseed_initialpose_topic", "/initialpose")
        self.declare_parameter("localization_reseed_use_startup_yaw", True)
        self.declare_parameter("localization_reseed_require_stable_sec", 0.8)

    # Replan request topic and in-progress indicator
    self.declare_parameter("request_replan_topic", "nav_request_replan")
    self.declare_parameter("replan_in_progress_topic", "nav_replan_in_progress")
    self.declare_parameter("replan_quick_timeout_sec", 2.0)
    self.declare_parameter("replan_rate_limit_s", 2.0)

        map_topic = self.get_parameter("map_topic").value
        self._global_frame = self.get_parameter("global_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._planner_cfg = {
            "minimum_turning_radius": float(self.get_parameter("minimum_turning_radius").value),
            "allow_reverse": bool(self.get_parameter("allow_reverse").value),
            "angle_quantization_bins": int(self.get_parameter("angle_quantization_bins").value),
            "allow_primitive_interpolation": bool(
                self.get_parameter("allow_primitive_interpolation").value
            ),
            "reverse_penalty": float(self.get_parameter("reverse_penalty").value),
            "non_straight_penalty": float(self.get_parameter("non_straight_penalty").value),
            "direction_change_penalty": float(self.get_parameter("direction_change_penalty").value),
            "steering_change_penalty": float(self.get_parameter("steering_change_penalty").value),
            "treat_unknown_as_occupied": bool(
                self.get_parameter("planner_treat_unknown_as_occupied").value
            ),
        }
        self._planner_max_iterations = int(self.get_parameter("planner_max_iterations").value)
        self._planner_timeout_sec = float(self.get_parameter("planner_timeout_sec").value)
        self._plan_fallback_ttl_sec = float(self.get_parameter("plan_fallback_ttl_sec").value)
        self._plan_fallback_keepalive_dist_to_global_m = max(
            0.1, float(self.get_parameter("plan_fallback_keepalive_dist_to_global_m").value)
        )
        self._plan_fallback_max_global_age_sec = max(
            1.0, float(self.get_parameter("plan_fallback_max_global_age_sec").value)
        )
        self._plan_fallback_max_dist_for_age_m = max(
            self._plan_fallback_keepalive_dist_to_global_m,
            float(self.get_parameter("plan_fallback_max_dist_for_age_m").value),
        )
        self._wall_memory_enabled = bool(self.get_parameter("wall_memory_enabled").value)
        self._wall_memory_path = str(self.get_parameter("wall_memory_path").value)
        self._wall_memory_inflation_radius_m = float(
            self.get_parameter("wall_memory_inflation_radius_m").value
        )
        self._wall_memory_max_points = int(self.get_parameter("wall_memory_max_points").value)
        self._wall_memory_min_separation_m = float(
            self.get_parameter("wall_memory_min_separation_m").value
        )
        self._wall_memory_capture_cooldown_sec = float(
            self.get_parameter("wall_memory_capture_cooldown_sec").value
        )
        self._wall_memory_collision_substring = str(
            self.get_parameter("wall_memory_collision_substring").value
        )
        self._wall_memory_controller_logger_name = str(
            self.get_parameter("wall_memory_controller_logger_name").value
        )
        self._live_obstacle_cloud_enabled = bool(
            self.get_parameter("live_obstacle_cloud_enabled").value
        )
        self._live_obstacle_cloud_topic = str(
            self.get_parameter("live_obstacle_cloud_topic").value
        )
        self._live_obstacle_cloud_timeout_sec = float(
            self.get_parameter("live_obstacle_cloud_timeout_sec").value
        )
        self._live_obstacle_inflation_radius_m = float(
            self.get_parameter("live_obstacle_inflation_radius_m").value
        )
        self._live_obstacle_ignore_near_start_m = max(
            0.0, float(self.get_parameter("live_obstacle_ignore_near_start_m").value)
        )
        self._path_hold_enabled = bool(self.get_parameter("path_hold_enabled").value)
        self._path_hold_goal_tolerance_m = float(
            self.get_parameter("path_hold_goal_tolerance_m").value
        )
        self._path_hold_goal_tolerance_yaw = float(
            self.get_parameter("path_hold_goal_tolerance_yaw_rad").value
        )
        self._path_hold_obstacle_ahead_radius = float(
            self.get_parameter("path_hold_obstacle_ahead_radius_m").value
        )
        self._path_hold_obstacle_ahead_half_angle = float(
            self.get_parameter("path_hold_obstacle_ahead_half_angle_rad").value
        )
        self._path_hold_cloud_block_min_drift_m = max(
            0.0, float(self.get_parameter("path_hold_cloud_block_min_drift_m").value)
        )
        self._path_live_obstacle_min_clearance = max(
            0.0, float(self.get_parameter("path_live_obstacle_min_clearance_m").value)
        )
        self._path_live_obstacle_soft_accept_min_clearance = max(
            0.0, float(self.get_parameter("path_live_obstacle_soft_accept_min_clearance_m").value)
        )
        self._path_clearance_retry_count = max(
            0, int(self.get_parameter("path_clearance_retry_count").value)
        )
        self._path_clearance_retry_inflation_step = max(
            0.0, float(self.get_parameter("path_clearance_retry_inflation_step_m").value)
        )
        self._path_none_retry_relax_live_overlay = bool(
            self.get_parameter("path_none_retry_relax_live_overlay").value
        )
        self._path_none_retry_min_inflation_m = max(
            0.0, float(self.get_parameter("path_none_retry_min_inflation_m").value)
        )
        self._planning_grid_static_inflation_radius_m = max(
            0.0, float(self.get_parameter("planning_grid_static_inflation_radius_m").value)
        )
        self._path_segment_obstacle_clearance_m = max(
            0.0, float(self.get_parameter("path_segment_obstacle_clearance_m").value)
        )
        self._path_segment_sample_step_m = max(
            0.02, float(self.get_parameter("path_segment_sample_step_m").value)
        )
        self._segment_treat_unknown_as_occupied = bool(
            self.get_parameter("segment_treat_unknown_as_occupied").value
        )
        self._rejected_path_block_radius_m = max(
            0.1, float(self.get_parameter("rejected_path_block_radius_m").value)
        )
        self._rejected_path_memory_ttl_sec = max(
            0.0, float(self.get_parameter("rejected_path_memory_ttl_sec").value)
        )
        self._remember_segment_grid_rejects = bool(
            self.get_parameter("remember_segment_grid_rejects").value
        )
        self._rejected_obstacle_near_path_radius_m = max(
            0.1, float(self.get_parameter("rejected_obstacle_near_path_radius_m").value)
        )
        self._rejected_obstacle_block_radius_m = max(
            0.1, float(self.get_parameter("rejected_obstacle_block_radius_m").value)
        )
        self._path_clearance_start_grace_m = max(
            0.0, float(self.get_parameter("path_clearance_start_grace_m").value)
        )
        self._start_progress_projection_ratio = max(
            0.0, float(self.get_parameter("start_progress_projection_ratio").value)
        )
        self._start_progress_projection_max_m = max(
            0.0, float(self.get_parameter("start_progress_projection_max_m").value)
        )
        self._start_heading_to_goal_blend = max(
            0.0, min(1.0, float(self.get_parameter("start_heading_to_goal_blend").value))
        )
        self._target_heading_relax_distance_m = max(
            0.0, float(self.get_parameter("target_heading_relax_distance_m").value)
        )
        self._target_heading_relax_blend = max(
            0.0, min(1.0, float(self.get_parameter("target_heading_relax_blend").value))
        )
        self._disallow_initial_reverse = bool(self.get_parameter("disallow_initial_reverse").value)
        self._allow_initial_reverse_when_goal_behind_rad = math.radians(
            max(0.0, float(self.get_parameter("allow_initial_reverse_when_goal_behind_deg").value))
        )
        self._allow_initial_reverse_when_obstacle_ahead = bool(
            self.get_parameter("allow_initial_reverse_when_obstacle_ahead").value
        )
        self._allow_initial_reverse_obstacle_ahead_radius_m = max(
            0.2, float(self.get_parameter("allow_initial_reverse_obstacle_ahead_radius_m").value)
        )
        self._local_takeover_enabled = bool(self.get_parameter("local_takeover_enabled").value)
        self._local_takeover_rate_hz = max(
            1.0, float(self.get_parameter("local_takeover_rate_hz").value)
        )
        self._local_takeover_trigger_obstacle_ahead_radius = max(
            0.5, float(self.get_parameter("local_takeover_trigger_obstacle_ahead_radius_m").value)
        )
        self._local_takeover_path_proximity_m = max(
            0.2, float(self.get_parameter("local_takeover_path_proximity_m").value)
        )
        self._local_takeover_min_drift_without_collision_hint_m = max(
            0.0,
            float(
                self.get_parameter("local_takeover_min_drift_without_collision_hint_m").value
            ),
        )
        self._local_takeover_horizon_m = max(
            0.5, float(self.get_parameter("local_takeover_horizon_m").value)
        )
        self._local_takeover_min_horizon_m = max(
            0.3, float(self.get_parameter("local_takeover_min_horizon_m").value)
        )
        self._local_takeover_anchor_advance_points = max(
            2, int(self.get_parameter("local_takeover_anchor_advance_points").value)
        )
        self._local_takeover_forward_only = bool(
            self.get_parameter("local_takeover_forward_only").value
        )
        self._local_takeover_start_from_front_m = max(
            0.0, float(self.get_parameter("local_takeover_start_from_front_m").value)
        )
        self._local_takeover_min_forward_progress_m = max(
            0.0, float(self.get_parameter("local_takeover_min_forward_progress_m").value)
        )
        self._local_takeover_retry_backoff_sec = max(
            0.1, float(self.get_parameter("local_takeover_retry_backoff_sec").value)
        )
        self._local_takeover_suppressed_backoff_sec = max(
            0.1, float(self.get_parameter("local_takeover_suppressed_backoff_sec").value)
        )
        self._local_takeover_microplan_timeout_sec = max(
            0.5, float(self.get_parameter("local_takeover_microplan_timeout_sec").value)
        )
        self._local_takeover_microplan_iterations_ratio = max(
            0.1, min(1.0, float(self.get_parameter("local_takeover_microplan_iterations_ratio").value))
        )
        self._local_takeover_bridge_min_poses = max(
            3, int(self.get_parameter("local_takeover_bridge_min_poses").value)
        )
        self._local_takeover_bridge_min_point_spacing_m = max(
            0.05, float(self.get_parameter("local_takeover_bridge_min_point_spacing_m").value)
        )
        self._local_takeover_post_jump_block_sec = max(
            0.0, float(self.get_parameter("local_takeover_post_jump_block_sec").value)
        )
        self._local_takeover_keepalive_sec = max(
            1.0, float(self.get_parameter("local_takeover_keepalive_sec").value)
        )
        self._local_takeover_fast_fallback_window_sec = max(
            0.2, float(self.get_parameter("local_takeover_fast_fallback_window_sec").value)
        )
        self._local_takeover_bridge_min_poses = max(
            3, int(self.get_parameter("local_takeover_bridge_min_poses").value)
        )
        self._local_takeover_bridge_min_point_spacing_m = max(
            0.05, float(self.get_parameter("local_takeover_bridge_min_point_spacing_m").value)
        )
        self._local_takeover_collision_substring = str(
            self.get_parameter("local_takeover_collision_substring").value
        )
        self._local_takeover_collision_logger_name = str(
            self.get_parameter("local_takeover_collision_logger_name").value
        )
        self._local_takeover_controller_stall_substring = str(
            self.get_parameter("local_takeover_controller_stall_substring").value
        )
        self._local_takeover_collision_hint_sec = max(
            0.2, float(self.get_parameter("local_takeover_collision_hint_sec").value)
        )
        self._initial_forward_cone_rad = math.radians(
            float(self.get_parameter("initial_forward_cone_deg").value)
        )
        self._localization_jump_guard_enabled = bool(
            self.get_parameter("localization_jump_guard_enabled").value
        )
        self._localization_jump_amcl_topic = str(
            self.get_parameter("localization_jump_amcl_topic").value
        )
        self._localization_jump_dist_m = max(
            0.05, float(self.get_parameter("localization_jump_dist_m").value)
        )
        self._localization_jump_yaw_rad = math.radians(
            max(1.0, float(self.get_parameter("localization_jump_yaw_deg").value))
        )
        self._localization_jump_guard_sec = max(
            0.1, float(self.get_parameter("localization_jump_guard_sec").value)
        )
        self._localization_jump_emergency_stop_enabled = bool(
            self.get_parameter("localization_jump_emergency_stop_enabled").value
        )
        self._localization_jump_emergency_stop_topic = str(
            self.get_parameter("localization_jump_emergency_stop_topic").value
        )
        self._localization_jump_burst_window_sec = max(
            0.2, float(self.get_parameter("localization_jump_burst_window_sec").value)
        )
        self._localization_jump_burst_count_threshold = max(
            2, int(self.get_parameter("localization_jump_burst_count_threshold").value)
        )
        self._localization_jump_burst_guard_sec = max(
            self._localization_jump_guard_sec,
            float(self.get_parameter("localization_jump_burst_guard_sec").value),
        )
        self._localization_reseed_on_deathloop_enabled = bool(
            self.get_parameter("localization_reseed_on_deathloop_enabled").value
        )
        self._localization_reseed_plan_zero_threshold = max(
            1, int(self.get_parameter("localization_reseed_plan_zero_threshold").value)
        )
        self._localization_reseed_plan_zero_window_sec = max(
            0.2, float(self.get_parameter("localization_reseed_plan_zero_window_sec").value)
        )
        self._localization_reseed_cooldown_sec = max(
            1.0, float(self.get_parameter("localization_reseed_cooldown_sec").value)
        )
        self._localization_reseed_initialpose_topic = str(
            self.get_parameter("localization_reseed_initialpose_topic").value
        )
        self._localization_reseed_use_startup_yaw = bool(
            self.get_parameter("localization_reseed_use_startup_yaw").value
        )
        self._localization_reseed_require_stable_sec = max(
            0.0, float(self.get_parameter("localization_reseed_require_stable_sec").value)
        )
        self._action_cb_group = ReentrantCallbackGroup()
        self._planner_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="hybrid_plan")
        self._planning_lock = threading.Lock()
        self._plan_failure_count = 0
        self._last_valid_path_msg: Optional[Path] = None
        self._last_valid_plan_cost: float = 0.0
        self._last_valid_plan_walltime: float = 0.0
        self._last_valid_goal: Optional[Pose2D] = None
        self._last_global_path_msg: Optional[Path] = None
        self._last_global_path_walltime: float = 0.0
        self._local_takeover_active: bool = False
        self._last_local_takeover_publish_walltime: float = 0.0
        self._last_local_takeover_log_walltime: float = 0.0
        self._last_local_takeover_idle_log_walltime: float = 0.0
        self._local_takeover_collision_hint_until: float = 0.0
        self._local_takeover_retry_not_before: float = 0.0
        self._action_planning_active: bool = False
        self._localization_jump_until: float = 0.0
        self._last_amcl_pose2d: Optional[Pose2D] = None
        self._last_plan_zero_window_start: float = 0.0
        self._last_plan_zero_hits: int = 0
        self._last_localization_reseed_walltime: float = 0.0
        self._last_localization_jump_walltime: float = 0.0
        self._localization_jump_events_walltime: list[float] = []
        self._startup_map_yaw: Optional[float] = None
        self._rejected_path_memory: list[tuple[float, list[Pose2D]]] = []
        self._rejected_obstacle_memory: list[tuple[float, list[tuple[float, float]]]] = []
        self._wall_points: list[tuple[float, float]] = []
        self._wall_memory_lock = threading.Lock()
        self._last_wall_capture_walltime: float = 0.0
        self._live_obstacle_points: list[tuple[float, float]] = []
        self._live_obstacle_lock = threading.Lock()
        self._live_obstacle_stamp_walltime: float = 0.0

        self._map: Optional[OccupancyGrid] = None
        self._map_wrapper: Optional[OccupancyGridMap] = None

        # Map servers typically publish with transient_local durability; match it to receive latched maps.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_sub = self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self._path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self._expansion_pub = self.create_publisher(
            MarkerArray, self.get_parameter("expansions_topic").value, 10
        )
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self._localization_reseed_initialpose_topic, 10
        )
        self._localization_jump_stop_pub = self.create_publisher(
            Bool, self._localization_jump_emergency_stop_topic, 10
        )

        # Replan request / progress topics
        self._request_replan_topic = str(self.get_parameter("request_replan_topic").value)
        self._replan_in_progress_topic = str(self.get_parameter("replan_in_progress_topic").value)
        self._replan_quick_timeout_sec = float(self.get_parameter("replan_quick_timeout_sec").value)
        self._replan_rate_limit_s = float(self.get_parameter("replan_rate_limit_s").value)
        self._last_replan_request_time = 0.0
        self._replan_in_progress_pub = self.create_publisher(Bool, self._replan_in_progress_topic, 1)
        self.create_subscription(Bool, self._request_replan_topic, self._on_replan_request, 10)

        # Use a larger TF cache to tolerate small timing skews between
        # simulated /clock, odometry, and sensor message stamps.
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Log, "/rosout", self._on_rosout, 100)
        if self._localization_jump_guard_enabled:
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._localization_jump_amcl_topic,
                self._on_amcl_pose,
                20,
            )
        if self._live_obstacle_cloud_enabled:
            self.create_subscription(
                PointCloud2, self._live_obstacle_cloud_topic, self._on_live_obstacle_cloud, 10
            )

        self._as_to_pose = ActionServer(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
            execute_callback=self._execute_to_pose,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._action_cb_group,
        )

        self._as_through_poses = ActionServer(
            self,
            ComputePathThroughPoses,
            "compute_path_through_poses",
            execute_callback=self._execute_through_poses,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._action_cb_group,
        )

        self.get_logger().info(
            "Nav2HybridAStarServer ready. Actions: 'compute_path_to_pose', 'compute_path_through_poses'."
        )
        self._load_wall_memory()
        if self._local_takeover_enabled:
            self.create_timer(1.0 / self._local_takeover_rate_hz, self._local_takeover_tick)
            self.get_logger().info(
                "local_takeover armed "
                f"(rate={self._local_takeover_rate_hz:.2f}Hz, "
                f"trigger_radius={self._local_takeover_trigger_obstacle_ahead_radius:.2f}m, "
                f"path_proximity={self._local_takeover_path_proximity_m:.2f}m)"
            )
        self.create_timer(0.1, self._publish_localization_jump_stop_state)

    def _publish_localization_jump_stop_state(self) -> None:
        if not self._localization_jump_emergency_stop_enabled:
            return
        msg = Bool()
        msg.data = time.perf_counter() < self._localization_jump_until
        self._localization_jump_stop_pub.publish(msg)

    def _distance_to_path(self, path_msg: Path, x: float, y: float) -> float:
        if not path_msg.poses:
            return float("inf")
        best = float("inf")
        for ps in path_msg.poses:
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            d = math.hypot(dx, dy)
            if d < best:
                best = d
        return best

    def _nearest_path_index(self, path_msg: Path, x: float, y: float) -> int:
        best_i = 0
        best_d2 = float("inf")
        for i, ps in enumerate(path_msg.poses):
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def _on_replan_request(self, msg: Bool) -> None:
        try:
            if not msg.data:
                return
        except Exception:
            # non-bool payloads are ignored
            return
        now = time.perf_counter()
        if (now - self._last_replan_request_time) < self._replan_rate_limit_s:
            return
        self._last_replan_request_time = now
        # Fire off a quick replan attempt in a background thread to avoid blocking subscriptions
        def _bg():
            self._publish_replan_in_progress(True)
            try:
                self._attempt_quick_replan()
            finally:
                self._publish_replan_in_progress(False)

        try:
            threading.Thread(target=_bg, name="quick_replan", daemon=True).start()
        except Exception as ex:
            self.get_logger().error(f"failed to start quick replan thread: {ex}")

    def _publish_replan_in_progress(self, val: bool) -> None:
        try:
            msg = Bool()
            msg.data = val
            self._replan_in_progress_pub.publish(msg)
            # Small console logs to indicate replan takeover and release
            if val:
                self.get_logger().info(
                    "Replan started: planner has taken over (nav_replan_in_progress=True)"
                )
            else:
                self.get_logger().info(
                    "Replan finished: planner has given control back (nav_replan_in_progress=False)"
                )
        except Exception:
            pass

    def _attempt_quick_replan(self) -> None:
        """Attempt a short, focused replan to the current goal using a reduced timeout.

        This function is best-effort: it will publish a new global path if a viable
        plan is found. It respects the same live-obstacle checks as the main action
        server but uses a smaller timeout and is rate-limited by the caller.
        """
        # Require a valid map and a recent global goal
        if self._map is None or self._map_wrapper is None:
            return
        if self._last_valid_goal is None:
            return
        # Obtain robot pose
        start_ps = self._get_robot_pose()
        if start_ps is None:
            return
        start_raw = _pose_to_pose2d(start_ps)
        target_raw = self._last_valid_goal
        # Build planner and map snapshot
        base_map_data = list(self._map.data)
        self._apply_static_inflation_to_data(
            base_map_data, inflation_radius_m=self._planning_grid_static_inflation_radius_m
        )
        self._apply_wall_memory_to_data(base_map_data)
        # Apply live obstacles with the configured inflation
        map_data = list(base_map_data)
        self._apply_live_obstacles_to_data(
            map_data, inflation_radius_m=self._live_obstacle_inflation_radius_m, start_pose=start_raw
        )
        start, target = self._adapt_start_and_target(start_raw, target_raw, map_data)
        planner = HybridAStarPlanner(self._map_wrapper.info, map_data, **self._planner_cfg)
        # Try to acquire planning lock briefly
        acquired = self._planning_lock.acquire(timeout=min(0.1, self._replan_quick_timeout_sec))
        if not acquired:
            return
        try:
            future = self._planner_pool.submit(
                planner.plan, start, target, max_iterations=self._planner_max_iterations
            )
            try:
                plan = future.result(timeout=self._replan_quick_timeout_sec)
            except FutureTimeoutError:
                future.cancel()
                return
        finally:
            self._planning_lock.release()
        if not plan or not plan.path:
            return
        # Validate plan against live obstacles and grid segments
        live_ok = self._path_respects_live_clearance(plan.path, start_pose=start_raw)
        seg_ok = self._path_segments_clear_in_grid(plan.path, map_data, start_pose=start_raw)
        if not (live_ok and seg_ok):
            return
        # Publish the new path as the planned path so downstream components pick it up
        path_msg = self._plan_to_path_msg(plan.path)
        self._path_pub.publish(path_msg)

    def _nearest_ahead_path_index(self, path_msg: Path, start: Pose2D) -> int:
        # Prefer points that are in front of the robot heading to keep
        # local takeover paths forward-drivable.
        best_i = self._nearest_path_index(path_msg, start.x, start.y)
        best_d2 = float("inf")
        hx = math.cos(start.yaw)
        hy = math.sin(start.yaw)
        found_ahead = False
        for i, ps in enumerate(path_msg.poses):
            dx = ps.pose.position.x - start.x
            dy = ps.pose.position.y - start.y
            if (dx * hx + dy * hy) < 0.0:
                continue
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
                found_ahead = True
        if found_ahead:
            return best_i
        return self._nearest_path_index(path_msg, start.x, start.y)

    def _anchor_pose_on_global_path(self, path_msg: Path, start_idx: int, horizon_m: float) -> Pose2D:
        poses = path_msg.poses
        if not poses:
            return Pose2D(0.0, 0.0, 0.0)
        i = min(max(0, start_idx + self._local_takeover_anchor_advance_points), len(poses) - 1)
        acc = 0.0
        while i < len(poses) - 1 and acc < horizon_m:
            p0 = poses[i].pose.position
            p1 = poses[i + 1].pose.position
            acc += math.hypot(p1.x - p0.x, p1.y - p0.y)
            i += 1
        target = poses[i]
        return _pose_to_pose2d(target)

    def _build_bridge_path_from_global(
        self,
        start: Pose2D,
        path_msg: Path,
        nearest_i: int,
        horizon_m: float,
        min_poses: Optional[int] = None,
        min_point_spacing_m: Optional[float] = None,
    ) -> Optional[Path]:
        if not path_msg.poses:
            return None
        req_min_poses = (
            self._local_takeover_bridge_min_poses
            if min_poses is None
            else max(2, int(min_poses))
        )
        req_min_spacing = (
            self._local_takeover_bridge_min_point_spacing_m
            if min_point_spacing_m is None
            else max(0.01, float(min_point_spacing_m))
        )
        out = Path()
        out.header.frame_id = self._global_frame
        out.header.stamp = self.get_clock().now().to_msg()

        start_ps = PoseStamped()
        start_ps.header = out.header
        start_ps.pose.position.x = start.x
        start_ps.pose.position.y = start.y
        start_ps.pose.position.z = 0.0
        half = 0.5 * start.yaw
        start_ps.pose.orientation.z = math.sin(half)
        start_ps.pose.orientation.w = math.cos(half)
        out.poses.append(start_ps)

        i = max(0, min(nearest_i, len(path_msg.poses) - 1))
        acc = 0.0
        while i < len(path_msg.poses):
            p = path_msg.poses[i]
            last = out.poses[-1].pose.position
            seg = math.hypot(p.pose.position.x - last.x, p.pose.position.y - last.y)
            if seg >= req_min_spacing:
                p_out = PoseStamped()
                p_out.header = out.header
                p_out.pose = p.pose
                out.poses.append(p_out)
                acc += seg
                if acc >= horizon_m:
                    break
            i += 1
        if len(out.poses) < req_min_poses:
            return None
        return out

    def _path_has_min_forward_progress(
        self, start: Pose2D, path: list[Pose2D], min_progress_m: float
    ) -> bool:
        if len(path) < 2:
            return False
        first = path[1]
        hx = math.cos(start.yaw)
        hy = math.sin(start.yaw)
        dx = first.x - start.x
        dy = first.y - start.y
        forward_progress = dx * hx + dy * hy
        return forward_progress >= min_progress_m

    def _path_msg_has_min_forward_progress(
        self, start: Pose2D, path_msg: Path, min_progress_m: float
    ) -> bool:
        if len(path_msg.poses) < 2:
            return False
        p1 = path_msg.poses[1].pose.position
        hx = math.cos(start.yaw)
        hy = math.sin(start.yaw)
        dx = p1.x - start.x
        dy = p1.y - start.y
        forward_progress = dx * hx + dy * hy
        return forward_progress >= min_progress_m

    def _local_takeover_tick(self) -> None:
        if not self._local_takeover_enabled:
            return
        if self._last_global_path_msg is None or self._map_wrapper is None:
            now = time.perf_counter()
            if (now - self._last_local_takeover_idle_log_walltime) > 2.0:
                self.get_logger().info("local_takeover idle: waiting for first global path")
                self._last_local_takeover_idle_log_walltime = now
            return
        now = time.perf_counter()
        if now < self._localization_jump_until:
            if self._local_takeover_active:
                self._local_takeover_active = False
                self._publish_local_takeover_marker(None)
            self._local_takeover_retry_not_before = now + 0.2
            return
        if now < self._local_takeover_retry_not_before:
            return
        if self._action_planning_active:
            if (now - self._last_local_takeover_log_walltime) > 1.0:
                self.get_logger().info("local_takeover waiting: global planning request in progress")
                self._last_local_takeover_log_walltime = now
            return
        robot_ps = self._get_robot_pose()
        if robot_ps is None:
            return
        start = _pose_to_pose2d(robot_ps)
        plan_start = start
        # Plan from a point in front of the robot to encourage immediately
        # drivable forward arcs for Ackermann steering.
        if self._local_takeover_start_from_front_m > 0.0 and self._map is not None:
            fx = start.x + self._local_takeover_start_from_front_m * math.cos(start.yaw)
            fy = start.y + self._local_takeover_start_from_front_m * math.sin(start.yaw)
            if self._is_world_free(list(self._map.data), fx, fy):
                plan_start = Pose2D(fx, fy, start.yaw)
        dist_to_global = self._distance_to_path(self._last_global_path_msg, start.x, start.y)
        obstacle_from_cloud = self._has_live_obstacle_ahead(
            start, radius_m=self._local_takeover_trigger_obstacle_ahead_radius
        )
        obstacle_from_collision_hint = time.perf_counter() <= self._local_takeover_collision_hint_until
        obstacle_ahead = obstacle_from_cloud or obstacle_from_collision_hint
        drift_ok_for_cloud_takeover = (
            dist_to_global >= self._local_takeover_min_drift_without_collision_hint_m
        )
        takeover_triggered = obstacle_from_collision_hint or (
            obstacle_from_cloud and drift_ok_for_cloud_takeover
        )
        post_jump_block_active = (
            (now - self._last_localization_jump_walltime) < self._local_takeover_post_jump_block_sec
        )
        if post_jump_block_active and (not obstacle_from_collision_hint):
            if (now - self._last_local_takeover_log_walltime) > 1.0:
                self.get_logger().info(
                    "local_takeover suppressed: post-jump stabilization window "
                    f"(age={now - self._last_localization_jump_walltime:.2f}s, "
                    f"block={self._local_takeover_post_jump_block_sec:.2f}s, "
                    f"dist_to_global={dist_to_global:.2f}m)"
                )
                self._last_local_takeover_log_walltime = now
            self._local_takeover_retry_not_before = now + self._local_takeover_suppressed_backoff_sec
            return

        if (not obstacle_ahead) or (dist_to_global > self._local_takeover_path_proximity_m):
            if self._local_takeover_active:
                # Hand control back to global path.
                self._path_pub.publish(self._last_global_path_msg)
                self._publish_local_takeover_marker(None)
                self._local_takeover_active = False
                self.get_logger().info(
                    "local_takeover deactivated: restoring global path "
                    f"(obstacle_ahead={obstacle_ahead}, dist_to_global={dist_to_global:.2f}m, "
                    f"cloud={obstacle_from_cloud}, collision_hint={obstacle_from_collision_hint})"
                )
            return

        if not takeover_triggered:
            now = time.perf_counter()
            if (now - self._last_local_takeover_log_walltime) > 1.0:
                self.get_logger().info(
                    "local_takeover suppressed: cloud obstacle but drift too small "
                    f"(dist_to_global={dist_to_global:.2f}m, min_drift={self._local_takeover_min_drift_without_collision_hint_m:.2f}m)"
                )
                self._last_local_takeover_log_walltime = now
            # Back off checks when repeatedly suppressed to reduce TF/query load.
            self._local_takeover_retry_not_before = now + self._local_takeover_suppressed_backoff_sec
            return

        if not self._local_takeover_active:
            self.get_logger().info(
                "local_takeover activated "
                f"(dist_to_global={dist_to_global:.2f}m, obstacle_radius={self._local_takeover_trigger_obstacle_ahead_radius:.2f}m, "
                f"cloud={obstacle_from_cloud}, collision_hint={obstacle_from_collision_hint})"
            )

        nearest_i = self._nearest_ahead_path_index(self._last_global_path_msg, plan_start)
        target = self._anchor_pose_on_global_path(
            self._last_global_path_msg, nearest_i, horizon_m=self._local_takeover_horizon_m
        )
        # Guard against too-short target pullahead.
        if math.hypot(target.x - plan_start.x, target.y - plan_start.y) < self._local_takeover_min_horizon_m:
            target = self._anchor_pose_on_global_path(
                self._last_global_path_msg,
                nearest_i,
                horizon_m=max(self._local_takeover_horizon_m, self._local_takeover_min_horizon_m + 1.0),
            )

        base_map_data = list(self._map.data) if self._map is not None else None
        if base_map_data is None:
            return
        self._apply_static_inflation_to_data(
            base_map_data, inflation_radius_m=self._planning_grid_static_inflation_radius_m
        )
        self._apply_wall_memory_to_data(base_map_data)
        self._apply_live_obstacles_to_data(base_map_data)
        local_planner_cfg = dict(self._planner_cfg)
        if self._local_takeover_forward_only:
            local_planner_cfg["allow_reverse"] = False
        planner = HybridAStarPlanner(self._map_wrapper.info, base_map_data, **local_planner_cfg)

        if not self._planning_lock.acquire(blocking=False):
            now = time.perf_counter()
            if (now - self._last_local_takeover_log_walltime) > 1.0:
                self.get_logger().warn("local_takeover skipped: planner lock busy")
                self._last_local_takeover_log_walltime = now
            self._local_takeover_retry_not_before = now + self._local_takeover_retry_backoff_sec
            return
        try:
            try:
                future = self._planner_pool.submit(
                    planner.plan,
                    plan_start,
                    target,
                    max_iterations=max(
                        3000, int(self._planner_max_iterations * self._local_takeover_microplan_iterations_ratio)
                    ),
                )
                local_plan = future.result(timeout=min(self._planner_timeout_sec, self._local_takeover_microplan_timeout_sec))
            except Exception as ex:
                now = time.perf_counter()
                if (now - self._last_local_takeover_log_walltime) > 1.0:
                    self.get_logger().warn(
                        "local_takeover skipped: local micro-plan failed "
                        f"(err={type(ex).__name__}: {ex})"
                    )
                    self._last_local_takeover_log_walltime = now
                self._local_takeover_retry_not_before = now + self._local_takeover_retry_backoff_sec
                return
        finally:
            self._planning_lock.release()

        if local_plan is None or not local_plan.path or len(local_plan.path) < 2:
            bridge_msg = self._build_bridge_path_from_global(
                plan_start,
                self._last_global_path_msg,
                nearest_i,
                horizon_m=max(2.0, self._local_takeover_horizon_m * 0.7),
            )
            if bridge_msg is None:
                # Relax bridge quality constraints as a second chance to avoid
                # dead loops when the micro-planner repeatedly returns None.
                bridge_msg = self._build_bridge_path_from_global(
                    plan_start,
                    self._last_global_path_msg,
                    nearest_i,
                    horizon_m=max(2.5, self._local_takeover_horizon_m),
                    min_poses=3,
                    min_point_spacing_m=0.05,
                )
            if bridge_msg is not None:
                self._path_pub.publish(bridge_msg)
                self._publish_local_takeover_marker(bridge_msg)
                self._local_takeover_active = True
                now = time.perf_counter()
                self.get_logger().warn(
                    "local_takeover fallback: published bridge path from global "
                    f"(poses={len(bridge_msg.poses)}, dist_to_global={dist_to_global:.2f}m)"
                )
                self._last_local_takeover_publish_walltime = now
                return
            now = time.perf_counter()
            if (now - self._last_local_takeover_log_walltime) > 1.0:
                local_len = 0 if (local_plan is None or not local_plan.path) else len(local_plan.path)
                target_dist = math.hypot(target.x - plan_start.x, target.y - plan_start.y)
                self.get_logger().warn(
                    "local_takeover skipped: micro-plan empty/short "
                    f"(local_len={local_len}, bridge_min_poses={self._local_takeover_bridge_min_poses}, "
                    f"bridge_spacing={self._local_takeover_bridge_min_point_spacing_m:.2f}, "
                    f"dist_to_global={dist_to_global:.2f}, target_dist={target_dist:.2f})"
                )
                self._last_local_takeover_log_walltime = now
            self._local_takeover_retry_not_before = now + self._local_takeover_retry_backoff_sec
            return
        if not self._path_has_min_forward_progress(
            plan_start, local_plan.path, self._local_takeover_min_forward_progress_m
        ):
            now = time.perf_counter()
            if (now - self._last_local_takeover_log_walltime) > 1.0:
                self.get_logger().warn(
                    "local_takeover skipped: micro-plan failed forward-progress "
                    f"(min_progress={self._local_takeover_min_forward_progress_m:.2f}m)"
                )
                self._last_local_takeover_log_walltime = now
            local_plan = None

        if local_plan is None:
            bridge_msg = self._build_bridge_path_from_global(
                plan_start,
                self._last_global_path_msg,
                nearest_i,
                horizon_m=max(2.0, self._local_takeover_horizon_m * 0.7),
            )
            if bridge_msg is None:
                bridge_msg = self._build_bridge_path_from_global(
                    plan_start,
                    self._last_global_path_msg,
                    nearest_i,
                    horizon_m=max(2.5, self._local_takeover_horizon_m),
                    min_poses=3,
                    min_point_spacing_m=0.05,
                )
            if bridge_msg is not None and self._path_msg_has_min_forward_progress(
                plan_start, bridge_msg, self._local_takeover_min_forward_progress_m
            ):
                self._path_pub.publish(bridge_msg)
                self._publish_local_takeover_marker(bridge_msg)
                self._local_takeover_active = True
                now = time.perf_counter()
                self.get_logger().warn(
                    "local_takeover fallback: published bridge path from global "
                    f"(poses={len(bridge_msg.poses)}, dist_to_global={dist_to_global:.2f}m)"
                )
                self._last_local_takeover_publish_walltime = now
                self._local_takeover_retry_not_before = now + 0.2
            else:
                self._local_takeover_retry_not_before = time.perf_counter() + self._local_takeover_retry_backoff_sec
            return
        if not self._path_segments_clear_in_grid(local_plan.path, base_map_data, start_pose=start):
            now = time.perf_counter()
            if (now - self._last_local_takeover_log_walltime) > 1.0:
                self.get_logger().warn("local_takeover skipped: micro-plan failed segment clearance")
                self._last_local_takeover_log_walltime = now
            self._local_takeover_retry_not_before = now + self._local_takeover_retry_backoff_sec
            return

        path_msg = self._build_path_msg(local_plan, frame_id=self._global_frame)
        self._path_pub.publish(path_msg)
        self._publish_local_takeover_marker(path_msg)
        self._local_takeover_active = True
        now = time.perf_counter()
        if (now - self._last_local_takeover_publish_walltime) > 1.0:
            self.get_logger().info(
                "local_takeover publish "
                f"(poses={len(path_msg.poses)}, target=({target.x:.2f},{target.y:.2f}), "
                f"dist_to_global={dist_to_global:.2f}m)"
            )
        self._last_local_takeover_publish_walltime = now
        self._local_takeover_retry_not_before = now + 0.2

    def _publish_local_takeover_marker(self, path_msg: Optional[Path]) -> None:
        marker = Marker()
        marker.header.frame_id = self._global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "local_takeover_path"
        marker.id = 101
        if path_msg is None or not path_msg.poses:
            marker.action = Marker.DELETE
            self._expansion_pub.publish(MarkerArray(markers=[marker]))
            return
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.1
        marker.color.a = 0.95
        for ps in path_msg.poses:
            p = Point()
            p.x = ps.pose.position.x
            p.y = ps.pose.position.y
            p.z = 0.06
            marker.points.append(p)
        self._expansion_pub.publish(MarkerArray(markers=[marker]))

    def _abort_to_pose(self, goal_handle, reason: str) -> ComputePathToPose.Result:
        self._plan_failure_count += 1
        self.get_logger().warn(
            f"compute_path_to_pose abort: {reason} (failure_count={self._plan_failure_count})"
        )
        goal_handle.abort()
        return ComputePathToPose.Result()

    def _goal_diag(self, start: Pose2D, target: Pose2D) -> str:
        dx = target.x - start.x
        dy = target.y - start.y
        dist = math.hypot(dx, dy)
        return (
            f"start=({start.x:.2f},{start.y:.2f},{start.yaw:.2f}) "
            f"goal=({target.x:.2f},{target.y:.2f},{target.yaw:.2f}) "
            f"euclid_dist={dist:.2f}"
        )

    def _try_fallback_to_recent_path(
        self, goal_handle, start: Pose2D, target: Pose2D, reason: str
    ) -> Optional[ComputePathToPose.Result]:
        now = time.perf_counter()
        in_jump_guard = now < self._localization_jump_until
        if self._last_valid_path_msg is not None:
            age = now - self._last_valid_plan_walltime
            if age <= self._plan_fallback_ttl_sec:
                path_msg = self._last_valid_path_msg
                self._path_pub.publish(path_msg)
                self.get_logger().warn(
                    "compute_path_to_pose fallback: reusing recent path "
                    f"(age={age:.2f}s, poses={len(path_msg.poses)}, prev_cost={self._last_valid_plan_cost:.3f}) "
                    f"after {reason}; {self._goal_diag(start, target)}"
                )
                goal_handle.succeed()
                result = ComputePathToPose.Result()
                result.path = path_msg
                result.planning_time = Duration(seconds=0.0).to_msg()
                return result

        # Soft fallback: if robot is still operating on a known global path (or
        # local takeover is actively publishing micro-paths), avoid hard goal
        # aborts and keep navigation running.
        if self._last_global_path_msg is not None and len(self._last_global_path_msg.poses) >= 2:
            global_age = now - self._last_global_path_walltime
            keepalive_by_tracking = False
            keepalive_by_takeover = False
            dist_to_global = float("inf")
            robot_ps = self._get_robot_pose()
            if robot_ps is not None:
                rx = robot_ps.pose.position.x
                ry = robot_ps.pose.position.y
                d = self._distance_to_path(self._last_global_path_msg, rx, ry)
                dist_to_global = d
                keepalive_by_tracking = d <= self._plan_fallback_keepalive_dist_to_global_m
            if self._local_takeover_active and self._last_local_takeover_publish_walltime > 0.0:
                since_takeover_pub = now - self._last_local_takeover_publish_walltime
                keepalive_by_takeover = since_takeover_pub <= self._local_takeover_keepalive_sec
            allow_by_age = (not in_jump_guard) and (
                global_age <= self._plan_fallback_max_global_age_sec
                and dist_to_global <= self._plan_fallback_max_dist_for_age_m
            )
            allow_by_tracking = keepalive_by_tracking or keepalive_by_takeover
            if allow_by_age or allow_by_tracking:
                path_msg = self._last_global_path_msg
                self._path_pub.publish(path_msg)
                if allow_by_tracking:
                    # Robot is still near the current global path; refresh
                    # freshness so transient planner timeouts do not abort goal.
                    self._last_global_path_walltime = now
                self.get_logger().warn(
                    "compute_path_to_pose fallback: reusing global path to prevent abort "
                    f"(global_age={global_age:.2f}s, poses={len(path_msg.poses)}, "
                    f"dist_to_global={dist_to_global if math.isfinite(dist_to_global) else float('nan'):.2f}m, "
                    f"local_takeover_active={self._local_takeover_active}, "
                    f"keepalive_by_tracking={keepalive_by_tracking}, "
                    f"keepalive_by_takeover={keepalive_by_takeover}, "
                    f"in_jump_guard={in_jump_guard}, "
                    f"allow_by_age={allow_by_age}) "
                    f"after {reason}; {self._goal_diag(start, target)}"
                )
                goal_handle.succeed()
                result = ComputePathToPose.Result()
                result.path = path_msg
                result.planning_time = Duration(seconds=0.0).to_msg()
                return result
        return None

    def _abort_through_poses(self, goal_handle, reason: str) -> ComputePathThroughPoses.Result:
        self._plan_failure_count += 1
        self.get_logger().warn(
            f"compute_path_through_poses abort: {reason} (failure_count={self._plan_failure_count})"
        )
        goal_handle.abort()
        return ComputePathThroughPoses.Result()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        info = msg.info
        grid_info = GridInfo(
            resolution=info.resolution,
            origin_xy=(info.origin.position.x, info.origin.position.y),
            width=info.width,
            height=info.height,
        )
        self._map_wrapper = OccupancyGridMap(grid_info, list(msg.data))

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._localization_jump_guard_enabled:
            return
        cur = Pose2D(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=_yaw_from_quaternion(msg.pose.pose.orientation),
        )
        if self._last_amcl_pose2d is not None:
            dxy = math.hypot(cur.x - self._last_amcl_pose2d.x, cur.y - self._last_amcl_pose2d.y)
            dyaw = abs(_norm_angle(cur.yaw - self._last_amcl_pose2d.yaw))
            if dxy >= self._localization_jump_dist_m or dyaw >= self._localization_jump_yaw_rad:
                self._register_localization_jump_event(
                    dxy=dxy,
                    dyaw_rad=dyaw,
                    source="amcl_pose",
                )
        self._last_amcl_pose2d = cur

    def _register_localization_jump_event(self, dxy: float, dyaw_rad: float, source: str) -> None:
        if not self._localization_jump_guard_enabled:
            return
        now = time.perf_counter()
        self._last_localization_jump_walltime = now
        self._localization_jump_events_walltime = [
            t for t in self._localization_jump_events_walltime
            if (now - t) <= self._localization_jump_burst_window_sec
        ]
        self._localization_jump_events_walltime.append(now)
        burst_count = len(self._localization_jump_events_walltime)
        burst_active = burst_count >= self._localization_jump_burst_count_threshold
        guard_sec = (
            self._localization_jump_burst_guard_sec
            if burst_active
            else self._localization_jump_guard_sec
        )
        self._localization_jump_until = max(self._localization_jump_until, now + guard_sec)
        self.get_logger().warn(
            "localization_jump_guard armed "
            f"(source={source}, dxy={dxy:.3f}m, dyaw_deg={math.degrees(dyaw_rad):.1f}, "
            f"guard={guard_sec:.1f}s, burst_count={burst_count}/{self._localization_jump_burst_count_threshold})"
        )

    def _try_localization_reseed(self, reason: str) -> bool:
        if not self._localization_reseed_on_deathloop_enabled:
            return False
        now = time.perf_counter()
        if (now - self._last_localization_reseed_walltime) < self._localization_reseed_cooldown_sec:
            return False
        if (now - self._last_localization_jump_walltime) < self._localization_reseed_require_stable_sec:
            return False
        robot_ps = self._get_robot_pose()
        if robot_ps is None:
            return False
        start = _pose_to_pose2d(robot_ps)
        if self._startup_map_yaw is None:
            self._startup_map_yaw = start.yaw
        yaw = self._startup_map_yaw if self._localization_reseed_use_startup_yaw else start.yaw
        msg = PoseWithCovarianceStamped()
        # Stamp=0 requests "latest transform" and avoids future extrapolation warnings.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = self._global_frame
        msg.pose.pose.position.x = start.x
        msg.pose.pose.position.y = start.y
        msg.pose.pose.position.z = 0.0
        half = 0.5 * yaw
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        # Moderate uncertainty so AMCL can quickly converge without teleporting.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.40
        self._initialpose_pub.publish(msg)
        self._last_localization_reseed_walltime = now
        self._localization_jump_until = now + self._localization_jump_guard_sec
        self._last_plan_zero_window_start = 0.0
        self._last_plan_zero_hits = 0
        self.get_logger().warn(
            "localization_reseed published "
            f"(reason={reason}, x={start.x:.2f}, y={start.y:.2f}, yaw={yaw:.2f})"
        )
        return True

    def _on_rosout(self, msg: Log) -> None:
        if int(msg.level) < 30:
            return

        now = time.perf_counter()
        if self._localization_jump_guard_enabled and ("tf_event_logger_node" in msg.name):
            tf_jump = "TF_JUMP map->odom" in msg.msg
            amcl_jump = "AMCL_JUMP" in msg.msg
            if tf_jump or amcl_jump:
                dxy_match = re.search(r"dxy=([0-9]+(?:\.[0-9]+)?)m", msg.msg)
                dyaw_match = re.search(r"dyaw_deg=([0-9]+(?:\.[0-9]+)?)", msg.msg)
                if dxy_match and dyaw_match:
                    try:
                        dxy = float(dxy_match.group(1))
                        dyaw_rad = math.radians(float(dyaw_match.group(1)))
                    except ValueError:
                        dxy = -1.0
                        dyaw_rad = -1.0
                    if dxy >= 0.0 and dyaw_rad >= 0.0:
                        self._register_localization_jump_event(
                            dxy=dxy,
                            dyaw_rad=dyaw_rad,
                            source="tf_event_logger",
                        )
        if self._local_takeover_enabled and (self._local_takeover_collision_logger_name in msg.name):
            collision_msg = self._local_takeover_collision_substring in msg.msg
            stall_msg = self._local_takeover_controller_stall_substring in msg.msg
            if collision_msg or stall_msg:
                if stall_msg and self._localization_reseed_on_deathloop_enabled:
                    if (
                        self._last_plan_zero_window_start == 0.0
                        or (now - self._last_plan_zero_window_start)
                        > self._localization_reseed_plan_zero_window_sec
                    ):
                        self._last_plan_zero_window_start = now
                        self._last_plan_zero_hits = 1
                    else:
                        self._last_plan_zero_hits += 1
                    if (
                        now < self._localization_jump_until
                        and self._last_plan_zero_hits >= self._localization_reseed_plan_zero_threshold
                    ):
                        self._try_localization_reseed("jump_guard_plan_zero_storm")
                if now < self._localization_jump_until:
                    return
                self._local_takeover_collision_hint_until = now + self._local_takeover_collision_hint_sec
                if (now - self._last_local_takeover_log_walltime) > 1.0:
                    self.get_logger().warn(
                        "local_takeover collision hint armed "
                        f"(ttl={self._local_takeover_collision_hint_sec:.1f}s, "
                        f"source={'collision_ahead' if collision_msg else 'controller_plan_zero'})"
                    )
                    self._last_local_takeover_log_walltime = now

        if not self._wall_memory_enabled:
            return
        if self._wall_memory_controller_logger_name not in msg.name:
            return
        if self._wall_memory_collision_substring not in msg.msg:
            return

        if (now - self._last_wall_capture_walltime) < self._wall_memory_capture_cooldown_sec:
            return
        self._last_wall_capture_walltime = now

        pose = self._get_robot_pose()
        if pose is None:
            return
        self._add_wall_point(pose.pose.position.x, pose.pose.position.y)

    def _on_live_obstacle_cloud(self, msg: PointCloud2) -> None:
        # Local planner publishes dense xyz float32 cloud in map frame.
        if msg.width == 0 or msg.point_step <= 0:
            return
        if msg.header.frame_id and msg.header.frame_id != self._global_frame:
            # Ignore mismatched frame to avoid corrupting planning grid.
            return
        if len(msg.data) < msg.point_step:
            return
        if len(msg.data) % msg.point_step != 0:
            return

        points: list[tuple[float, float]] = []
        x_off = 0
        y_off = 4
        if msg.point_step < 8:
            return
        for i in range(0, len(msg.data), msg.point_step):
            try:
                x = struct.unpack_from("f", msg.data, i + x_off)[0]
                y = struct.unpack_from("f", msg.data, i + y_off)[0]
            except struct.error:
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            points.append((x, y))

        with self._live_obstacle_lock:
            self._live_obstacle_points = points
            self._live_obstacle_stamp_walltime = time.perf_counter()

    def _load_wall_memory(self) -> None:
        if not self._wall_memory_enabled:
            return
        try:
            if not os.path.exists(self._wall_memory_path):
                return
            with open(self._wall_memory_path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            pts = payload.get("points", [])
            loaded: list[tuple[float, float]] = []
            for p in pts:
                x = float(p["x"])
                y = float(p["y"])
                loaded.append((x, y))
            self._wall_points = loaded[-self._wall_memory_max_points :]
            self.get_logger().info(
                f"wall_memory loaded: {len(self._wall_points)} points from {self._wall_memory_path}"
            )
        except Exception as ex:
            self.get_logger().warn(f"wall_memory load failed: {ex}")

    def _save_wall_memory(self) -> None:
        if not self._wall_memory_enabled:
            return
        try:
            folder = os.path.dirname(self._wall_memory_path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            payload = {"points": [{"x": x, "y": y} for x, y in self._wall_points]}
            with open(self._wall_memory_path, "w", encoding="utf-8") as f:
                json.dump(payload, f)
        except Exception as ex:
            self.get_logger().warn(f"wall_memory save failed: {ex}")

    def _add_wall_point(self, x: float, y: float) -> None:
        with self._wall_memory_lock:
            for px, py in self._wall_points:
                if (px - x) * (px - x) + (py - y) * (py - y) < (
                    self._wall_memory_min_separation_m * self._wall_memory_min_separation_m
                ):
                    return
            self._wall_points.append((x, y))
            if len(self._wall_points) > self._wall_memory_max_points:
                self._wall_points = self._wall_points[-self._wall_memory_max_points :]
            self._save_wall_memory()
        self.get_logger().warn(
            f"wall_memory captured point=({x:.2f},{y:.2f}) total={len(self._wall_points)}"
        )

    def _apply_wall_memory_to_data(self, data: list[int]) -> None:
        if not self._wall_memory_enabled:
            return
        if self._map_wrapper is None:
            return
        with self._wall_memory_lock:
            points = list(self._wall_points)
        if not points:
            return
        info = self._map_wrapper.info
        radius_cells = max(1, int(math.ceil(self._wall_memory_inflation_radius_m / info.resolution)))
        for wx, wy in points:
            center = self._map_wrapper.world_to_grid(wx, wy)
            for dy in range(-radius_cells, radius_cells + 1):
                iy = center.iy + dy
                if iy < 0 or iy >= info.height:
                    continue
                for dx in range(-radius_cells, radius_cells + 1):
                    ix = center.ix + dx
                    if ix < 0 or ix >= info.width:
                        continue
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    data[iy * info.width + ix] = 100

    def _apply_live_obstacles_to_data(
        self,
        data: list[int],
        inflation_radius_m: Optional[float] = None,
        start_pose: Optional[Pose2D] = None,
    ) -> None:
        if not self._live_obstacle_cloud_enabled:
            return
        if self._map_wrapper is None:
            return
        now = time.perf_counter()
        with self._live_obstacle_lock:
            age = now - self._live_obstacle_stamp_walltime
            points = list(self._live_obstacle_points)
        if age > self._live_obstacle_cloud_timeout_sec:
            return
        if not points:
            return
        info = self._map_wrapper.info
        inflation = (
            self._live_obstacle_inflation_radius_m
            if inflation_radius_m is None
            else max(0.0, inflation_radius_m)
        )
        radius_cells = max(1, int(math.ceil(inflation / info.resolution)))
        for wx, wy in points:
            if start_pose is not None and self._live_obstacle_ignore_near_start_m > 0.0:
                if math.hypot(wx - start_pose.x, wy - start_pose.y) < self._live_obstacle_ignore_near_start_m:
                    continue
            center = self._map_wrapper.world_to_grid(wx, wy)
            for dy in range(-radius_cells, radius_cells + 1):
                iy = center.iy + dy
                if iy < 0 or iy >= info.height:
                    continue
                for dx in range(-radius_cells, radius_cells + 1):
                    ix = center.ix + dx
                    if ix < 0 or ix >= info.width:
                        continue
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    data[iy * info.width + ix] = 100

    def _apply_static_inflation_to_data(self, data: list[int], inflation_radius_m: float) -> None:
        if self._map_wrapper is None:
            return
        if inflation_radius_m <= 0.0:
            return
        info = self._map_wrapper.info
        radius_cells = max(1, int(math.ceil(inflation_radius_m / info.resolution)))
        original = list(data)
        for iy in range(info.height):
            base = iy * info.width
            for ix in range(info.width):
                v = original[base + ix]
                if v < 50:
                    continue
                for dy in range(-radius_cells, radius_cells + 1):
                    ny = iy + dy
                    if ny < 0 or ny >= info.height:
                        continue
                    for dx in range(-radius_cells, radius_cells + 1):
                        nx = ix + dx
                        if nx < 0 or nx >= info.width:
                            continue
                        if dx * dx + dy * dy > radius_cells * radius_cells:
                            continue
                        data[ny * info.width + nx] = 100

    def _path_respects_live_clearance(self, path: list[Pose2D], start_pose: Optional[Pose2D] = None) -> bool:
        if self._path_live_obstacle_min_clearance <= 0.0:
            return True
        now = time.perf_counter()
        with self._live_obstacle_lock:
            age = now - self._live_obstacle_stamp_walltime
            points = list(self._live_obstacle_points)
        if age > self._live_obstacle_cloud_timeout_sec or not points:
            return True
        min_d2 = self._path_live_obstacle_min_clearance * self._path_live_obstacle_min_clearance
        for p in path:
            if start_pose is not None and self._path_clearance_start_grace_m > 0.0:
                if math.hypot(p.x - start_pose.x, p.y - start_pose.y) < self._path_clearance_start_grace_m:
                    continue
            px = p.x
            py = p.y
            for ox, oy in points:
                dx = px - ox
                dy = py - oy
                if dx * dx + dy * dy < min_d2:
                    return False
        return True

    def _min_live_clearance_on_path(self, path: list[Pose2D], start_pose: Optional[Pose2D] = None) -> Optional[float]:
        now = time.perf_counter()
        with self._live_obstacle_lock:
            age = now - self._live_obstacle_stamp_walltime
            points = list(self._live_obstacle_points)
        if age > self._live_obstacle_cloud_timeout_sec or not points:
            return None
        best = float("inf")
        for p in path:
            if start_pose is not None and self._path_clearance_start_grace_m > 0.0:
                if math.hypot(p.x - start_pose.x, p.y - start_pose.y) < self._path_clearance_start_grace_m:
                    continue
            for ox, oy in points:
                d = math.hypot(p.x - ox, p.y - oy)
                if d < best:
                    best = d
        if best == float("inf"):
            return None
        return best

    def _is_world_occupied_in_data(self, data: list[int], x: float, y: float) -> bool:
        if self._map_wrapper is None:
            return True
        idx = self._map_wrapper.world_to_grid(x, y)
        info = self._map_wrapper.info
        if idx.ix < 0 or idx.ix >= info.width or idx.iy < 0 or idx.iy >= info.height:
            return True
        v = data[idx.iy * info.width + idx.ix]
        if v < 0:
            return self._segment_treat_unknown_as_occupied
        return v >= 50

    def _path_segments_clear_in_grid(
        self, path: list[Pose2D], data: list[int], start_pose: Optional[Pose2D] = None
    ) -> bool:
        if len(path) < 2:
            return False
        clearance = self._path_segment_obstacle_clearance_m
        if clearance <= 0.0:
            return True
        step = self._path_segment_sample_step_m
        for i in range(len(path) - 1):
            p0 = path[i]
            p1 = path[i + 1]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            seg_len = math.hypot(dx, dy)
            n = max(1, int(math.ceil(seg_len / step)))
            for k in range(n + 1):
                t = k / n
                sx = p0.x + dx * t
                sy = p0.y + dy * t
                if start_pose is not None and self._path_clearance_start_grace_m > 0.0:
                    if math.hypot(sx - start_pose.x, sy - start_pose.y) < self._path_clearance_start_grace_m:
                        continue
                # Check a small ring around sample point for robust corridor clearance.
                for ang in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                    cx = sx + clearance * math.cos(ang)
                    cy = sy + clearance * math.sin(ang)
                    if self._is_world_occupied_in_data(data, cx, cy):
                        return False
                if self._is_world_occupied_in_data(data, sx, sy):
                    return False
        return True

    def _segment_grid_collision_sample(
        self, path: list[Pose2D], data: list[int], start_pose: Optional[Pose2D] = None
    ) -> Optional[tuple[float, float]]:
        if len(path) < 2:
            return None
        clearance = self._path_segment_obstacle_clearance_m
        step = self._path_segment_sample_step_m
        for i in range(len(path) - 1):
            p0 = path[i]
            p1 = path[i + 1]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            seg_len = math.hypot(dx, dy)
            n = max(1, int(math.ceil(seg_len / step)))
            for k in range(n + 1):
                t = k / n
                sx = p0.x + dx * t
                sy = p0.y + dy * t
                if start_pose is not None and self._path_clearance_start_grace_m > 0.0:
                    if math.hypot(sx - start_pose.x, sy - start_pose.y) < self._path_clearance_start_grace_m:
                        continue
                if self._is_world_occupied_in_data(data, sx, sy):
                    return (sx, sy)
                for ang in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                    cx = sx + clearance * math.cos(ang)
                    cy = sy + clearance * math.sin(ang)
                    if self._is_world_occupied_in_data(data, cx, cy):
                        return (cx, cy)
        return None

    def _block_path_corridor_in_data(self, data: list[int], path: list[Pose2D], radius_m: float) -> None:
        if self._map_wrapper is None or len(path) < 2:
            return
        info = self._map_wrapper.info
        radius_cells = max(1, int(math.ceil(max(0.0, radius_m) / info.resolution)))
        step = max(info.resolution * 0.5, self._path_segment_sample_step_m)
        for i in range(len(path) - 1):
            p0 = path[i]
            p1 = path[i + 1]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            seg_len = math.hypot(dx, dy)
            n = max(1, int(math.ceil(seg_len / step)))
            for k in range(n + 1):
                t = k / n
                sx = p0.x + dx * t
                sy = p0.y + dy * t
                center = self._map_wrapper.world_to_grid(sx, sy)
                for oy in range(-radius_cells, radius_cells + 1):
                    iy = center.iy + oy
                    if iy < 0 or iy >= info.height:
                        continue
                    for ox in range(-radius_cells, radius_cells + 1):
                        ix = center.ix + ox
                        if ix < 0 or ix >= info.width:
                            continue
                        if ox * ox + oy * oy > radius_cells * radius_cells:
                            continue
                        data[iy * info.width + ix] = 100

    def _prune_rejected_path_memory(self) -> None:
        if self._rejected_path_memory_ttl_sec <= 0.0:
            self._rejected_path_memory = []
            self._rejected_obstacle_memory = []
            return
        now = time.perf_counter()
        self._rejected_path_memory = [
            (ts, p) for (ts, p) in self._rejected_path_memory if (now - ts) <= self._rejected_path_memory_ttl_sec
        ]
        self._rejected_obstacle_memory = [
            (ts, pts)
            for (ts, pts) in self._rejected_obstacle_memory
            if (now - ts) <= self._rejected_path_memory_ttl_sec
        ]

    def _collect_live_obstacles_near_path(
        self, path: list[Pose2D], start_pose: Optional[Pose2D], near_radius_m: float
    ) -> list[tuple[float, float]]:
        now = time.perf_counter()
        with self._live_obstacle_lock:
            age = now - self._live_obstacle_stamp_walltime
            points = list(self._live_obstacle_points)
        if age > self._live_obstacle_cloud_timeout_sec or not points:
            return []
        near_r2 = near_radius_m * near_radius_m
        out: list[tuple[float, float]] = []
        for ox, oy in points:
            keep = False
            for p in path:
                if start_pose is not None and self._path_clearance_start_grace_m > 0.0:
                    if math.hypot(p.x - start_pose.x, p.y - start_pose.y) < self._path_clearance_start_grace_m:
                        continue
                dx = p.x - ox
                dy = p.y - oy
                if (dx * dx + dy * dy) <= near_r2:
                    keep = True
                    break
            if keep:
                out.append((ox, oy))
        return out

    def _block_obstacle_points_in_data(
        self, data: list[int], points: list[tuple[float, float]], radius_m: float
    ) -> None:
        if self._map_wrapper is None or not points:
            return
        info = self._map_wrapper.info
        radius_cells = max(1, int(math.ceil(max(0.0, radius_m) / info.resolution)))
        for wx, wy in points:
            center = self._map_wrapper.world_to_grid(wx, wy)
            for oy in range(-radius_cells, radius_cells + 1):
                iy = center.iy + oy
                if iy < 0 or iy >= info.height:
                    continue
                for ox in range(-radius_cells, radius_cells + 1):
                    ix = center.ix + ox
                    if ix < 0 or ix >= info.width:
                        continue
                    if ox * ox + oy * oy > radius_cells * radius_cells:
                        continue
                    data[iy * info.width + ix] = 100

    def _accept_goal(self, _goal_request) -> GoalResponse:
        if self._map_wrapper is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _is_same_goal_as_last_valid(self, target: Pose2D) -> bool:
        if self._last_valid_goal is None:
            return False
        dx = target.x - self._last_valid_goal.x
        dy = target.y - self._last_valid_goal.y
        if math.hypot(dx, dy) > self._path_hold_goal_tolerance_m:
            return False
        dyaw = _norm_angle(target.yaw - self._last_valid_goal.yaw)
        return abs(dyaw) <= self._path_hold_goal_tolerance_yaw

    def _has_live_obstacle_ahead(
        self, start: Pose2D, radius_m: Optional[float] = None
    ) -> bool:
        if not self._live_obstacle_cloud_enabled:
            return False
        now = time.perf_counter()
        with self._live_obstacle_lock:
            age = now - self._live_obstacle_stamp_walltime
            points = list(self._live_obstacle_points)
        if age > self._live_obstacle_cloud_timeout_sec:
            return False
        if not points:
            return False
        for ox, oy in points:
            dx = ox - start.x
            dy = oy - start.y
            dist = math.hypot(dx, dy)
            max_radius = (
                self._path_hold_obstacle_ahead_radius if radius_m is None else max(0.1, radius_m)
            )
            if dist <= 1e-6 or dist > max_radius:
                continue
            bearing = math.atan2(dy, dx)
            rel = _norm_angle(bearing - start.yaw)
            if abs(rel) <= self._path_hold_obstacle_ahead_half_angle:
                return True
        return False

    def _try_hold_last_path(
        self, goal_handle, start: Pose2D, target: Pose2D
    ) -> Optional[ComputePathToPose.Result]:
        if not self._path_hold_enabled:
            return None
        if self._last_valid_path_msg is None:
            return None
        if not self._is_same_goal_as_last_valid(target):
            return None
        if self._local_takeover_active or (time.perf_counter() <= self._local_takeover_collision_hint_until):
            # During imminent/active takeover we should not keep re-affirming
            # stale held global paths, otherwise local avoidance cannot take over.
            return None
        if self._has_live_obstacle_ahead(start):
            # Cloud-only "obstacle ahead" can be noisy while still tracking well.
            # If drift is small and there is no collision-hint context, keep the
            # current held path instead of triggering expensive replans.
            drift = self._distance_to_path(self._last_valid_path_msg, start.x, start.y)
            if drift >= self._path_hold_cloud_block_min_drift_m:
                return None
            self.get_logger().info(
                "compute_path_to_pose hold: cloud obstacle ignored due to low drift "
                f"(drift={drift:.2f}m, threshold={self._path_hold_cloud_block_min_drift_m:.2f}m)"
            )
        self._last_global_path_msg = self._last_valid_path_msg
        self._last_global_path_walltime = time.perf_counter()
        self.get_logger().info(
            "compute_path_to_pose hold: reusing last valid path "
            f"(poses={len(self._last_valid_path_msg.poses)})"
        )
        goal_handle.succeed()
        result = ComputePathToPose.Result()
        result.path = self._last_valid_path_msg
        result.planning_time = Duration(seconds=0.0).to_msg()
        return result

    def _is_world_free(self, data: list[int], x: float, y: float) -> bool:
        if self._map_wrapper is None:
            return False
        idx = self._map_wrapper.world_to_grid(x, y)
        info = self._map_wrapper.info
        if idx.ix < 0 or idx.ix >= info.width or idx.iy < 0 or idx.iy >= info.height:
            return False
        v = data[idx.iy * info.width + idx.ix]
        return (v >= 0) and (v < 50)

    def _adapt_start_and_target(
        self, start: Pose2D, target: Pose2D, map_data: list[int]
    ) -> tuple[Pose2D, Pose2D]:
        dx = target.x - start.x
        dy = target.y - start.y
        dist = math.hypot(dx, dy)
        if dist <= 1e-3:
            return start, target

        goal_bearing = math.atan2(dy, dx)
        # Blend the initial heading toward the goal bearing so replans are less
        # likely to "hook back" to a just-passed segment.
        adapted_start = Pose2D(
            x=start.x,
            y=start.y,
            yaw=_lerp_angle(start.yaw, goal_bearing, self._start_heading_to_goal_blend),
        )

        # 20% "look-ahead" upgrade: project planning start forward by a fraction
        # of min turning radius when free, to keep momentum through corners.
        projection = min(
            self._start_progress_projection_max_m,
            self._planner_cfg["minimum_turning_radius"] * self._start_progress_projection_ratio,
        )
        if projection > 0.0 and dist > (projection * 1.5):
            px = adapted_start.x + projection * math.cos(adapted_start.yaw)
            py = adapted_start.y + projection * math.sin(adapted_start.yaw)
            if self._is_world_free(map_data, px, py):
                adapted_start = Pose2D(x=px, y=py, yaw=adapted_start.yaw)

        adapted_target = target
        # Far from the goal, slightly relax strict final yaw to the direction of
        # travel, reducing unnecessary reversals in mid-route replans.
        if dist >= self._target_heading_relax_distance_m and self._target_heading_relax_blend > 0.0:
            adapted_target = Pose2D(
                x=target.x,
                y=target.y,
                yaw=_lerp_angle(target.yaw, goal_bearing, self._target_heading_relax_blend),
            )

        return adapted_start, adapted_target

    def _get_robot_pose(self) -> Optional[PoseStamped]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return None

        ps = PoseStamped()
        ps.header.frame_id = self._global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        if self._startup_map_yaw is None:
            self._startup_map_yaw = _yaw_from_quaternion(tf.transform.rotation)
        return ps

    def _build_path_msg(self, result: PlanResult, frame_id: str) -> Path:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        for pose in result.path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = pose.x
            ps.pose.position.y = pose.y
            ps.pose.position.z = 0.0
            half = 0.5 * pose.yaw
            ps.pose.orientation.z = math.sin(half)
            ps.pose.orientation.w = math.cos(half)
            path_msg.poses.append(ps)
        return path_msg

    def _build_expansion_markers(self, result: PlanResult, frame_id: str) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.ns = "expansions"
        clear.id = 0
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if not result.expanded:
            return markers

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "expansions"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 0.35

        from geometry_msgs.msg import Point

        for pose in result.expanded:
            p = Point()
            p.x = pose.x
            p.y = pose.y
            p.z = 0.0
            marker.points.append(p)

        markers.markers.append(marker)
        return markers

    def _execute_to_pose(self, goal_handle):
        started = time.perf_counter()
        self._action_planning_active = True
        try:
            if self._map is None or self._map_wrapper is None:
                return self._abort_to_pose(goal_handle, "map_not_ready")

            goal: PoseStamped = goal_handle.request.goal
            if goal_handle.request.use_start:
                start_ps = goal_handle.request.start
            else:
                start_ps = self._get_robot_pose()
                if start_ps is None:
                    return self._abort_to_pose(goal_handle, "tf_robot_pose_unavailable")

            start_raw = _pose_to_pose2d(start_ps)
            target_raw = _pose_to_pose2d(goal)
            held = self._try_hold_last_path(goal_handle, start_raw, target_raw)
            if held is not None:
                return held
            now = time.perf_counter()
            if now < self._localization_jump_until:
                fallback = self._try_fallback_to_recent_path(
                    goal_handle, start_raw, target_raw, reason="localization_jump_guard"
                )
                if fallback is not None:
                    return fallback
                if self._last_global_path_msg is not None and len(self._last_global_path_msg.poses) >= 2:
                    path_msg = self._last_global_path_msg
                    self._path_pub.publish(path_msg)
                    self.get_logger().warn(
                        "compute_path_to_pose guard-hold: reusing last global path "
                        f"(poses={len(path_msg.poses)}); {self._goal_diag(start_raw, target_raw)}"
                    )
                    goal_handle.succeed()
                    result = ComputePathToPose.Result()
                    result.path = path_msg
                    result.planning_time = Duration(seconds=0.0).to_msg()
                    return result
                return self._abort_to_pose(
                    goal_handle, f"localization_jump_guard_active; {self._goal_diag(start_raw, target_raw)}"
                )
            # If local takeover is already publishing fresh bridge/micro paths,
            # avoid expensive full global replans on every planner action tick.
            if (
                self._local_takeover_active
                and self._last_local_takeover_publish_walltime > 0.0
                and (now - self._last_local_takeover_publish_walltime)
                <= self._local_takeover_fast_fallback_window_sec
            ):
                fast_fb = self._try_fallback_to_recent_path(
                    goal_handle, start_raw, target_raw, reason="local_takeover_fast_keepalive"
                )
                if fast_fb is not None:
                    return fast_fb

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ComputePathToPose.Result()

            base_map_data = list(self._map.data)
            self._apply_static_inflation_to_data(
                base_map_data, inflation_radius_m=self._planning_grid_static_inflation_radius_m
            )
            self._apply_wall_memory_to_data(base_map_data)
            start = start_raw
            target = target_raw
            plan = None
            attempts = self._path_clearance_retry_count + 1
            rejected_paths: list[list[Pose2D]] = []
            self._prune_rejected_path_memory()
            remembered_rejected_paths = [p for _, p in self._rejected_path_memory]
            remembered_rejected_obstacles = [pts for _, pts in self._rejected_obstacle_memory]
            remembered_rejected_obstacle_points = [pt for pts in remembered_rejected_obstacles for pt in pts]
            self.get_logger().info(
                "compute_path_to_pose start "
                f"attempts={attempts} start=({start_raw.x:.2f},{start_raw.y:.2f},{start_raw.yaw:.2f}) "
                f"goal=({target_raw.x:.2f},{target_raw.y:.2f},{target_raw.yaw:.2f}) "
                f"live_points={len(self._live_obstacle_points)} remembered_rejected={len(remembered_rejected_paths)} "
                f"rejected_obs_points={len(remembered_rejected_obstacle_points)} "
                f"clearance_min={self._path_live_obstacle_min_clearance:.2f} "
                f"segment_clearance={self._path_segment_obstacle_clearance_m:.2f} "
                f"unknown_segment_block={self._segment_treat_unknown_as_occupied} "
            f"unknown_planner_block={self._planner_cfg['treat_unknown_as_occupied']} "
            f"none_retry_relax={self._path_none_retry_relax_live_overlay}"
            )
            last_attempt_failed = False
            for attempt in range(attempts):
                inflation = self._live_obstacle_inflation_radius_m + (
                    self._path_clearance_retry_inflation_step * attempt
                )
                if (
                    attempt > 0
                    and last_attempt_failed
                    and self._path_none_retry_relax_live_overlay
                ):
                    inflation = max(
                        self._path_none_retry_min_inflation_m,
                        self._live_obstacle_inflation_radius_m
                        - self._path_clearance_retry_inflation_step * attempt,
                    )
                map_data = list(base_map_data)
                self._apply_live_obstacles_to_data(
                    map_data, inflation_radius_m=inflation, start_pose=start_raw
                )
                self._block_obstacle_points_in_data(
                    map_data,
                    remembered_rejected_obstacle_points,
                    radius_m=self._rejected_obstacle_block_radius_m,
                )
                start, target = self._adapt_start_and_target(start_raw, target_raw, map_data)
                planner = HybridAStarPlanner(self._map_wrapper.info, map_data, **self._planner_cfg)
                if not self._planning_lock.acquire(timeout=self._planner_timeout_sec):
                    self.get_logger().warn(
                        f"compute_path_to_pose attempt={attempt + 1}/{attempts} rejected: "
                        f"planner_busy_lock_timeout_{self._planner_timeout_sec:.2f}s"
                    )
                    plan = None
                    last_attempt_failed = True
                    continue
                try:
                    try:
                        future = self._planner_pool.submit(
                            planner.plan,
                            start,
                            target,
                            max_iterations=self._planner_max_iterations,
                        )
                        plan = future.result(timeout=self._planner_timeout_sec)
                    except FutureTimeoutError:
                        self.get_logger().warn(
                            f"compute_path_to_pose attempt={attempt + 1}/{attempts} rejected: "
                            f"planner_future_timeout_{self._planner_timeout_sec:.2f}s"
                        )
                        future.cancel()
                        plan = None
                        last_attempt_failed = True
                        continue
                finally:
                    self._planning_lock.release()
                if plan is None:
                    last_attempt_failed = True
                    self.get_logger().warn(
                        f"compute_path_to_pose attempt={attempt + 1}/{attempts} rejected: planner_returned_none "
                        f"inflation={inflation:.2f}m remembered_rejected={len(remembered_rejected_paths)} "
                        f"new_rejected={len(rejected_paths)}"
                    )
                    continue
                last_attempt_failed = False
                if not plan.path:
                    self.get_logger().warn(
                        f"compute_path_to_pose attempt={attempt + 1}/{attempts} rejected: planner_returned_empty_path "
                        f"inflation={inflation:.2f}m"
                    )
                    last_attempt_failed = True
                    continue
                live_ok = self._path_respects_live_clearance(plan.path, start_pose=start_raw)
                seg_ok = self._path_segments_clear_in_grid(plan.path, map_data, start_pose=start_raw)
                if live_ok and seg_ok:
                    self.get_logger().info(
                        f"compute_path_to_pose attempt={attempt + 1}/{attempts} accepted "
                        f"poses={len(plan.path)} cost={plan.cost:.3f} inflation={inflation:.2f}m"
                    )
                    break
                min_live = self._min_live_clearance_on_path(plan.path, start_pose=start_raw)
                seg_collision = self._segment_grid_collision_sample(plan.path, map_data, start_pose=start_raw)
                reason_bits = []
                if not live_ok:
                    reason_bits.append("live_clearance")
                if not seg_ok:
                    reason_bits.append("segment_grid")
                if (
                    (not live_ok)
                    and seg_ok
                    and (min_live is not None)
                    and (min_live >= self._path_live_obstacle_soft_accept_min_clearance)
                ):
                    self.get_logger().warn(
                        "compute_path_to_pose soft-accept: live clearance below strict threshold "
                        f"but above soft floor (min_live={min_live:.2f}m, "
                        f"strict={self._path_live_obstacle_min_clearance:.2f}m, "
                        f"soft={self._path_live_obstacle_soft_accept_min_clearance:.2f}m)"
                    )
                    break
                self.get_logger().warn(
                    "compute_path_to_pose clearance reject "
                    f"attempt={attempt + 1}/{attempts} reasons={'+'.join(reason_bits)} "
                    f"inflation={inflation:.2f}m poses={len(plan.path)} cost={plan.cost:.3f} "
                    f"min_live_clearance={'n/a' if min_live is None else f'{min_live:.2f}m'} "
                    f"seg_collision={'none' if seg_collision is None else f'({seg_collision[0]:.2f},{seg_collision[1]:.2f})'}"
                )
                near_points = self._collect_live_obstacles_near_path(
                    plan.path, start_pose=start_raw, near_radius_m=self._rejected_obstacle_near_path_radius_m
                )
                if near_points:
                    self._rejected_obstacle_memory.append((time.perf_counter(), near_points))
                    remembered_rejected_obstacle_points.extend(near_points)
                    self.get_logger().info(
                        "compute_path_to_pose reject memory stored "
                        f"obstacle_points={len(near_points)} near_radius={self._rejected_obstacle_near_path_radius_m:.2f}m "
                        f"block_radius={self._rejected_obstacle_block_radius_m:.2f}m"
                    )
                else:
                    self.get_logger().info(
                        "compute_path_to_pose reject memory skipped "
                        f"(no_live_obstacles_near_path within {self._rejected_obstacle_near_path_radius_m:.2f}m)"
                    )
                plan = None

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ComputePathToPose.Result()

            if plan is None:
                rescue_data = list(base_map_data)
                start_rescue, target_rescue = self._adapt_start_and_target(start_raw, target_raw, rescue_data)
                rescue_planner = HybridAStarPlanner(self._map_wrapper.info, rescue_data, **self._planner_cfg)
                rescue_plan = None
                if self._planning_lock.acquire(timeout=self._planner_timeout_sec):
                    try:
                        try:
                            rescue_future = self._planner_pool.submit(
                                rescue_planner.plan,
                                start_rescue,
                                target_rescue,
                                max_iterations=self._planner_max_iterations,
                            )
                            rescue_plan = rescue_future.result(timeout=self._planner_timeout_sec)
                        except FutureTimeoutError:
                            rescue_plan = None
                    finally:
                        self._planning_lock.release()
                if rescue_plan is not None and rescue_plan.path and len(rescue_plan.path) >= 2:
                    self.get_logger().warn(
                        "compute_path_to_pose rescue: accepted plan without live-obstacle overlay "
                        f"(poses={len(rescue_plan.path)}, cost={rescue_plan.cost:.3f})"
                    )
                    plan = rescue_plan
                    start = start_rescue
                    target = target_rescue

            if plan is None:
                fallback = self._try_fallback_to_recent_path(
                    goal_handle, start, target, reason="planner_returned_none_or_clearance_rejected"
                )
                if fallback is not None:
                    return fallback
                return self._abort_to_pose(
                    goal_handle, f"planner_returned_none_or_clearance_rejected; {self._goal_diag(start, target)}"
                )
            if not plan.path:
                fallback = self._try_fallback_to_recent_path(
                    goal_handle, start, target, reason="planner_returned_empty_path"
                )
                if fallback is not None:
                    return fallback
                return self._abort_to_pose(
                    goal_handle, f"planner_returned_empty_path; {self._goal_diag(start, target)}"
                )
            if len(plan.path) < 2:
                fallback = self._try_fallback_to_recent_path(
                    goal_handle, start, target, reason="planner_returned_short_path_lt2"
                )
                if fallback is not None:
                    return fallback
                return self._abort_to_pose(
                    goal_handle, f"planner_returned_short_path_lt2; {self._goal_diag(start, target)}"
                )

            def _initial_move_is_behind(start_pose: Pose2D, plan_result: PlanResult) -> bool:
                if not plan_result.path or len(plan_result.path) < 2:
                    return False
                p1 = plan_result.path[1]
                bearing = math.atan2(p1.y - start_pose.y, p1.x - start_pose.x)
                delta = _norm_angle(bearing - start_pose.yaw)
                return abs(delta) > self._initial_forward_cone_rad

            if self._disallow_initial_reverse and _initial_move_is_behind(start, plan):
                self.get_logger().warn(
                    "planner returned initial reverse move; attempting forward-only replan"
                )
                planner_cfg_no_reverse = dict(self._planner_cfg)
                planner_cfg_no_reverse["allow_reverse"] = False
                planner2 = HybridAStarPlanner(self._map_wrapper.info, map_data, **planner_cfg_no_reverse)
                replan = None
                if self._planning_lock.acquire(timeout=self._planner_timeout_sec):
                    try:
                        try:
                            future2 = self._planner_pool.submit(
                                planner2.plan,
                                start,
                                target,
                                max_iterations=self._planner_max_iterations,
                            )
                            replan = future2.result(timeout=self._planner_timeout_sec)
                        except FutureTimeoutError:
                            self.get_logger().warn("forward-only replan timed out")
                            replan = None
                    finally:
                        self._planning_lock.release()
                else:
                    self.get_logger().warn("could not acquire planning lock for forward-only replan")

                if replan is not None and replan.path and not _initial_move_is_behind(start, replan):
                    self.get_logger().info("forward-only replan succeeded, using forward plan")
                    plan = replan
                elif _initial_move_is_behind(start, plan):
                    goal_bearing = math.atan2(target.y - start.y, target.x - start.x)
                    goal_rel = abs(_norm_angle(goal_bearing - start.yaw))
                    goal_is_behind = goal_rel >= self._allow_initial_reverse_when_goal_behind_rad
                    obstacle_ahead_ctx = self._has_live_obstacle_ahead(
                        start, radius_m=self._allow_initial_reverse_obstacle_ahead_radius_m
                    )
                    # Strict direction policy: do not publish a newly planned path
                    # whose second pose starts behind the robot unless the goal
                    # itself is genuinely behind the robot heading.
                    if not (goal_is_behind or (
                        self._allow_initial_reverse_when_obstacle_ahead and obstacle_ahead_ctx
                    )):
                        self.get_logger().warn(
                            "planner initial reverse rejected by policy "
                            f"(goal_rel_deg={math.degrees(goal_rel):.1f}, "
                            f"goal_is_behind={goal_is_behind}, "
                            f"obstacle_ahead_ctx={obstacle_ahead_ctx}, "
                            f"allow_on_obstacle_ahead={self._allow_initial_reverse_when_obstacle_ahead})"
                        )
                        fallback = self._try_fallback_to_recent_path(
                            goal_handle, start, target, reason="initial_reverse_policy_rejected"
                        )
                        if fallback is not None:
                            return fallback
                        return self._abort_to_pose(
                            goal_handle, f"initial_reverse_policy_rejected; {self._goal_diag(start, target)}"
                        )

            path_msg = self._build_path_msg(plan, frame_id=self._global_frame)
            if len(path_msg.poses) < 2:
                return self._abort_to_pose(goal_handle, f"built_path_invalid_lt2; {self._goal_diag(start, target)}")
            self._path_pub.publish(path_msg)
            self._expansion_pub.publish(self._build_expansion_markers(plan, frame_id=self._global_frame))
            self._last_valid_path_msg = path_msg
            self._last_global_path_msg = path_msg
            self._last_global_path_walltime = time.perf_counter()
            self._last_valid_plan_cost = plan.cost
            self._last_valid_plan_walltime = time.perf_counter()
            self._last_valid_goal = target_raw

            elapsed = time.perf_counter() - started
            self.get_logger().info(
                f"compute_path_to_pose success: poses={len(path_msg.poses)} "
                f"cost={plan.cost:.3f} time={elapsed:.3f}s"
            )
            goal_handle.succeed()

            result = ComputePathToPose.Result()
            result.path = path_msg
            result.planning_time = Duration(seconds=elapsed).to_msg()
            return result
        finally:
            self._action_planning_active = False

    def _execute_through_poses(self, goal_handle):
        started = time.perf_counter()

        if self._map is None or self._map_wrapper is None:
            return self._abort_through_poses(goal_handle, "map_not_ready")

        if goal_handle.request.use_start:
            current = _pose_to_pose2d(goal_handle.request.start)
        else:
            start_ps = self._get_robot_pose()
            if start_ps is None:
                return self._abort_through_poses(goal_handle, "tf_robot_pose_unavailable")
            current = _pose_to_pose2d(start_ps)

        # Greedy stitching: plan segment-by-segment through the list of poses.
        stitched: list[Pose2D] = []
        expanded_all: list[Pose2D] = []
        total_cost = 0.0

        map_data = list(self._map.data)
        self._apply_wall_memory_to_data(map_data)
        self._apply_live_obstacles_to_data(map_data)
        planner = HybridAStarPlanner(self._map_wrapper.info, map_data, **self._planner_cfg)
        for pose_stamped in goal_handle.request.goals:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ComputePathThroughPoses.Result()
            target = _pose_to_pose2d(pose_stamped)
            if not self._planning_lock.acquire(timeout=self._planner_timeout_sec):
                return self._abort_through_poses(
                    goal_handle, f"planner_busy_lock_timeout_{self._planner_timeout_sec:.2f}s"
                )
            try:
                try:
                    future = self._planner_pool.submit(
                        planner.plan,
                        current,
                        target,
                        max_iterations=self._planner_max_iterations,
                    )
                    segment = future.result(timeout=self._planner_timeout_sec)
                except FutureTimeoutError:
                    return self._abort_through_poses(
                        goal_handle, f"planner_future_timeout_{self._planner_timeout_sec:.2f}s"
                    )
            finally:
                self._planning_lock.release()
            if segment is None:
                return self._abort_through_poses(goal_handle, "planner_returned_none_segment")
            if not segment.path:
                return self._abort_through_poses(goal_handle, "planner_returned_empty_segment")
            if len(segment.path) < 2:
                return self._abort_through_poses(goal_handle, "planner_returned_short_segment_lt2")

            # If configured, avoid returning segments whose very first motion is
            # outside the forward cone relative to the current pose. Attempt a
            # forward-only replan for this segment if necessary.
            def _segment_initial_move_invalid(start_pose: Pose2D, plan_result: PlanResult) -> bool:
                if not plan_result.path or len(plan_result.path) < 2:
                    return False
                p1 = plan_result.path[1]
                bearing = math.atan2(p1.y - start_pose.y, p1.x - start_pose.x)
                delta = _norm_angle(bearing - start_pose.yaw)
                return abs(delta) > self._initial_forward_cone_rad

            if self._disallow_initial_reverse and _segment_initial_move_invalid(current, segment):
                self.get_logger().warn("segment planner returned initial move outside forward cone; attempting forward-only replan for segment")
                planner_cfg_no_reverse = dict(self._planner_cfg)
                planner_cfg_no_reverse["allow_reverse"] = False
                planner2 = HybridAStarPlanner(self._map_wrapper.info, map_data, **planner_cfg_no_reverse)
                reseg = None
                if self._planning_lock.acquire(timeout=self._planner_timeout_sec):
                    try:
                        try:
                            future2 = self._planner_pool.submit(
                                planner2.plan,
                                current,
                                target,
                                max_iterations=self._planner_max_iterations,
                            )
                            reseg = future2.result(timeout=self._planner_timeout_sec)
                        except FutureTimeoutError:
                            self.get_logger().warn("forward-only segment replan timed out")
                            reseg = None
                    finally:
                        self._planning_lock.release()
                else:
                    self.get_logger().warn("could not acquire planning lock for forward-only segment replan")

                if reseg is not None and reseg.path and not _segment_initial_move_invalid(current, reseg):
                    self.get_logger().info("forward-only segment replan succeeded, using forward segment")
                    segment = reseg

            if stitched:
                stitched.extend(segment.path[1:])
            else:
                stitched.extend(segment.path)

            if segment.expanded:
                expanded_all.extend(segment.expanded)
            total_cost += segment.cost
            current = target

        elapsed = time.perf_counter() - started
        plan = PlanResult(path=stitched, expanded=expanded_all, cost=total_cost)
        if len(plan.path) < 2:
            return self._abort_through_poses(goal_handle, "stitched_path_invalid_lt2")

        path_msg = self._build_path_msg(plan, frame_id=self._global_frame)
        if len(path_msg.poses) < 2:
            return self._abort_through_poses(goal_handle, "built_stitched_path_invalid_lt2")
        self._path_pub.publish(path_msg)
        self._expansion_pub.publish(self._build_expansion_markers(plan, frame_id=self._global_frame))
        self._last_global_path_msg = path_msg
        self._last_global_path_walltime = time.perf_counter()

        self.get_logger().info(
            f"compute_path_through_poses success: poses={len(path_msg.poses)} "
            f"cost={plan.cost:.3f} time={elapsed:.3f}s"
        )
        goal_handle.succeed()

        result = ComputePathThroughPoses.Result()
        result.path = path_msg
        result.planning_time = Duration(seconds=elapsed).to_msg()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2HybridAStarServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._planner_pool.shutdown(wait=False, cancel_futures=True)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

