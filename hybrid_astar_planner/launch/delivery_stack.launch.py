"""
Launch the five-node delivery pipeline together with the Hybrid A* Nav2 planner server.

Typical use with Gazebo Classic + Nav2:

* Run your robot / simulation and Nav2 bringup (e.g. ``nav2_hybrid_astar_bringup.launch.py``)
  **or** include that launch from a higher-level file.
* This launch only starts the perception/safety pipeline and the Python
  ``compute_path_to_pose`` action server.

Remapping: In this stack Nav2 publishes ``geometry_msgs/Twist`` on ``/cmd_vel_nav``.
This node subscribes to ``/cmd_vel_nav``, applies scaling and limits, and
republishes to ``/cmd_vel_safe``.
The Gazebo bridge (``sim/bridge_minimal.yaml``) forwards ``/cmd_vel_safe`` to the
simulator so only this node feeds the robot — not Nav2 and the bridge both on the
same topic.

Set ``use_sim_time`` true when playing bags or simulation clock.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    global_frame = LaunchConfiguration("global_frame")
    launch_hybrid_global = LaunchConfiguration("launch_hybrid_global")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Set true for Gazebo Classic / bags.",
    )
    declare_scan = DeclareLaunchArgument(
        "scan_topic",
        default_value="scan",
        description="Raw ``sensor_msgs/LaserScan`` topic from the robot.",
    )
    declare_global_frame = DeclareLaunchArgument(
        "global_frame",
        default_value="map",
        description="Frame for tracked obstacles and the point cloud (must match localization).",
    )
    declare_launch_hybrid_global = DeclareLaunchArgument(
        "launch_hybrid_global",
        default_value="true",
        description="Set false when nav2_hybrid_astar_server is launched elsewhere.",
    )

    sensor_proc = Node(
        package="hybrid_astar_planner",
        executable="sensor_processor_node",
        name="sensor_processor_node",
        output="screen",
        parameters=[
            {
                "scan_topic": scan_topic,
                "scan_filtered_topic": "scan_filtered",
                "median_window": 0,
                "replace_inf_with_max": True,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    obstacle_tracker = Node(
        package="hybrid_astar_planner",
        executable="obstacle_tracker_node",
        name="obstacle_tracker_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "scan_topic": "scan_filtered",
                "tracks_topic": "tracked_obstacles",
                "global_frame": global_frame,
                "laser_frame": "laser",
                "diagnostic_log_every": 1000,
                # Keep lidar detections stable in map frame in sim: use latest
                # TF on arrival rather than exact scan-time lookup.
                "tf_ignore_scan_timestamp": True,
                "use_latest_tf_fallback": True,
                "max_tf_age_sec": 1.5,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    local_planner = Node(
        package="hybrid_astar_planner",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
        parameters=[
            {
                "tracks_topic": "tracked_obstacles",
                "path_topic": "planned_path",
                "obstacle_cloud_topic": "tracked_obstacles_cloud",
                "speed_scale_topic": "nav_speed_scale",
                "emergency_stop_topic": "nav_emergency_stop",
                # Keep obstacle cloud continuous; periodic empty-cloud flushing
                # can create transient holes that let global plans cut walls.
                "cloud_flush_interval": 0.0,
                "global_frame": global_frame,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    safety = Node(
        package="hybrid_astar_planner",
        executable="ackermann_safety_controller_node",
        name="ackermann_safety_controller_node",
        output="screen",
        parameters=[
            {
                "cmd_vel_in_topic": "cmd_vel_nav",
                "cmd_vel_out_topic": "cmd_vel_safe",
                "speed_scale_topic": "nav_speed_scale",
                "emergency_stop_topic": "nav_emergency_stop",
                "aux_emergency_stop_topic": "nav_localization_jump_stop",
                "aux_emergency_stop_topic_2": "nav_anchor_drift_stop",
                "max_linear_speed": 2.0,
                "max_angular_speed": 1.2,
                "recovery_active_topic": "nav_recovery_active",
                "recovery_cmd_topic": "cmd_vel_recovery",
                # Keep long straights straighter: reduce early steering pull
                # from path-assist and only intervene near larger heading error.
                "path_assist_enabled": True,
                "path_assist_distance_m": 0.75,
                "path_assist_heading_deadband_rad": 0.18,
                "path_assist_min_angular_rad_s": 0.08,
                "path_assist_max_angular_rad_s": 0.35,
                "path_assist_lookahead_points": 3,
                "path_rejoin_countersteer_enabled": True,
                "path_rejoin_band_m": 0.90,
                "path_rejoin_countersteer_deadband_m": 0.08,
                "path_rejoin_heading_weight": 0.70,
                "path_rejoin_cross_track_gain": 1.10,
                "path_rejoin_speed_floor_mps": 0.10,
                "front_collision_stop_enabled": True,
                "front_collision_scan_topic": "scan_filtered",
                "front_collision_stop_distance_m": 0.40,
                "front_collision_half_angle_rad": 0.35,
                "front_collision_min_points": 3,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    reverse_recovery = Node(
        package="hybrid_astar_planner",
        executable="reverse_recovery_node",
        name="reverse_recovery_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "path_topic": "planned_path",
                "odom_topic": "odom_combined",
                "cmd_nav_topic": "cmd_vel_nav",
                "cmd_safe_topic": "cmd_vel_safe",
                "recovery_active_topic": "nav_recovery_active",
                "recovery_cmd_topic": "cmd_vel_recovery",
                "dist_to_path_trigger": 1.2,
                "heading_error_trigger": 0.8,
                "stuck_zero_speed_streak_threshold": 25,
                "stuck_dist_to_path_trigger": 0.8,
                "reverse_linear_speed": 0.18,
                "reverse_angular_speed": 0.35,
                "collision_warning_threshold": 1,
                "collision_immediate_trigger": True,
                "collision_immediate_window_sec": 0.5,
                "reverse_min_duration_sec": 0.8,
                "reverse_hard_max_duration_sec": 14.0,
                "reverse_clear_dist_to_path_m": 0.65,
                "reverse_clear_heading_error_rad": 0.65,
                "reverse_collision_quiet_sec": 1.2,
                "maneuver_duration_sec": 1.4,
                "trigger_cooldown_sec": 8.0,
                # Avoid mid-run goal preemption loops from pre-replan goal reissue.
                "pre_replan_enabled": False,
                "allow_goal_republish_pre_replan": False,
                # Suppress all recovery triggers briefly after goal receipt.
                "goal_start_grace_sec": 14.0,
                # Also suppress recovery before first goal is observed and during
                # initial node startup stabilization.
                "require_goal_before_trigger": True,
                "startup_grace_sec": 12.0,
                "log_debug": False,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    replan_watchdog = Node(
        package="hybrid_astar_planner",
        executable="replan_watchdog_node",
        name="replan_watchdog_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "goal_topic": "goal_pose",
                "cmd_topic": "cmd_vel_safe",
                "path_topic": "planned_path",
                "tick_hz": 1.0,
                "enable_periodic_replan": False,
                "periodic_replan_sec": 5.0,
                "zero_speed_linear_eps": 0.02,
                "zero_speed_angular_eps": 0.05,
                "zero_streak_threshold": 8,
                "replan_cooldown_sec": 10.0,
                "path_stale_for_stall_sec": 4.0,
                "goal_freshness_timeout_sec": 300.0,
                "enable_zero_speed_replan": False,
                "log_debug": False,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    tf_event_logger = Node(
        package="hybrid_astar_planner",
        executable="tf_event_logger_node",
        name="tf_event_logger_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "global_frame": global_frame,
                "odom_frame": "odom",
                "base_frame": "base_link",
                "tick_hz": 10.0,
                "yaw_jump_warn_deg": 8.0,
                "xy_jump_warn_m": 0.35,
                "clock_back_jump_warn_sec": 0.3,
                "clock_gap_warn_sec": 1.5,
                "tf_stale_warn_sec": 0.35,
                "amcl_pose_topic": "/amcl_pose",
                "amcl_jump_warn_m": 0.5,
                "amcl_yaw_jump_warn_deg": 12.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    anchor_frame_guard = Node(
        package="hybrid_astar_planner",
        executable="anchor_frame_guard_node",
        name="anchor_frame_guard_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "odom_topic": "odom_combined",
                "amcl_pose_topic": "/amcl_pose",
                "initialpose_topic": "/initialpose",
                "drift_warn_m": 0.8,
                "drift_warn_yaw_deg": 18.0,
                "reseed_enabled": True,
                "reseed_trigger_m": 1.6,
                "reseed_trigger_yaw_deg": 24.0,
                "reseed_sustained_trigger_m": 1.2,
                "reseed_sustained_trigger_yaw_deg": 20.0,
                "reseed_sustained_sec": 6.0,
                "reseed_cooldown_sec": 10.0,
                "reseed_max_speed_mps": 0.08,
                "max_reseed_xy_step_m": 1.2,
                "max_reseed_yaw_step_deg": 20.0,
                "persistent_drift_stop_enabled": True,
                "persistent_drift_stop_yaw_deg": 15.0,
                "persistent_drift_stop_dist_m": 0.6,
                "persistent_drift_stop_sec": 8.0,
                "persistent_drift_stop_topic": "nav_anchor_drift_stop",
                "goal_topic": "goal_pose",
                "goal_republish_after_reseed": True,
                "goal_republish_delay_sec": 0.4,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    hybrid_global = Node(
        condition=IfCondition(launch_hybrid_global),
        package="hybrid_astar_planner",
        executable="nav2_hybrid_astar_server",
        name="nav2_hybrid_astar_server",
        output="screen",
        parameters=[
            {
                "map_topic": "map",
                "path_topic": "planned_path",
                "expansions_topic": "search_expansions",
                "global_frame": global_frame,
                "robot_base_frame": "base_link",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_scan,
            declare_global_frame,
            declare_launch_hybrid_global,
            sensor_proc,
            obstacle_tracker,
            local_planner,
            reverse_recovery,
            replan_watchdog,
            tf_event_logger,
            anchor_frame_guard,
            safety,
            hybrid_global,
        ]
    )
