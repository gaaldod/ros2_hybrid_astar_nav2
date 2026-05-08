from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _nav2_param_rewrites(spawn_x: str, spawn_y: str, smac_min_turning_radius: str) -> dict:
    """Rewrites applied on top of nav2_bringup nav2_params.yaml."""
    return {
        # --- AMCL: default pose matches Gazebo spawn → no "set initial pose" warning ---
        "amcl.ros__parameters.set_initial_pose": "true",
        "amcl.ros__parameters.initial_pose.x": spawn_x,
        "amcl.ros__parameters.initial_pose.y": spawn_y,
        "amcl.ros__parameters.initial_pose.z": "0.0",
        "amcl.ros__parameters.initial_pose.yaw": "0.0",
        # AMCL CPU: fewer particles, slightly more frequent updates (good for sim + WSL).
        "amcl.ros__parameters.max_particles": "1000",
        "amcl.ros__parameters.min_particles": "250",
        "amcl.ros__parameters.update_min_a": "0.12",
        "amcl.ros__parameters.update_min_d": "0.12",
        # BT / TF
        "bt_loop_duration": "60",
        "transform_tolerance": "1.0",
        "odom_topic": "/odom_combined",
        "min_y_velocity_threshold": "0.001",
        # BT navigator: run only NavigateToPose mode in this wired setup.
        # NavigateThroughPoses expects additional planner/global-costmap service
        # wiring that is absent when planner_server is intentionally omitted.
        "bt_navigator.ros__parameters.navigators": "['navigate_to_pose']",
        "bt_navigator.ros__parameters.navigate_to_pose.plugin": (
            "nav2_bt_navigator::NavigateToPoseNavigator"
        ),
        "bt_navigator.ros__parameters.default_server_timeout": "30000",
        # Controller: RPP + Ackermann-friendly tuning
        "controller_server.ros__parameters.controller_frequency": "20.0",
        "controller_server.ros__parameters.min_x_velocity_threshold": "0.001",
        "controller_server.ros__parameters.min_y_velocity_threshold": "0.001",
        "controller_server.ros__parameters.min_theta_velocity_threshold": "0.001",
        "controller_server.ros__parameters.progress_checker.required_movement_radius": "0.03",
        "controller_server.ros__parameters.progress_checker.movement_time_allowance": "60.0",
        "controller_server.ros__parameters.current_goal_checker": "general_goal_checker",
        "controller_server.ros__parameters.FollowPath.plugin": (
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        ),
        "controller_server.ros__parameters.FollowPath.desired_linear_vel": "0.22",
        "controller_server.ros__parameters.FollowPath.lookahead_dist": "0.7",
        "controller_server.ros__parameters.FollowPath.min_lookahead_dist": "0.25",
        "controller_server.ros__parameters.FollowPath.max_lookahead_dist": "1.0",
        "controller_server.ros__parameters.FollowPath.lookahead_time": "1.0",
        "controller_server.ros__parameters.FollowPath.max_angular_accel": "2.0",
        "controller_server.ros__parameters.FollowPath.use_rotate_to_heading": "false",
        "controller_server.ros__parameters.FollowPath.allow_reversing": "false",
        "controller_server.ros__parameters.FollowPath.use_velocity_scaled_lookahead_dist": "true",
        "controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel": "0.85",
        "controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_radius": "0.35",
        "controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_speed": "0.14",
        "controller_server.ros__parameters.FollowPath.min_approach_linear_velocity": "0.12",
        "controller_server.ros__parameters.FollowPath.transform_tolerance": "0.5",
        # Planner server: keep up with controller / replans
        "planner_server.ros__parameters.expected_planner_frequency": "15.0",
        # Global planner: use kinematically-feasible Hybrid-A* (SMAC) instead of
        # 2D NavFn so plans respect turning radius and avoid near-in-place flips.
        "planner_server.ros__parameters.planner_plugins": "['GridBased']",
        "planner_server.ros__parameters.GridBased.plugin": "nav2_smac_planner/SmacPlannerHybrid",
        "planner_server.ros__parameters.GridBased.motion_model_for_search": "DUBIN",
        "planner_server.ros__parameters.GridBased.minimum_turning_radius": smac_min_turning_radius,
        "planner_server.ros__parameters.GridBased.angle_quantization_bins": "72",
        "planner_server.ros__parameters.GridBased.reverse_penalty": "4.0",
    "planner_server.ros__parameters.GridBased.change_penalty": "0.2",
    "planner_server.ros__parameters.GridBased.non_straight_penalty": "0.6",
        "planner_server.ros__parameters.GridBased.cost_penalty": "2.0",
        "planner_server.ros__parameters.GridBased.allow_unknown": "true",
        # Global costmap: lower rate saves CPU; local stays responsive
        "global_costmap.global_costmap.ros__parameters.update_frequency": "0.75",
        "local_costmap.local_costmap.ros__parameters.update_frequency": "7.5",
        "local_costmap.local_costmap.ros__parameters.voxel_layer.publish_voxel_map": "false",
        # Clearance tuning: keep larger margin from corners/obstacles.
        "local_costmap.local_costmap.ros__parameters.footprint": (
            "[[0.37, 0.23], [0.37, -0.23], [-0.37, -0.23], [-0.37, 0.23]]"
        ),
        "global_costmap.global_costmap.ros__parameters.footprint": (
            "[[0.37, 0.23], [0.37, -0.23], [-0.37, -0.23], [-0.37, 0.23]]"
        ),
        "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius": "0.90",
        "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius": "0.90",
        "local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor": "2.0",
        "global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor": "2.0",
        # Velocity smoother
        "velocity_smoother.ros__parameters.feedback": "CLOSED_LOOP",
        "velocity_smoother.ros__parameters.scale_velocities": "true",
        "velocity_smoother.ros__parameters.smoothing_frequency": "30.0",
        # Behaviors
        "behavior_server.ros__parameters.cycle_frequency": "12.0",
    }


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("hybrid_astar_planner")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    nav2_params = LaunchConfiguration("nav2_params")
    nav2_map = LaunchConfiguration("nav2_map")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    smac_min_turning_radius = LaunchConfiguration("smac_min_turning_radius").perform(context)

    tuned_nav2_params = RewrittenYaml(
        source_file=nav2_params.perform(context),
        root_key="",
        param_rewrites=_nav2_param_rewrites(spawn_x, spawn_y, smac_min_turning_radius),
        convert_types=True,
    )

    spawn_x_lc = LaunchConfiguration("spawn_x")
    spawn_y_lc = LaunchConfiguration("spawn_y")
    spawn_z_lc = LaunchConfiguration("spawn_z")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gz_sim_roboworks.launch.py")),
        launch_arguments={
            "spawn_x": spawn_x_lc,
            "spawn_y": spawn_y_lc,
            "spawn_z": spawn_z_lc,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "nav2_bringup_no_planner.launch.py")
        ),
        launch_arguments={
            "params_file": tuned_nav2_params,
            "map": nav2_map,
            "slam": "False",
            "use_sim_time": "True",
            "autostart": "True",
        }.items(),
    )
    hybrid_global_planner = Node(
        package="hybrid_astar_planner",
        executable="nav2_hybrid_astar_server",
        name="nav2_hybrid_astar_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "minimum_turning_radius": 1.35,
                "allow_reverse": True,
                # Keep planner search bounded to avoid compute_path timeouts.
                "angle_quantization_bins": 72,
                "allow_primitive_interpolation": False,
                "reverse_penalty": 0.9,
                "non_straight_penalty": 0.6,
                "direction_change_penalty": 0.3,
                "steering_change_penalty": 0.1,
                "planner_max_iterations": 12000,
                "planner_timeout_sec": 20.0,
                # Inflate remembered collision points more aggressively so replans
                # avoid hugging the same corner/wall zones.
                "wall_memory_inflation_radius_m": 0.9,
            }
        ],
    )
    global_costmap_compat = Node(
        package="hybrid_astar_planner",
        executable="global_costmap_compat_node",
        name="global_costmap_compat_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    delivery = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "delivery_stack.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "scan_topic": "scan",
            "global_frame": "map",
            "launch_hybrid_global": "false",
        }.items(),
    )

    odom_to_base = Node(
        package="hybrid_astar_planner",
        executable="odom_tf_bridge_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "use_latest_stamp": False,
                "odom_topic": "odom_combined",
                "tf_parent_frame": "odom",
                "tf_child_frame": "base_link",
            }
        ],
    )
    base_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_to_laser",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--x",
            "0.25",
            "--y",
            "0",
            "--z",
            "0.094",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "laser",
        ],
        output="screen",
    )
    base_to_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_to_footprint",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "base_footprint",
        ],
        output="screen",
    )
    # Light backup after AMCL param init; helps if spawn and AMCL ever drift.
    initial_pose_seed = Node(
        package="hybrid_astar_planner",
        executable="initial_pose_seed_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "pose_x": spawn_x_lc,
                "pose_y": spawn_y_lc,
                "pose_yaw": 0.0,
                # Short startup burst only. Excessive re-seeding keeps resetting
                # AMCL pose during navigation and prevents controller progress.
                "publish_count": 4,
                "publish_period_s": 0.4,
            }
        ],
    )
    rviz = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="nav2_delivery_debug_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return [
        LogInfo(msg="Starting modern Gazebo + Nav2 corner goal test stack"),
        sim,
        # Start the moving obstacle node after the sim has a moment to initialize.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="hybrid_astar_planner",
                    executable="moving_obstacles_node",
                    name="moving_obstacles_node",
                    output="screen",
                    parameters=[
                        {"use_sim_time": True},
                        # Example moving obstacle params (override in launch args if desired)
                        {"moving_obstacle_name": "moving_center_box"},
                        {"moving_obstacle_amp_y": 8.0},
                        {"moving_obstacle_freq": 0.25},
                    ],
                )
            ],
        ),
        odom_to_base,
        base_to_footprint,
        base_to_laser,
        nav2,
        hybrid_global_planner,
        global_costmap_compat,
        # Delay seeding slightly until Nav2 localization is up, then stop quickly.
        TimerAction(period=8.0, actions=[initial_pose_seed]),
        delivery,
        # Delay RViz startup to avoid early message-filter queue flooding before
        # map->odom->base_link->laser TF chain is available.
        TimerAction(period=10.0, actions=[rviz]),
    ]


def generate_launch_description() -> LaunchDescription:
    """
    Modern Gazebo + Nav2 action-goal corner test setup.

    AMCL initial pose defaults match ``spawn_x`` / ``spawn_y`` so localization starts
    immediately without RViz "2D Pose Estimate".
    """
    try:
        get_package_share_directory("ros_gz_sim")
    except PackageNotFoundError:
        return LaunchDescription(
            [
                LogInfo(
                    msg=(
                        "ERROR: 'ros_gz_sim' package not found. Install modern Gazebo bridge:\n"
                        "  sudo apt update && sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge"
                    )
                ),
                Shutdown(reason="ros_gz_sim missing"),
            ]
        )

    try:
        nav2_bringup_share = get_package_share_directory("nav2_bringup")
    except PackageNotFoundError:
        return LaunchDescription(
            [
                LogInfo(
                    msg=(
                        "ERROR: 'nav2_bringup' package not found. Install Nav2:\n"
                        "  sudo apt update && sudo apt install ros-humble-nav2-bringup"
                    )
                ),
                Shutdown(reason="nav2_bringup missing"),
            ]
        )

    pkg_share = get_package_share_directory("hybrid_astar_planner")

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="FASTDDS_BUILTIN_TRANSPORTS", value="UDPv4"),
            DeclareLaunchArgument("spawn_x", default_value="-8.0"),
            DeclareLaunchArgument("spawn_y", default_value="-8.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.1"),
            DeclareLaunchArgument(
                "smac_min_turning_radius",
                default_value="1.35",
                description="Smac Hybrid planner minimum turning radius (meters).",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=os.path.join(nav2_bringup_share, "params", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_map",
                default_value=os.path.join(pkg_share, "maps", "warehouse_lightweight_map.yaml"),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz visualization for lidar/path debugging.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(pkg_share, "rviz", "nav2_delivery_debug.rviz"),
                description="RViz config file path.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
