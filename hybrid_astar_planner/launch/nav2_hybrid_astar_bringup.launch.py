from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Bring up Nav2 with Hybrid A* planning actions provided by Python.

    Run Gazebo + bridge separately (e.g. `gz_sim_roboworks.launch.py`).
    """
    pkg_share = get_package_share_directory("hybrid_astar_planner")

    try:
        nav2_bringup_share = get_package_share_directory("nav2_bringup")
    except PackageNotFoundError:
        return LaunchDescription(
            [
                LogInfo(
                    msg=(
                        "ERROR: 'nav2_bringup' package not found. "
                        "Install Nav2 for ROS 2 Humble, e.g.: "
                        "sudo apt update && sudo apt install ros-humble-nav2-bringup"
                    )
                ),
                Shutdown(reason="Nav2 not installed (nav2_bringup missing)."),
            ]
        )

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    slam = LaunchConfiguration("slam")
    use_mock_tf = LaunchConfiguration("use_mock_tf")
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav2_bringup_share, "params", "nav2_params.yaml"),
        description="Nav2 parameters file (default: nav2_bringup defaults)",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_share, "maps", "empty_map.yaml"),
        description="Full path to map yaml file to load (required by nav2_bringup).",
    )
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether to run SLAM. For smoke-tests, False is more robust with mock TF.",
    )
    use_mock_tf_arg = DeclareLaunchArgument(
        "use_mock_tf",
        default_value="true",
        description="If true, publish static mock TF tree (disable when simulation already publishes TF).",
    )

    # Our planner action server (replaces planner_server for BT navigator requests)
    hybrid_planner = Node(
        package="hybrid_astar_planner",
        executable="nav2_hybrid_astar_server",
        name="nav2_hybrid_astar_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Use our Nav2 bringup variant that omits planner_server to avoid action name conflicts.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "nav2_bringup_no_planner.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": params_file,
            "autostart": "True",
            "map": map_yaml,
            "slam": slam,
        }.items(),
    )

    # Delay starting the bulk of Nav2 briefly to give the hybrid planner
    # action server time to come up and register. Without this delay the
    # bt_navigator may attempt to load its BT and fail because the planner
    # action server isn't visible yet (1s timeout inside BT loading).
    return LaunchDescription(
        [
            params_arg,
            map_arg,
            slam_arg,
            use_mock_tf_arg,
            LogInfo(msg="Starting Nav2 bringup + Hybrid A* planning actions"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "mock_tf_tree.launch.py")),
                condition=IfCondition(use_mock_tf),
            ),
            hybrid_planner,
            TimerAction(
                period=2.0,
                actions=[nav2],
            ),
        ]
    )

