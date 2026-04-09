from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Bring up Nav2 with Hybrid A* planning actions provided by Python.

    Run Gazebo + bridge separately (e.g. `gz_sim_roboworks.launch.py`).
    """
    pkg_share = get_package_share_directory("hybrid_astar_planner")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    params_file = LaunchConfiguration("params_file")
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "config", "nav2_roboworks_minimal.yaml"),
        description="Nav2 parameters file",
    )

    # Our planner action server (replaces planner_server for BT navigator requests)
    hybrid_planner = Node(
        package="hybrid_astar_planner",
        executable="nav2_hybrid_astar_server",
        name="nav2_hybrid_astar_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Standard Nav2 bringup launch, but you should disable planner_server in composition
    # or avoid launching it in your own Nav2 bringup variant if it conflicts.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": params_file,
            "autostart": "True",
        }.items(),
    )

    return LaunchDescription(
        [
            params_arg,
            LogInfo(msg="Starting Nav2 bringup + Hybrid A* planning actions"),
            hybrid_planner,
            nav2,
        ]
    )

