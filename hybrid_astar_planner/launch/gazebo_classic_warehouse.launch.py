from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_roboworks_sdf() -> str:
    """
    Resolve a roboworks SDF path.

    Preferred source is `robotverseny_description` if available in the ament index.
    Fallback path points at this repository's bundled model.
    """
    try:
        desc_share = get_package_share_directory("robotverseny_description")
        return os.path.join(desc_share, "models", "roboworks", "model.sdf")
    except Exception:
        # Fall back to the bundled roboworks model shipped in this package.
        pkg_share = get_package_share_directory("hybrid_astar_planner")
        return os.path.join(pkg_share, "roboworks", "model.sdf")


def generate_launch_description() -> LaunchDescription:
    """
    Gazebo Classic warehouse test setup.

    Starts:
    - Gazebo Classic with a tweaked AWS small warehouse world
    - roboworks spawn into the warehouse

    Notes:
    - Dynamic obstacles are world-side scripted actors in the `.world` file.
    - This launch focuses on early environment integration and perception tests.
    """
    pkg_share = get_package_share_directory("hybrid_astar_planner")
    try:
        gazebo_ros_share = get_package_share_directory("gazebo_ros")
    except PackageNotFoundError:
        return LaunchDescription(
            [
                LogInfo(
                    msg=(
                        "ERROR: package 'gazebo_ros' not found. "
                        "Install Gazebo Classic integration for ROS 2 Humble:\n"
                        "  sudo apt update && sudo apt install ros-humble-gazebo-ros-pkgs"
                    )
                ),
                Shutdown(reason="gazebo_ros missing"),
            ]
        )
    world_light = os.path.join(pkg_share, "sim", "worlds", "warehouse_lightweight.world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    roboworks_sdf = LaunchConfiguration("roboworks_sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_x", default_value="-6.0"),
            DeclareLaunchArgument("spawn_y", default_value="-8.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("roboworks_sdf", default_value=_default_roboworks_sdf()),
            SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=os.environ.get('GAZEBO_MODEL_PATH','')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
                ),
                launch_arguments={"world": world_light}.items(),
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-entity",
                    "roboworks",
                    "-file",
                    roboworks_sdf,
                    "-x",
                    spawn_x,
                    "-y",
                    spawn_y,
                    "-z",
                    spawn_z,
                    "-Y",
                    spawn_yaw,
                ],
            ),
        ]
    )
