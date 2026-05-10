from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Gazebo (ros_gz_sim) + bridge + roboworks model spawn.

    This launch starts modern Gazebo headless by default.
    """
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_sim_launch_file = os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")

    # Use installed package share paths (works both from source + install space).
    pkg_share = get_package_share_directory("hybrid_astar_planner")
    sdf_file = os.path.join(pkg_share, "sim", "models", "roboworks", "model.sdf")
    bridge_config = os.path.join(pkg_share, "sim", "bridge_minimal.yaml")
    world_file = os.path.join(pkg_share, "sim", "worlds", "warehouse_lightweight_gz.sdf")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")

    # Ensure the simulator can resolve `model://roboworks/...` and `model://box/...`
    # URIs by pointing the resource path at the directory that contains those
    # model folders. Ignition Fortress uses IGN_GAZEBO_RESOURCE_PATH (and
    # respects GZ_SIM_RESOURCE_PATH for forward compatibility); GAZEBO_MODEL_PATH
    # is kept for any Gazebo Classic launch path that still consumes it.
    model_path = os.path.join(pkg_share, "sim", "models")
    gz_args = f"-r -v 1 {world_file}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_file]),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "True"}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": bridge_config,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            },
            {"use_sim_time": True},
        ],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file",
                    sdf_file,
                    "-name",
                    "roboworks",
                    "-x",
                    spawn_x,
                    "-y",
                    spawn_y,
                    "-z",
                    spawn_z,
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_x", default_value="-8.0"),
            DeclareLaunchArgument("spawn_y", default_value="-8.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.1"),
            LogInfo(msg="Starting Gazebo + bridge + roboworks spawn"),
            LogInfo(msg=f"World file: {world_file}"),
            LogInfo(msg=f"Robot model SDF: {sdf_file}"),
            LogInfo(msg=f"Bridge config: {bridge_config}"),
            # Export model resource paths so `model://<name>/...` URIs resolve
            # from this package's sim/models directory.
            SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=model_path),
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=model_path),
            SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=model_path),
            gz_sim,
            bridge,
            spawn_robot,
        ]
    )

