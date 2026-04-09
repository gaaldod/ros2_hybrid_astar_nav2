from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Gazebo (ros_gz_sim) + bridge + roboworks model spawn.

    This launch is intended for WSL/headless usage (server-only Gazebo).
    """
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_sim_launch_file = os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")

    # Use installed package share paths (works both from source + install space).
    pkg_share = get_package_share_directory("hybrid_astar_planner")
    sdf_file = os.path.join(pkg_share, "sim", "roboworks_model", "roboworks", "model.sdf")
    bridge_config = os.path.join(pkg_share, "sim", "bridge_minimal.yaml")

    # Minimal headless world content (no GUI rendering).
    world_content = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="minimal_headless">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>"""

    # Write world into a deterministic temp folder under package share.
    tmp_dir = os.path.join(pkg_share, "sim", "_tmp")
    os.makedirs(tmp_dir, exist_ok=True)
    world_file = os.path.join(tmp_dir, "minimal_world.sdf")
    with open(world_file, "w", encoding="utf-8") as f:
        f.write(world_content)

    gz_args = f"-s -r -v 1 {world_file}"

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
                arguments=["-file", sdf_file, "-name", "roboworks", "-x", "0", "-y", "0", "-z", "0.1"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            LogInfo(msg="Starting Gazebo headless + bridge + roboworks spawn"),
            LogInfo(msg=f"World file: {world_file}"),
            LogInfo(msg=f"Robot model SDF: {sdf_file}"),
            LogInfo(msg=f"Bridge config: {bridge_config}"),
            gz_sim,
            bridge,
            spawn_robot,
        ]
    )

