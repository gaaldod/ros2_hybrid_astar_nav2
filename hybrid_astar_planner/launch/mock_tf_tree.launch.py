from __future__ import annotations

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Minimal TF tree for smoke-testing Nav2 without full localization stack.

    Publishes:
    - map -> odom (identity)
    - odom -> base_link (identity)
    - base_link -> laser (from roboworks model.sdf)
    """

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        output="screen",
    )

    base_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_to_laser",
        # From sim/roboworks_model/roboworks/model.sdf: <link name="laser"><pose>0.25 0 0.094 ...</pose>
        arguments=["0.25", "0", "0.094", "0", "0", "0", "base_link", "laser"],
        output="screen",
    )

    return LaunchDescription([map_to_odom, odom_to_base, base_to_laser])

