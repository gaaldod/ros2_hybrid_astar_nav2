from __future__ import annotations

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Simple demo launch:
      - hybrid_astar_planner_node
      - rviz2 with a preconfigured view

    Assumes a separate map server is running and publishing `map`.
    """
    planner_node = Node(
        package="hybrid_astar_planner",
        executable="hybrid_astar_planner_node",
        name="hybrid_astar_planner_node",
        output="screen",
        parameters=[
            {
                "map_topic": "map",
                "plan_topic": "planned_path",
                "expansions_topic": "search_expansions",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            "share/hybrid_astar_planner/rviz/hybrid_astar_demo.rviz",
        ],
        output="screen",
    )

    return LaunchDescription([planner_node, rviz_node])

