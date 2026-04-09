from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from visualization_msgs.msg import Marker, MarkerArray

from .grid_map import OccupancyGridMap
from .hybrid_astar import HybridAStarPlanner
from .types import GridInfo, PlanResult, Pose2D


class HybridAStarPlannerNode(Node):
    """
    Standalone Hybrid A* global planner node.

    - Subscribes to a static or dynamic OccupancyGrid (`map`).
    - Exposes a `nav_msgs/srv/GetPlan` service (`/hybrid_astar/make_plan`).
    - Publishes RViz-friendly topics:
        * `planned_path` (`nav_msgs/Path`)
        * `search_expansions` (`visualization_msgs/MarkerArray`)
    """

    def __init__(self) -> None:
        super().__init__("hybrid_astar_planner_node")

        self.declare_parameter("map_topic", "map")
        self.declare_parameter("plan_topic", "planned_path")
        self.declare_parameter("expansions_topic", "search_expansions")

        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        plan_topic = self.get_parameter("plan_topic").get_parameter_value().string_value
        expansions_topic = self.get_parameter("expansions_topic").get_parameter_value().string_value

        self._map: Optional[OccupancyGrid] = None
        self._map_wrapper: Optional[OccupancyGridMap] = None

        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, 10
        )
        self._plan_pub = self.create_publisher(Path, plan_topic, 10)
        self._expansion_pub = self.create_publisher(MarkerArray, expansions_topic, 10)

        self._plan_srv = self.create_service(GetPlan, "hybrid_astar/make_plan", self._on_make_plan)

        self.get_logger().info(
            f"HybridAStarPlannerNode started. Listening for map on '{map_topic}', "
            f"service 'hybrid_astar/make_plan', publishing path on '{plan_topic}'."
        )

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
        self.get_logger().info(
            f"Received map: {grid_info.width}x{grid_info.height}, resolution={grid_info.resolution:.3f}"
        )

    def _on_make_plan(self, request: GetPlan.Request, response: GetPlan.Response) -> GetPlan.Response:
        if self._map is None or self._map_wrapper is None:
            self.get_logger().warn("No map available yet; cannot plan.")
            return response

        start_pose = request.start
        goal_pose = request.goal

        start = Pose2D(
            x=start_pose.pose.position.x,
            y=start_pose.pose.position.y,
            yaw=self._yaw_from_quaternion(start_pose.pose.orientation),
        )
        goal = Pose2D(
            x=goal_pose.pose.position.x,
            y=goal_pose.pose.position.y,
            yaw=self._yaw_from_quaternion(goal_pose.pose.orientation),
        )

        self.get_logger().info(
            f"Planning from ({start.x:.2f}, {start.y:.2f}) to ({goal.x:.2f}, {goal.y:.2f})."
        )

        planner = HybridAStarPlanner(self._map_wrapper.info, list(self._map.data))
        result = planner.plan(start, goal)

        if result is None or not result.path:
            self.get_logger().warn("Hybrid A* could not find a path.")
            return response

        path_msg = self._build_path_msg(result, start_pose.header.frame_id or self._map.header.frame_id)
        response.plan = path_msg

        self._plan_pub.publish(path_msg)
        markers = self._build_expansion_markers(
            result, start_pose.header.frame_id or self._map.header.frame_id
        )
        self._expansion_pub.publish(markers)

        self.get_logger().info(
            f"Hybrid A* produced path with {len(result.path)} poses, cost={result.cost:.3f}"
        )
        return response

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        # Minimal quaternion to yaw conversion to avoid extra deps at this layer.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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

            half_yaw = pose.yaw * 0.5
            ps.pose.orientation.z = math.sin(half_yaw)
            ps.pose.orientation.w = math.cos(half_yaw)

            path_msg.poses.append(ps)
        return path_msg

    def _build_expansion_markers(self, result: PlanResult, frame_id: str) -> MarkerArray:
        markers = MarkerArray()

        # Clear previous markers
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
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5

        from geometry_msgs.msg import Point

        for pose in result.expanded:
            p = Point()
            p.x = pose.x
            p.y = pose.y
            p.z = 0.0
            marker.points.append(p)

        markers.markers.append(marker)
        return markers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HybridAStarPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

