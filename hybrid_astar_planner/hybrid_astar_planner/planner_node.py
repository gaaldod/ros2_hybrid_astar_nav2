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

        # Publisher for costmap overlay as MarkerArray (for RViz)
        self._costmap_overlay_pub = self.create_publisher(MarkerArray, "costmap_overlay", 1)
        # Timer to publish costmap overlay at low frequency (0.5 Hz)
        self.create_timer(2.0, self._publish_costmap_overlay_timer)

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

    def _publish_costmap_overlay_timer(self):
        # Publish costmap overlay at a low frequency for RViz visuals
        if self._map is not None:
            self._publish_costmap_overlay(self._map.header)

    def _publish_costmap_overlay(self, header):
        if self._map_wrapper is None:
            return
        from visualization_msgs.msg import Marker, MarkerArray
        markers = MarkerArray()
        info = self._map_wrapper.info
        width, height = info.width, info.height
        res = info.resolution
        ox, oy = info.origin_xy

        # Clear previous markers for both namespaces
        clear_pts = Marker()
        clear_pts.header = header
        clear_pts.ns = "costmap_points"
        clear_pts.id = 0
        clear_pts.action = Marker.DELETEALL
        markers.markers.append(clear_pts)

        clear_txt = Marker()
        clear_txt.header = header
        clear_txt.ns = "costmap_text"
        clear_txt.id = 0
        clear_txt.action = Marker.DELETEALL
        markers.markers.append(clear_txt)

        # Build a POINTS marker for fast, colored visualization of costs
        pts = Marker()
        pts.header = header
        pts.ns = "costmap_points"
        pts.id = 1
        pts.type = Marker.POINTS
        pts.action = Marker.ADD
        # size each point roughly the size of a grid cell
        pts.scale.x = res
        pts.scale.y = res
        pts.scale.z = 0.01
        pts.color.a = 0.0

        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA

        # Only show non-free cells for clarity (cost > 0)
        for iy in range(height):
            for ix in range(width):
                idx = iy * width + ix
                cost = self._map_wrapper._data[idx]
                if cost <= 0:
                    continue
                # position at cell center
                p = Point()
                p.x = ox + (ix + 0.5) * res
                p.y = oy + (iy + 0.5) * res
                p.z = 0.02
                pts.points.append(p)
                # color map: 1..100 -> green->red
                norm = max(0.0, min(1.0, float(cost) / 100.0))
                c = ColorRGBA()
                c.r = norm
                c.g = 1.0 - norm
                c.b = 0.0
                c.a = 0.8
                pts.colors.append(c)

        markers.markers.append(pts)

        # Optionally also include text markers for small maps / presentations
        # Keep them, but they can be visually noisy on large maps.
        marker_id = 0
        for iy in range(height):
            for ix in range(width):
                idx = iy * width + ix
                cost = self._map_wrapper._data[idx]
                if cost <= 0:
                    continue
                m = Marker()
                m.header = header
                m.ns = "costmap_text"
                m.id = marker_id
                marker_id += 1
                m.type = Marker.TEXT_VIEW_FACING
                m.action = Marker.ADD
                m.scale.z = max(0.06, res * 0.5)  # text height scaled for resolution
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.9
                m.pose.position.x = ox + (ix + 0.5) * res
                m.pose.position.y = oy + (iy + 0.5) * res
                m.pose.position.z = 0.05
                m.text = str(cost)
                markers.markers.append(m)

        self._costmap_overlay_pub.publish(markers)

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

