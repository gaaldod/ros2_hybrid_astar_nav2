from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformException, TransformListener

from .grid_map import OccupancyGridMap
from .hybrid_astar import HybridAStarPlanner
from .types import GridInfo, PlanResult, Pose2D


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _pose_to_pose2d(ps: PoseStamped) -> Pose2D:
    return Pose2D(
        x=ps.pose.position.x,
        y=ps.pose.position.y,
        yaw=_yaw_from_quaternion(ps.pose.orientation),
    )


class Nav2HybridAStarServer(Node):
    """
    Nav2-compatible planning action servers implemented in Python.

    Provides:
    - `compute_path_to_pose` (nav2_msgs/action/ComputePathToPose)
    - `compute_path_through_poses` (nav2_msgs/action/ComputePathThroughPoses)

    This lets you run Nav2 BT Navigator without the default `planner_server`.
    """

    def __init__(self) -> None:
        super().__init__("nav2_hybrid_astar_server")

        self.declare_parameter("map_topic", "map")
        self.declare_parameter("path_topic", "planned_path")
        self.declare_parameter("expansions_topic", "search_expansions")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")

        map_topic = self.get_parameter("map_topic").value
        self._global_frame = self.get_parameter("global_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value

        self._map: Optional[OccupancyGrid] = None
        self._map_wrapper: Optional[OccupancyGridMap] = None

        self._map_sub = self.create_subscription(OccupancyGrid, map_topic, self._on_map, 10)
        self._path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self._expansion_pub = self.create_publisher(
            MarkerArray, self.get_parameter("expansions_topic").value, 10
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._as_to_pose = ActionServer(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
            execute_callback=self._execute_to_pose,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )

        self._as_through_poses = ActionServer(
            self,
            ComputePathThroughPoses,
            "compute_path_through_poses",
            execute_callback=self._execute_through_poses,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )

        self.get_logger().info(
            "Nav2HybridAStarServer ready. Actions: 'compute_path_to_pose', 'compute_path_through_poses'."
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

    def _accept_goal(self, _goal_request) -> GoalResponse:
        if self._map_wrapper is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _get_robot_pose(self) -> Optional[PoseStamped]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return None

        ps = PoseStamped()
        ps.header.frame_id = self._global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        return ps

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
            half = 0.5 * pose.yaw
            ps.pose.orientation.z = math.sin(half)
            ps.pose.orientation.w = math.cos(half)
            path_msg.poses.append(ps)
        return path_msg

    def _build_expansion_markers(self, result: PlanResult, frame_id: str) -> MarkerArray:
        markers = MarkerArray()
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
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 0.35

        from geometry_msgs.msg import Point

        for pose in result.expanded:
            p = Point()
            p.x = pose.x
            p.y = pose.y
            p.z = 0.0
            marker.points.append(p)

        markers.markers.append(marker)
        return markers

    async def _execute_to_pose(self, goal_handle):
        started = time.perf_counter()

        if self._map is None or self._map_wrapper is None:
            goal_handle.abort()
            return ComputePathToPose.Result()

        goal: PoseStamped = goal_handle.request.goal

        if goal_handle.request.use_start:
            start_ps = goal_handle.request.start
        else:
            start_ps = self._get_robot_pose()
            if start_ps is None:
                goal_handle.abort()
                return ComputePathToPose.Result()

        start = _pose_to_pose2d(start_ps)
        target = _pose_to_pose2d(goal)

        planner = HybridAStarPlanner(self._map_wrapper.info, list(self._map.data))
        plan = planner.plan(start, target)

        if plan is None or not plan.path:
            goal_handle.abort()
            return ComputePathToPose.Result()

        path_msg = self._build_path_msg(plan, frame_id=self._global_frame)
        self._path_pub.publish(path_msg)
        self._expansion_pub.publish(self._build_expansion_markers(plan, frame_id=self._global_frame))

        elapsed = time.perf_counter() - started
        goal_handle.succeed()

        result = ComputePathToPose.Result()
        result.path = path_msg
        result.planning_time = Duration(seconds=elapsed).to_msg()
        return result

    async def _execute_through_poses(self, goal_handle):
        started = time.perf_counter()

        if self._map is None or self._map_wrapper is None:
            goal_handle.abort()
            return ComputePathThroughPoses.Result()

        if goal_handle.request.use_start:
            current = _pose_to_pose2d(goal_handle.request.start)
        else:
            start_ps = self._get_robot_pose()
            if start_ps is None:
                goal_handle.abort()
                return ComputePathThroughPoses.Result()
            current = _pose_to_pose2d(start_ps)

        # Greedy stitching: plan segment-by-segment through the list of poses.
        stitched: list[Pose2D] = []
        expanded_all: list[Pose2D] = []
        total_cost = 0.0

        planner = HybridAStarPlanner(self._map_wrapper.info, list(self._map.data))
        for pose_stamped in goal_handle.request.goals:
            target = _pose_to_pose2d(pose_stamped)
            segment = planner.plan(current, target)
            if segment is None or not segment.path:
                goal_handle.abort()
                return ComputePathThroughPoses.Result()

            if stitched:
                stitched.extend(segment.path[1:])
            else:
                stitched.extend(segment.path)

            if segment.expanded:
                expanded_all.extend(segment.expanded)
            total_cost += segment.cost
            current = target

        elapsed = time.perf_counter() - started
        plan = PlanResult(path=stitched, expanded=expanded_all, cost=total_cost)

        path_msg = self._build_path_msg(plan, frame_id=self._global_frame)
        self._path_pub.publish(path_msg)
        self._expansion_pub.publish(self._build_expansion_markers(plan, frame_id=self._global_frame))

        goal_handle.succeed()

        result = ComputePathThroughPoses.Result()
        result.path = path_msg
        result.planning_time = Duration(seconds=elapsed).to_msg()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2HybridAStarServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

