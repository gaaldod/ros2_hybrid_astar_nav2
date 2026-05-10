from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

from .grid_map import GridIndex, OccupancyGridMap
from .types import GridInfo, PlanResult, Pose2D


@dataclass(frozen=True)
class MotionPrimitive:
    distance: float
    delta_yaw: float
    reverse: bool


@dataclass
class SearchState:
    x: float
    y: float
    yaw: float
    direction: int  # +1 forward, -1 reverse
    cost_so_far: float
    heuristic: float
    parent: Optional[int]
    grid_index: GridIndex
    primitive_index: int = -1

    @property
    def total_score(self) -> float:
        return self.cost_so_far + self.heuristic


class HybridAStarPlanner:
    """
    Simple Hybrid A* implementation tailored to ROS 2 Humble Python nodes.

    This implementation is intentionally written with a naming scheme and
    structure distinct from common open-source Hybrid A* examples.
    """

    def __init__(
        self,
        grid_info: GridInfo,
        occupancy: List[int],
        *,
        step_length: float = 0.75,
        angle_quantization_bins: int = 72,
        allow_reverse: bool = True,
        minimum_turning_radius: float = 1.35,
        allow_primitive_interpolation: bool = True,
        collision_radius: float = 0.35,
        heuristic_weight: float = 1.1,
        reverse_penalty: float = 1.2,
        non_straight_penalty: float = 1.2,
        direction_change_penalty: float = 0.3,
        steering_change_penalty: float = 0.1,
        treat_unknown_as_occupied: bool = False,
    ) -> None:
        self._grid_info = grid_info
        self._map = OccupancyGridMap(grid_info, occupancy)
        self._step_length = max(step_length, grid_info.resolution)
        self._angle_bins = max(16, angle_quantization_bins)
        self._bin_size = 2.0 * math.pi / float(self._angle_bins)
        self._allow_reverse = allow_reverse
        self._min_turning_radius = max(0.05, minimum_turning_radius)
        self._allow_primitive_interpolation = allow_primitive_interpolation
        self._collision_radius = collision_radius
        self._heuristic_weight = heuristic_weight
        self._reverse_penalty = max(1.0, reverse_penalty)
        self._non_straight_penalty = max(1.0, non_straight_penalty)
        self._direction_change_penalty = max(0.0, direction_change_penalty)
        self._steering_change_penalty = max(0.0, steering_change_penalty)
        self._treat_unknown_as_occupied = bool(treat_unknown_as_occupied)

        self._primitives = self._build_motion_primitives()

    def _build_motion_primitives(self) -> List[MotionPrimitive]:
        primitives: List[MotionPrimitive] = []
        chord = max(self._step_length, math.sqrt(2.0) * self._grid_info.resolution)
        ratio = max(-1.0, min(1.0, chord / (2.0 * self._min_turning_radius)))
        min_heading_change = 2.0 * math.asin(ratio)
        increments = max(1, int(math.ceil(min_heading_change / self._bin_size)))
        turn_increments = [increments]
        if self._allow_primitive_interpolation and increments > 1:
            turn_increments = list(range(1, increments + 1))

        # Straight primitive.
        primitives.append(MotionPrimitive(distance=self._step_length, delta_yaw=0.0, reverse=False))
        if self._allow_reverse:
            primitives.append(MotionPrimitive(distance=self._step_length, delta_yaw=0.0, reverse=True))

        # Arc primitives using quantized heading deltas.
        for k in turn_increments:
            delta = k * self._bin_size
            arc_length = self._min_turning_radius * abs(delta)
            arc_length = max(arc_length, self._step_length)
            primitives.append(MotionPrimitive(distance=arc_length, delta_yaw=delta, reverse=False))
            primitives.append(MotionPrimitive(distance=arc_length, delta_yaw=-delta, reverse=False))
            if self._allow_reverse:
                primitives.append(MotionPrimitive(distance=arc_length, delta_yaw=delta, reverse=True))
                primitives.append(MotionPrimitive(distance=arc_length, delta_yaw=-delta, reverse=True))
        return primitives

    def _pose_to_grid(self, pose: Pose2D) -> GridIndex:
        return self._map.world_to_grid(pose.x, pose.y)

    def _is_goal_reached(self, pose: Pose2D, goal: Pose2D, pos_tol: float, yaw_tol: float) -> bool:
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        pos_ok = math.hypot(dx, dy) <= pos_tol
        yaw_diff = abs(self._normalize_angle(goal.yaw - pose.yaw))
        yaw_ok = yaw_diff <= yaw_tol
        return pos_ok and yaw_ok

    @staticmethod
    def _normalize_angle(theta: float) -> float:
        return math.atan2(math.sin(theta), math.cos(theta))

    def _collision_free(self, x: float, y: float) -> bool:
        idx = self._map.world_to_grid(x, y)
        if not self._map.in_bounds(idx):
            return False

        # Sample a small neighborhood based on collision radius
        cell_radius = int(math.ceil(self._collision_radius / self._grid_info.resolution))
        for dy in range(-cell_radius, cell_radius + 1):
            for dx in range(-cell_radius, cell_radius + 1):
                nidx = GridIndex(ix=idx.ix + dx, iy=idx.iy + dy)
                if not self._map.in_bounds(nidx):
                    return False
                if self._map.is_occupied(
                    nidx, treat_unknown_as_occupied=self._treat_unknown_as_occupied
                ):
                    return False
        return True

    def _primitive_rollout(
        self, state: SearchState, primitive: MotionPrimitive
    ) -> tuple[float, float, float, float]:
        direction = -1.0 if primitive.reverse else 1.0
        distance_signed = primitive.distance * direction
        delta_yaw_signed = primitive.delta_yaw * direction

        if abs(delta_yaw_signed) < 1e-9:
            dx_body = distance_signed
            dy_body = 0.0
        else:
            radius = distance_signed / delta_yaw_signed
            dx_body = radius * math.sin(delta_yaw_signed)
            dy_body = radius * (1.0 - math.cos(delta_yaw_signed))

        c = math.cos(state.yaw)
        s = math.sin(state.yaw)
        new_x = state.x + c * dx_body - s * dy_body
        new_y = state.y + s * dx_body + c * dy_body
        new_yaw = self._normalize_angle(state.yaw + delta_yaw_signed)
        return new_x, new_y, new_yaw, abs(distance_signed)

    def _segment_collision_free(
        self, state: SearchState, primitive: MotionPrimitive, sample_ds: float = 0.2
    ) -> bool:
        samples = max(1, int(math.ceil(primitive.distance / max(sample_ds, 1e-3))))
        for i in range(1, samples + 1):
            frac = i / float(samples)
            partial = MotionPrimitive(
                distance=primitive.distance * frac,
                delta_yaw=primitive.delta_yaw * frac,
                reverse=primitive.reverse,
            )
            x, y, _, _ = self._primitive_rollout(state, partial)
            if not self._collision_free(x, y):
                return False
        return True

    def _heuristic(self, pose: Pose2D, goal: Pose2D) -> float:
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        distance = math.hypot(dx, dy)
        yaw_diff = abs(self._normalize_angle(goal.yaw - pose.yaw))
        turning_cost = self._min_turning_radius * yaw_diff
        return self._heuristic_weight * (distance + turning_cost)

    def _expand(
        self,
        state: SearchState,
        goal: Pose2D,
        pos_tol: float,
        yaw_tol: float,
    ) -> Iterable[Tuple[SearchState, Pose2D]]:
        for primitive_index, primitive in enumerate(self._primitives):
            if not self._segment_collision_free(state, primitive):
                continue

            direction = -1 if primitive.reverse else 1
            new_x, new_y, new_yaw, travel_distance = self._primitive_rollout(state, primitive)
            grid_index = self._map.world_to_grid(new_x, new_y)

            primitive_cost = travel_distance
            if abs(primitive.delta_yaw) > 1e-6:
                primitive_cost *= self._non_straight_penalty
            if primitive.reverse:
                primitive_cost *= self._reverse_penalty
            if state.direction != direction:
                primitive_cost += self._direction_change_penalty
            if state.primitive_index >= 0:
                prev = self._primitives[state.primitive_index]
                steering_change = abs(prev.delta_yaw - primitive.delta_yaw)
                primitive_cost += self._steering_change_penalty * steering_change

            pose = Pose2D(x=new_x, y=new_y, yaw=new_yaw)
            is_goal = self._is_goal_reached(pose, goal, pos_tol, yaw_tol)

            new_state = SearchState(
                x=new_x,
                y=new_y,
                yaw=new_yaw,
                direction=direction,
                cost_so_far=state.cost_so_far + primitive_cost,
                heuristic=self._heuristic(pose, goal),
                parent=None,  # set later
                grid_index=grid_index,
                primitive_index=primitive_index,
            )
            yield new_state, pose, is_goal

    def plan(
        self,
        start: Pose2D,
        goal: Pose2D,
        *,
        position_tolerance: float = 0.5,
        yaw_tolerance: float = math.radians(10.0),
        max_iterations: int = 50000,
    ) -> Optional[PlanResult]:
        """
        Run the Hybrid A* search.
        """
        if not self._collision_free(start.x, start.y):
            return None
        if not self._collision_free(goal.x, goal.y):
            return None

        open_heap: List[Tuple[float, int]] = []
        states: List[SearchState] = []
        came_from: List[Optional[int]] = []
        visited_cost: dict[Tuple[int, int, int], float] = {}
        expanded_poses: List[Pose2D] = []

        start_idx = self._pose_to_grid(start)
        start_state = SearchState(
            x=start.x,
            y=start.y,
            yaw=start.yaw,
            direction=1,
            cost_so_far=0.0,
            heuristic=self._heuristic(start, goal),
            parent=None,
            grid_index=start_idx,
            primitive_index=-1,
        )
        states.append(start_state)
        came_from.append(None)

        heapq.heappush(open_heap, (start_state.total_score, 0))

        def discrete_key(s: SearchState) -> Tuple[int, int, int]:
            yaw_norm = (self._normalize_angle(s.yaw) + 2.0 * math.pi) % (2.0 * math.pi)
            yaw_bin = int(round(yaw_norm / self._bin_size)) % self._angle_bins
            dir_bin = 1 if s.direction > 0 else 0
            return s.grid_index.ix, s.grid_index.iy, yaw_bin * 2 + dir_bin

        visited_cost[discrete_key(start_state)] = 0.0

        goal_state_index: Optional[int] = None

        iterations = 0
        while open_heap and iterations < max_iterations:
            iterations += 1
            _, current_index = heapq.heappop(open_heap)
            current_state = states[current_index]

            current_pose = Pose2D(current_state.x, current_state.y, current_state.yaw)
            expanded_poses.append(current_pose)

            if self._is_goal_reached(current_pose, goal, position_tolerance, yaw_tolerance):
                goal_state_index = current_index
                break

            for next_state, next_pose, maybe_goal in self._expand(
                current_state, goal, position_tolerance, yaw_tolerance
            ):
                key = discrete_key(next_state)
                best_cost = visited_cost.get(key)
                if best_cost is not None and next_state.cost_so_far >= best_cost:
                    continue

                visited_cost[key] = next_state.cost_so_far

                next_state.parent = current_index
                state_index = len(states)
                states.append(next_state)
                came_from.append(current_index)

                heapq.heappush(open_heap, (next_state.total_score, state_index))

                if maybe_goal:
                    goal_state_index = state_index
                    open_heap = []
                    break

        if goal_state_index is None:
            return None

        # Reconstruct path
        path: List[Pose2D] = []
        idx = goal_state_index
        while idx is not None:
            s = states[idx]
            path.append(Pose2D(x=s.x, y=s.y, yaw=s.yaw))
            idx = came_from[idx]

        path.reverse()
        final_cost = states[goal_state_index].cost_so_far

        return PlanResult(path=path, expanded=expanded_poses, cost=final_cost)

