from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

from .grid_map import GridIndex, OccupancyGridMap
from .types import GridInfo, PlanResult, Pose2D


@dataclass(frozen=True)
class MotionPrimitive:
    step: float
    steering: float
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
        step_length: float = 0.5,
        steering_angle_deg: float = 30.0,
        steering_samples: int = 3,
        allow_reverse: bool = True,
        min_turning_radius: float = 2.0,
        collision_radius: float = 0.35,
        heuristic_weight: float = 1.2,
    ) -> None:
        self._grid_info = grid_info
        self._map = OccupancyGridMap(grid_info, occupancy)
        self._step_length = step_length
        self._steering_angle = math.radians(steering_angle_deg)
        self._steering_samples = max(1, steering_samples)
        self._allow_reverse = allow_reverse
        self._min_turning_radius = min_turning_radius
        self._collision_radius = collision_radius
        self._heuristic_weight = heuristic_weight

        self._primitives = self._build_motion_primitives()

    def _build_motion_primitives(self) -> List[MotionPrimitive]:
        primitives: List[MotionPrimitive] = []
        if self._steering_samples == 1:
            steering_values = [0.0]
        else:
            steering_values = []
            for i in range(self._steering_samples):
                t = i / (self._steering_samples - 1)
                angle = -self._steering_angle + 2.0 * self._steering_angle * t
                steering_values.append(angle)

        directions = [1]
        if self._allow_reverse:
            directions.append(-1)

        for d in directions:
            for angle in steering_values:
                primitives.append(MotionPrimitive(step=self._step_length, steering=angle, reverse=(d < 0)))
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
                if self._map.is_occupied(nidx, treat_unknown_as_occupied=True):
                    return False
        return True

    def _heuristic(self, pose: Pose2D, goal: Pose2D) -> float:
        # Admissible if heuristic_weight == 1.0, slightly inflated otherwise.
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        distance = math.hypot(dx, dy)

        yaw_diff = abs(self._normalize_angle(goal.yaw - pose.yaw))

        # Approximate extra cost due to turning and orientation
        turning_penalty = self._min_turning_radius * yaw_diff
        return self._heuristic_weight * (distance + 0.1 * turning_penalty)

    def _expand(
        self,
        state: SearchState,
        goal: Pose2D,
        pos_tol: float,
        yaw_tol: float,
    ) -> Iterable[Tuple[SearchState, Pose2D]]:
        for primitive in self._primitives:
            direction = -1 if primitive.reverse else 1

            # Bicycle-model like forward integration
            step = primitive.step * direction
            yaw_change = step * math.tan(primitive.steering) / max(self._min_turning_radius, 1e-3)
            new_yaw = self._normalize_angle(state.yaw + yaw_change)
            new_x = state.x + step * math.cos(new_yaw)
            new_y = state.y + step * math.sin(new_yaw)

            if not self._collision_free(new_x, new_y):
                continue

            grid_index = self._map.world_to_grid(new_x, new_y)

            # Accumulate cost; reverse and steering incur mild penalties
            translation_cost = abs(step)
            steering_cost = 0.1 * abs(primitive.steering)
            reverse_cost = 0.5 if primitive.reverse else 0.0
            delta_cost = translation_cost + steering_cost + reverse_cost

            pose = Pose2D(x=new_x, y=new_y, yaw=new_yaw)
            is_goal = self._is_goal_reached(pose, goal, pos_tol, yaw_tol)

            new_state = SearchState(
                x=new_x,
                y=new_y,
                yaw=new_yaw,
                direction=direction,
                cost_so_far=state.cost_so_far + delta_cost,
                heuristic=self._heuristic(pose, goal),
                parent=None,  # set later
                grid_index=grid_index,
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
        )
        states.append(start_state)
        came_from.append(None)

        heapq.heappush(open_heap, (start_state.total_score, 0))

        def discrete_key(s: SearchState) -> Tuple[int, int, int]:
            yaw_bin = int(round(self._normalize_angle(s.yaw) / math.radians(10.0)))
            return s.grid_index.ix, s.grid_index.iy, yaw_bin

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

