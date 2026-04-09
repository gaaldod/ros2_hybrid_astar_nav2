from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class GridInfo:
    resolution: float
    origin_xy: Tuple[float, float]
    width: int
    height: int


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PlanResult:
    path: list[Pose2D]
    expanded: Optional[list[Pose2D]] = None
    cost: float = 0.0

