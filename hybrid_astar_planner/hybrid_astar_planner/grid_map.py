from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple

import math

from .types import GridInfo


@dataclass(frozen=True)
class GridIndex:
    ix: int
    iy: int


class OccupancyGridMap:
    """
    Minimal occupancy grid wrapper (0 free, 100 occupied, -1 unknown).
    """

    def __init__(self, info: GridInfo, data: list[int]) -> None:
        if len(data) != info.width * info.height:
            raise ValueError("Occupancy data size does not match width*height.")
        self._info = info
        self._data = data

    @property
    def info(self) -> GridInfo:
        return self._info

    def in_bounds(self, idx: GridIndex) -> bool:
        return 0 <= idx.ix < self._info.width and 0 <= idx.iy < self._info.height

    def cell_value(self, idx: GridIndex) -> int:
        return self._data[idx.iy * self._info.width + idx.ix]

    def is_occupied(self, idx: GridIndex, *, treat_unknown_as_occupied: bool = True) -> bool:
        v = self.cell_value(idx)
        if v < 0:
            return treat_unknown_as_occupied
        return v >= 50

    def world_to_grid(self, x: float, y: float) -> GridIndex:
        ox, oy = self._info.origin_xy
        ix = int(math.floor((x - ox) / self._info.resolution))
        iy = int(math.floor((y - oy) / self._info.resolution))
        return GridIndex(ix=ix, iy=iy)

    def grid_to_world_center(self, idx: GridIndex) -> Tuple[float, float]:
        ox, oy = self._info.origin_xy
        x = ox + (idx.ix + 0.5) * self._info.resolution
        y = oy + (idx.iy + 0.5) * self._info.resolution
        return x, y

    def iter_occupied_cells(self, *, treat_unknown_as_occupied: bool = True) -> Iterable[GridIndex]:
        for iy in range(self._info.height):
            base = iy * self._info.width
            for ix in range(self._info.width):
                v = self._data[base + ix]
                if v < 0 and treat_unknown_as_occupied:
                    yield GridIndex(ix=ix, iy=iy)
                elif v >= 50:
                    yield GridIndex(ix=ix, iy=iy)

