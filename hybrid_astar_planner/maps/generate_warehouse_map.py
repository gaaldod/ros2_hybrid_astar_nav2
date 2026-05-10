#!/usr/bin/env python3
"""Regenerate warehouse_lightweight_map.pgm from the canonical warehouse layout.

The previous map was an empty 80x80 grid (no obstacles), which let the global
planner draw paths straight through what should be walls. This script re-emits
a PGM that matches the static models declared in
``hybrid_astar_planner/sim/worlds/warehouse_lightweight_gz.sdf`` (and the
identical layout in ``warehouse_lightweight.world``).

Layout:
    Outer walls : 22 x 22 m enclosure centred on (0, 0), wall thickness 0.3 m
    Shelf row 1 : centre (-3.0, -2.5), size 1.2 x 7.0 m
    Shelf row 2 : centre (+2.0, +1.5), size 1.2 x 7.0 m

Map metadata (matches warehouse_lightweight_map.yaml):
    image:      warehouse_lightweight_map.pgm
    resolution: 0.5 m / cell
    origin:     (-10.0, -10.0, 0.0)  -> bottom-left of the image
    size:       80 x 80 cells -> 40 x 40 m world extent

Run from any directory; the output is written next to this script.
"""
from __future__ import annotations

from pathlib import Path

WIDTH = 80
HEIGHT = 80
RESOLUTION = 0.5
ORIGIN_X = -10.0
ORIGIN_Y = -10.0

FREE = 254
OCCUPIED = 0

OUT_PATH = Path(__file__).resolve().parent / "warehouse_lightweight_map.pgm"


def world_to_col(world_x: float) -> int:
    return int(round((world_x - ORIGIN_X) / RESOLUTION))


def world_y_to_row(world_y: float) -> int:
    """Convert world y to PGM row index (row 0 = top of image)."""
    row_from_bottom = (world_y - ORIGIN_Y) / RESOLUTION
    return int(round(HEIGHT - 1 - row_from_bottom))


def fill_box(grid: list[list[int]], cx: float, cy: float, sx: float, sy: float, value: int = OCCUPIED) -> None:
    """Mark every cell whose centre falls inside the world-frame AABB."""
    x_min = cx - sx / 2.0
    x_max = cx + sx / 2.0
    y_min = cy - sy / 2.0
    y_max = cy + sy / 2.0

    col_min = max(0, world_to_col(x_min))
    col_max = min(WIDTH - 1, world_to_col(x_max))
    # Note: world y_max -> small PGM row, world y_min -> large PGM row
    row_top = max(0, world_y_to_row(y_max))
    row_bot = min(HEIGHT - 1, world_y_to_row(y_min))

    for r in range(row_top, row_bot + 1):
        for c in range(col_min, col_max + 1):
            grid[r][c] = value


def main() -> None:
    grid = [[FREE for _ in range(WIDTH)] for _ in range(HEIGHT)]

    # Outer walls (centre-pose, size in xy from the SDF)
    fill_box(grid, cx=0.0, cy=+10.0, sx=22.0, sy=0.3)   # wall_north
    fill_box(grid, cx=0.0, cy=-10.0, sx=22.0, sy=0.3)   # wall_south
    fill_box(grid, cx=-10.0, cy=0.0, sx=0.3, sy=20.0)   # wall_west
    fill_box(grid, cx=+10.0, cy=0.0, sx=0.3, sy=20.0)   # wall_east

    # Interior shelves
    fill_box(grid, cx=-3.0, cy=-2.5, sx=1.2, sy=7.0)    # shelf_row_1
    fill_box(grid, cx=+2.0, cy=+1.5, sx=1.2, sy=7.0)    # shelf_row_2

    occ = sum(1 for row in grid for v in row if v == OCCUPIED)
    free = WIDTH * HEIGHT - occ

    with OUT_PATH.open("w", encoding="latin-1") as f:
        f.write("P2\n")
        f.write("# warehouse_lightweight_map (regenerated from sim/worlds/warehouse_lightweight_gz.sdf)\n")
        f.write(f"{WIDTH} {HEIGHT}\n")
        f.write("255\n")
        for row in grid:
            f.write(" ".join(str(v) for v in row) + "\n")

    print(f"Wrote {OUT_PATH}")
    print(f"  size: {WIDTH} x {HEIGHT}, resolution: {RESOLUTION} m/cell")
    print(f"  occupied cells: {occ}  ({occ * RESOLUTION ** 2:.2f} m^2)")
    print(f"  free cells:     {free}  ({free * RESOLUTION ** 2:.2f} m^2)")


if __name__ == "__main__":
    main()
