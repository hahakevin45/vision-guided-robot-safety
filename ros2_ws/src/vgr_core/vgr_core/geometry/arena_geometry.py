"""Authoritative geometry for the Nav2 Gazebo acceptance arena."""
from __future__ import annotations

from dataclasses import dataclass
import math


ARENA_BOUNDS = (0.0, 4.0, -1.0, 1.0)
MAP_RESOLUTION_M = 0.05
MAP_PADDING_M = 0.5


@dataclass(frozen=True)
class Box2D:
    x: float
    y: float
    size_x: float
    size_y: float

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (
            self.x - self.size_x / 2.0,
            self.x + self.size_x / 2.0,
            self.y - self.size_y / 2.0,
            self.y + self.size_y / 2.0,
        )

    def contains(self, x: float, y: float) -> bool:
        min_x, max_x, min_y, max_y = self.bounds
        return min_x <= x <= max_x and min_y <= y <= max_y


def box_edges(box: Box2D) -> tuple[tuple[float, float, float, float, float, float], ...]:
    """Box2D 的四條邊：(x1, y1, x2, y2, nx, ny)。

    法線為**外向**單位法線（指向箱外自由空間），供 filter 把矩形障礙
    當作四段牆處理（SAPF vortex 對每條邊各自旋轉、CBF 對每條邊建 barrier）。
    """
    min_x, max_x, min_y, max_y = box.bounds
    corners = (
        (min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y),
    )
    edges = []
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        ex, ey = x2 - x1, y2 - y1
        length = math.hypot(ex, ey)
        if length == 0.0:
            continue
        # CCW 頂點順序：外向法線 = 邊方向右轉 90°（(ey, -ex)）
        edges.append((x1, y1, x2, y2, ey / length, -ex / length))
    return tuple(edges)


def box_distance_to_point(box: Box2D, x: float, y: float) -> float:
    """車心到箱體表面距離（箱內為負）。"""
    min_x, max_x, min_y, max_y = box.bounds
    if box.contains(x, y):
        dx = min(x - min_x, max_x - x)
        dy = min(y - min_y, max_y - y)
        return -min(dx, dy)
    dx = max(min_x - x, 0.0, x - max_x)
    dy = max(min_y - y, 0.0, y - max_y)
    return math.hypot(dx, dy)


# Tall enough to block the straight path, with traversable corridors on both sides.
NAV_OBSTACLE = Box2D(x=2.0, y=0.0, size_x=0.40, size_y=0.60)


@dataclass(frozen=True)
class OccupancyGrid:
    width: int
    height: int
    resolution_m: float
    origin_x: float
    origin_y: float
    pixels: tuple[tuple[int, ...], ...]

    def is_occupied(self, x: float, y: float) -> bool:
        col = math.floor((x - self.origin_x) / self.resolution_m)
        row_from_bottom = math.floor((y - self.origin_y) / self.resolution_m)
        if not (0 <= col < self.width and 0 <= row_from_bottom < self.height):
            return True
        row = self.height - 1 - row_from_bottom
        return self.pixels[row][col] == 0

    def to_pgm(self) -> str:
        rows = (" ".join(str(value) for value in row) for row in self.pixels)
        return f"P2\n{self.width} {self.height}\n255\n" + "\n".join(rows) + "\n"


def build_occupancy_grid(
    *,
    resolution_m: float = MAP_RESOLUTION_M,
    padding_m: float = MAP_PADDING_M,
    include_obstacle: bool = True,
) -> OccupancyGrid:
    """產生場地佔據圖。

    `include_obstacle=False` 產生「隱藏障礙」map：NAV_OBSTACLE 不畫入，
    全局 planner 會規劃穿箱直線——隱藏牆比較場景（局部避障的測試場）。
    """
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")
    if padding_m < 0.0:
        raise ValueError("padding_m must be non-negative")

    min_x, max_x, min_y, max_y = ARENA_BOUNDS
    origin_x = min_x - padding_m
    origin_y = min_y - padding_m
    width = math.ceil((max_x - min_x + 2.0 * padding_m) / resolution_m)
    height = math.ceil((max_y - min_y + 2.0 * padding_m) / resolution_m)
    pixels: list[tuple[int, ...]] = []
    for row in range(height):
        y = origin_y + (height - row - 0.5) * resolution_m
        values: list[int] = []
        for col in range(width):
            x = origin_x + (col + 0.5) * resolution_m
            inside_arena = min_x < x < max_x and min_y < y < max_y
            occupied = not inside_arena or (
                include_obstacle and NAV_OBSTACLE.contains(x, y))
            values.append(0 if occupied else 254)
        pixels.append(tuple(values))
    return OccupancyGrid(
        width=width,
        height=height,
        resolution_m=resolution_m,
        origin_x=origin_x,
        origin_y=origin_y,
        pixels=tuple(pixels),
    )
# Polygon form of the arena boundary (clockwise from origin corner).
# Matches ARENA_BOUNDS = (0.0, 4.0, -1.0, 1.0).
ARENA = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))
