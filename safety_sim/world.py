"""2D 世界：geofence 多邊形 + 圓形障礙 + 碰撞/淨空判定。

這裡的判定吃 ground truth 位姿，只給 metrics 與情境判準用；
安全層看到的世界資訊一律走 Observation。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from vgr_core.geometry.arena_geometry import Box2D, box_distance_to_point

from .types import Circle, Pose


def _point_in_polygon(x: float, y: float, poly: tuple[tuple[float, float], ...]) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


def _distance_to_segment(x: float, y: float,
                         x1: float, y1: float, x2: float, y2: float) -> float:
    dx, dy = x2 - x1, y2 - y1
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(x - x1, y - y1)
    t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / length_sq))
    return math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))


@dataclass(frozen=True)
class World:
    geofence: tuple[tuple[float, float], ...]   # 多邊形頂點；空 tuple = 無界
    obstacles: tuple[Circle, ...]
    robot_radius_m: float
    goal: tuple[float, float] | None = None

    def contains(self, pose: Pose) -> bool:
        if not self.geofence:
            return True
        return _point_in_polygon(pose.x, pose.y, self.geofence)

    def min_clearance(self, pose: Pose) -> float:
        """車體外緣到最近邊界/障礙的距離；負值表示已侵入。

        出界時回傳負的（邊界距離 + 車半徑），讓 metrics 能量化違規深度。
        """
        clearance = math.inf
        if self.geofence:
            n = len(self.geofence)
            boundary = min(
                _distance_to_segment(pose.x, pose.y,
                                     *self.geofence[i], *self.geofence[(i + 1) % n])
                for i in range(n)
            )
            if self.contains(pose):
                clearance = boundary - self.robot_radius_m
            else:
                clearance = -(boundary + self.robot_radius_m)
        for ob in self.obstacles:
            if isinstance(ob, Box2D):
                d = box_distance_to_point(ob, pose.x, pose.y) - self.robot_radius_m
            else:
                d = (math.hypot(pose.x - ob.x, pose.y - ob.y)
                     - ob.radius - self.robot_radius_m)
            clearance = min(clearance, d)
        return clearance

    def collided(self, pose: Pose) -> bool:
        return self.min_clearance(pose) < 0.0
