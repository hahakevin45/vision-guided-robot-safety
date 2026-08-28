"""Input-constrained CBF variant with stopping-distance-aware barriers.

This keeps the same halfspace projection machinery as ``CbfFilter`` but builds
barriers with a conservative braking-distance term computed from encoder
feedback.  The filter still uses only ``Observation`` and static map data.
"""
from __future__ import annotations

import math

from vgr_core.geometry.arena_geometry import Box2D, box_edges

from .cbf import CbfFilter
from ..types import Observation, Pose, StaticInfo, Twist


class IccbfFilter(CbfFilter):
    name = "iccbf"

    def __init__(
        self,
        *,
        alpha: float = 2.0,
        a_max_mps2: float = 0.4,
        lookahead_m: float = 0.10,
        margin_m: float = 0.05,
        pose_age_limit_s: float = 0.5,
        link_age_limit_s: float = 0.5,
        max_iterations: int = 30,
    ) -> None:
        super().__init__(
            alpha=alpha,
            lookahead_m=lookahead_m,
            buffer_m=margin_m,
            pose_age_limit_s=pose_age_limit_s,
            link_age_limit_s=link_age_limit_s,
            max_iterations=max_iterations,
        )
        self._a_max = a_max_mps2

    def _build_constraints(
        self, pose: Pose, obs: Observation, static: StaticInfo
    ) -> list[tuple[float, float, float]]:
        """回傳 (a_v, a_omega, h)：h 扣掉朝障礙煞停距離。"""
        margin = static.robot_radius_m + self._buffer
        cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
        center_x, center_y = pose.x, pose.y
        ahead_x = pose.x + self._lookahead * cos_t
        ahead_y = pose.y + self._lookahead * sin_t
        current = self._twist_from_wheel_feedback(obs.wheel_feedback, static)

        constraints: list[tuple[float, float, float]] = []

        def add(nx: float, ny: float, raw_h: float, a_w: float) -> None:
            a_v = nx * cos_t + ny * sin_t
            current_hdot = a_v * current.v + a_w * current.omega
            v_toward = max(0.0, -current_hdot)
            braking_distance = v_toward * v_toward / (2.0 * self._a_max)
            constraints.append((a_v, a_w, raw_h - braking_distance))

        def add_wall(nx: float, ny: float, x1: float, y1: float) -> None:
            center_h = nx * (center_x - x1) + ny * (center_y - y1) - margin
            ahead_h = nx * (ahead_x - x1) + ny * (ahead_y - y1) - margin
            if ahead_h < center_h:
                a_w = self._lookahead * (-nx * sin_t + ny * cos_t)
                add(nx, ny, ahead_h, a_w)
            else:
                add(nx, ny, center_h, 0.0)

        fence = static.geofence
        if fence:
            n = len(fence)
            for i in range(n):
                x1, y1 = fence[i]
                x2, y2 = fence[(i + 1) % n]
                ex, ey = x2 - x1, y2 - y1
                length = math.hypot(ex, ey)
                if length == 0.0:
                    continue
                nx, ny = -ey / length, ex / length
                add_wall(nx, ny, x1, y1)

        for ob in obs.obstacles:
            if isinstance(ob, Box2D):
                for x1, y1, x2, y2, _nx, _ny in box_edges(ob):
                    ex, ey = x2 - x1, y2 - y1
                    length_sq = ex * ex + ey * ey
                    t = 0.0 if length_sq == 0.0 else min(
                        1.0, max(0.0, ((center_x - x1) * ex
                                        + (center_y - y1) * ey) / length_sq))
                    cx, cy = x1 + t * ex, y1 + t * ey
                    center_dist = math.hypot(center_x - cx, center_y - cy)
                    ahead_dist = math.hypot(ahead_x - cx, ahead_y - cy)
                    if center_dist < 1e-9 and ahead_dist < 1e-9:
                        continue
                    if ahead_dist < center_dist and ahead_dist >= 1e-9:
                        raw_h = ahead_dist - margin
                        nx, ny = (ahead_x - cx) / ahead_dist, (ahead_y - cy) / ahead_dist
                        a_w = self._lookahead * (-nx * sin_t + ny * cos_t)
                        add(nx, ny, raw_h, a_w)
                    elif center_dist >= 1e-9:
                        raw_h = center_dist - margin
                        add((center_x - cx) / center_dist,
                            (center_y - cy) / center_dist, raw_h, 0.0)
                continue
            center_dx, center_dy = center_x - ob.x, center_y - ob.y
            center_dist = math.hypot(center_dx, center_dy)
            ahead_dx, ahead_dy = ahead_x - ob.x, ahead_y - ob.y
            ahead_dist = math.hypot(ahead_dx, ahead_dy)
            if center_dist < 1e-9 and ahead_dist < 1e-9:
                continue
            if ahead_dist < center_dist and ahead_dist >= 1e-9:
                raw_h = ahead_dist - ob.radius - margin
                nx, ny = ahead_dx / ahead_dist, ahead_dy / ahead_dist
                a_w = self._lookahead * (-nx * sin_t + ny * cos_t)
                add(nx, ny, raw_h, a_w)
            elif center_dist >= 1e-9:
                raw_h = center_dist - ob.radius - margin
                add(center_dx / center_dist, center_dy / center_dist, raw_h, 0.0)

        return constraints

    def _twist_from_wheel_feedback(
        self, wheel_feedback: tuple[float, float], static: StaticInfo
    ) -> Twist:
        left_cps, right_cps = wheel_feedback
        params = static.params
        circumference = math.pi * params.wheel_diameter_m
        v_l = left_cps / params.left_counts_per_rev * circumference
        v_r = right_cps / params.right_counts_per_rev * circumference
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / params.wheel_base_m
        return Twist(
            max(-static.max_v_mps, min(static.max_v_mps, v)),
            max(-static.max_omega_rad_s, min(static.max_omega_rad_s, omega)),
        )
