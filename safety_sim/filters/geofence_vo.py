"""簡化版 geofence velocity-obstacle safety filter.

牆段視為靜態障礙（v_O=0），機器人用外接圓半徑加 margin 膨脹牆段。
每個候選差速命令在 tau 時間窗內 rollout；若 rollout 軌跡碰到膨脹牆段
就視為落在 VO 內。desired 不在 VO 內時原樣通過；否則在目前輪速附近的
可達命令集合中選最近的安全候選，沒有安全候選時用碰撞時間做降級。
"""
from __future__ import annotations

import math

from ..types import Observation, Pose, SafetyDecision, StaticInfo, Twist


class GeofenceVoFilter:
    name = "geofence_vo"

    def __init__(
        self,
        *,
        tau_s: float = 2.0,
        margin_m: float = 0.05,
        a_max_mps2: float = 0.5,
        alpha_max_rad_s2: float = 4.0,
        dt_sim_s: float = 0.05,
        gamma_v: int = 13,
        gamma_omega: int = 13,
        pose_age_limit_s: float = 0.5,
        link_age_limit_s: float = 0.5,
    ) -> None:
        self._tau = tau_s
        self._margin = margin_m
        self._a_max = a_max_mps2
        self._alpha_max = alpha_max_rad_s2
        self._dt_sim = dt_sim_s
        self._gamma_v = min(13, max(2, gamma_v))
        self._gamma_omega = min(13, max(2, gamma_omega))
        self._pose_age_limit = pose_age_limit_s
        self._link_age_limit = link_age_limit_s
        self._static: StaticInfo | None = None

    def reset(self, static_info: StaticInfo) -> None:
        self._static = static_info

    def filter(self, desired: Twist, obs: Observation,
               t: float, dt: float) -> SafetyDecision:
        static = self._static
        assert static is not None, "reset() must be called before filter()"
        debug: dict[str, float] = {"pose_age_s": obs.pose_age_s,
                                   "link_age_s": obs.link_age_s}

        if (obs.pose is None
                or obs.pose_age_s > self._pose_age_limit
                or obs.link_age_s > self._link_age_limit):
            return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)

        desired = self._clamp_twist(desired, static)
        current = self._twist_from_wheel_feedback(obs.wheel_feedback, static)
        debug["current_v"] = current.v
        debug["current_omega"] = current.omega

        desired_tc = self._collision_time(obs.pose, desired, static)
        debug["desired_tc_s"] = min(desired_tc, self._tau)
        if math.isinf(desired_tc):
            return SafetyDecision(cmd=desired, mode="PASS", debug=debug)

        v_window = self._window(current.v, static.max_v_mps, self._a_max, dt)
        omega_window = self._window(
            current.omega, static.max_omega_rad_s, self._alpha_max, dt
        )

        best_safe: tuple[float, Twist] | None = None
        best_unsafe: tuple[float, float, Twist] | None = None
        for candidate in self._candidate_twists(v_window, omega_window, desired):
            tc = self._collision_time(obs.pose, candidate, static)
            distance = math.hypot(candidate.v - desired.v,
                                  candidate.omega - desired.omega)
            if math.isinf(tc):
                if best_safe is None or distance < best_safe[0]:
                    best_safe = (distance, candidate)
                continue
            # 降級用：先最大化碰撞時間，再讓速度偏小、命令接近 desired。
            unsafe_key = (tc, -abs(candidate.v), -distance)
            if best_unsafe is None or unsafe_key > best_unsafe[:3]:
                best_unsafe = (*unsafe_key, candidate)

        if best_safe is not None:
            cmd = best_safe[1]
            debug["selected_tc_s"] = self._tau
            modified = not (math.isclose(cmd.v, desired.v)
                            and math.isclose(cmd.omega, desired.omega))
            return SafetyDecision(cmd=cmd, mode="MODIFIED" if modified else "PASS",
                                  debug=debug)

        if best_unsafe is None or best_unsafe[0] < 0.5:
            debug["selected_tc_s"] = 0.0 if best_unsafe is None else best_unsafe[0]
            return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)

        cmd = self._slow_down(best_unsafe[3], static)
        debug["selected_tc_s"] = best_unsafe[0]
        return SafetyDecision(cmd=cmd, mode="MODIFIED", debug=debug)

    def _clamp_twist(self, twist: Twist, static: StaticInfo) -> Twist:
        return Twist(
            max(-static.max_v_mps, min(static.max_v_mps, twist.v)),
            max(-static.max_omega_rad_s, min(static.max_omega_rad_s, twist.omega)),
        )

    def _window(
        self, current: float, limit: float, rate_limit: float, dt: float
    ) -> tuple[float, float]:
        delta = rate_limit * max(dt, 0.0)
        return max(-limit, current - delta), min(limit, current + delta)

    def _candidate_twists(
        self,
        v_window: tuple[float, float],
        omega_window: tuple[float, float],
        desired: Twist,
    ) -> list[Twist]:
        vs = self._linspace(v_window[0], v_window[1], self._gamma_v)
        omegas = self._linspace(omega_window[0], omega_window[1], self._gamma_omega)
        candidates = [Twist(v, omega) for v in vs for omega in omegas]
        for extra in (Twist.stop(), desired):
            if (v_window[0] - 1e-9 <= extra.v <= v_window[1] + 1e-9
                    and omega_window[0] - 1e-9 <= extra.omega <= omega_window[1] + 1e-9):
                candidates.append(extra)
        return candidates

    def _linspace(self, lo: float, hi: float, count: int) -> list[float]:
        if count <= 1 or math.isclose(lo, hi):
            return [(lo + hi) / 2.0]
        step = (hi - lo) / (count - 1)
        return [lo + i * step for i in range(count)]

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
        return self._clamp_twist(Twist(v, omega), static)

    def _collision_time(self, pose: Pose, twist: Twist,
                        static: StaticInfo) -> float:
        if not static.geofence:
            return math.inf
        segments = self._nearby_segments(pose, static)
        if not segments:
            return math.inf

        radius = static.robot_radius_m + self._margin
        steps = max(1, math.ceil(self._tau / self._dt_sim))
        sim_pose = pose
        for step in range(steps + 1):
            elapsed = min(step * self._dt_sim, self._tau)
            if self._touches_any_segment(sim_pose, segments, radius):
                return elapsed
            if step < steps:
                sim_pose = self._integrate(sim_pose, twist, self._dt_sim)
        return math.inf

    def _nearby_segments(
        self, pose: Pose, static: StaticInfo
    ) -> list[tuple[float, float, float, float]]:
        fence = static.geofence
        radius = static.robot_radius_m + self._margin
        neighborhood = static.max_v_mps * self._tau + radius
        segments: list[tuple[float, float, float, float]] = []
        n = len(fence)
        for i in range(n):
            x1, y1 = fence[i]
            x2, y2 = fence[(i + 1) % n]
            if self._point_segment_distance(pose.x, pose.y, x1, y1, x2, y2) <= neighborhood:
                segments.append((x1, y1, x2, y2))
        return segments

    def _touches_any_segment(
        self,
        pose: Pose,
        segments: list[tuple[float, float, float, float]],
        radius: float,
    ) -> bool:
        for x1, y1, x2, y2 in segments:
            if self._point_segment_distance(pose.x, pose.y, x1, y1, x2, y2) <= radius:
                return True
        return False

    def _integrate(self, pose: Pose, twist: Twist, dt: float) -> Pose:
        mid_theta = pose.theta + twist.omega * dt / 2.0
        return Pose(
            pose.x + twist.v * math.cos(mid_theta) * dt,
            pose.y + twist.v * math.sin(mid_theta) * dt,
            pose.theta + twist.omega * dt,
        )

    def _point_segment_distance(
        self, px: float, py: float, x1: float, y1: float, x2: float, y2: float
    ) -> float:
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            return math.hypot(px - x1, py - y1)
        u = ((px - x1) * dx + (py - y1) * dy) / length_sq
        u = max(0.0, min(1.0, u))
        return math.hypot(px - (x1 + u * dx), py - (y1 + u * dy))

    def _slow_down(self, twist: Twist, static: StaticInfo) -> Twist:
        v = 0.5 * twist.v
        omega = twist.omega
        if abs(v) < 0.02:
            v = 0.0
        return self._clamp_twist(Twist(v, omega), static)
