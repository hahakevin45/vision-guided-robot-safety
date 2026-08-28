"""簡化版 geofence DWA safety filter.

GF-DWA 在任務單中被當作安全 filter：每個 tick 從 encoder 回報估目前
(v, omega)，建立加速度可達的動態窗口，對候選速度做短 horizon rollout，
只要任一 rollout 點侵入 geofence 安全裕度就丟棄該候選。
"""
from __future__ import annotations

import math

from ..types import Observation, Pose, SafetyDecision, StaticInfo, Twist


class GfDwaFilter:
    name = "gf_dwa"
    _CREEP_V_EPS = 0.03
    _CREEP_OMEGA_EPS = 0.1
    _CREEP_MARGIN_EPS = 0.03

    def __init__(
        self,
        *,
        q_col: float = 0.1,
        q_ref: float = 1.0,
        margin_m: float = 0.05,
        a_max_mps2: float = 0.5,
        dt_sim_s: float = 0.1,
        rollout_steps: int = 10,
        gamma_v: int = 7,
        gamma_omega: int = 7,
        pose_age_limit_s: float = 0.5,
        link_age_limit_s: float = 0.5,
    ) -> None:
        self._q_col = q_col
        self._q_ref = q_ref
        self._margin = margin_m
        self._a_max = a_max_mps2
        self._dt_sim = dt_sim_s
        self._steps = min(10, max(1, rollout_steps))
        self._gamma_v = min(7, max(2, gamma_v))
        self._gamma_omega = min(7, max(2, gamma_omega))
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

        current = self._twist_from_wheel_feedback(obs.wheel_feedback, static)
        v_window = self._window(current.v, static.max_v_mps, dt)
        omega_window = self._window(current.omega, static.max_omega_rad_s, dt)
        debug["current_v"] = current.v
        debug["current_omega"] = current.omega

        if (self._in_window(desired.v, v_window)
                and self._in_window(desired.omega, omega_window)):
            desired_margin = self._rollout_min_margin(obs.pose, desired, static)
            debug["desired_min_margin_m"] = desired_margin
            if desired_margin >= 0.0:
                return SafetyDecision(cmd=desired, mode="PASS", debug=debug)

        best: tuple[float, Twist, float] | None = None
        for candidate in self._candidate_twists(v_window, omega_window, desired):
            min_margin = self._rollout_min_margin(obs.pose, candidate, static)
            if min_margin < 0.0:
                continue
            j_col = 0.0 if math.isinf(min_margin) else 1.0 / max(min_margin, 1e-9)
            j_ref = math.hypot(candidate.v - desired.v,
                               candidate.omega - desired.omega)
            cost = self._q_col * j_col + self._q_ref * j_ref
            if best is None or cost < best[0]:
                best = (cost, candidate, min_margin)

        if best is None:
            debug["min_margin_m"] = self._geofence_margin(obs.pose, static)
            return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)

        _, cmd, min_margin = best
        debug["min_margin_m"] = min_margin
        if (min_margin < self._CREEP_MARGIN_EPS
                and self._is_creep(cmd)
                and not self._is_creep(desired)):
            return SafetyDecision(cmd=Twist.stop(), mode="MODIFIED", debug=debug)
        modified = not (math.isclose(cmd.v, desired.v)
                        and math.isclose(cmd.omega, desired.omega))
        return SafetyDecision(cmd=cmd, mode="MODIFIED" if modified else "PASS",
                              debug=debug)

    def _window(self, current: float, limit: float, dt: float) -> tuple[float, float]:
        delta = self._a_max * max(dt, 0.0)
        return max(-limit, current - delta), min(limit, current + delta)

    def _in_window(self, value: float, window: tuple[float, float]) -> bool:
        lo, hi = window
        return lo - 1e-9 <= value <= hi + 1e-9

    def _is_creep(self, twist: Twist) -> bool:
        return (abs(twist.v) < self._CREEP_V_EPS
                and abs(twist.omega) < self._CREEP_OMEGA_EPS)

    def _candidate_twists(
        self,
        v_window: tuple[float, float],
        omega_window: tuple[float, float],
        desired: Twist,
    ) -> list[Twist]:
        vs = self._linspace(v_window[0], v_window[1], self._gamma_v)
        omegas = self._linspace(omega_window[0], omega_window[1], self._gamma_omega)
        candidates = [Twist(v, omega) for v in vs for omega in omegas]
        if (self._in_window(desired.v, v_window)
                and self._in_window(desired.omega, omega_window)):
            candidates.append(desired)
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
        return Twist(v, omega)

    def _rollout_min_margin(self, pose: Pose, cmd: Twist,
                            static: StaticInfo) -> float:
        sim_pose = pose
        min_margin = self._geofence_margin(sim_pose, static)
        if min_margin < 0.0:
            return min_margin
        for _ in range(self._steps):
            sim_pose = self._integrate(sim_pose, cmd, self._dt_sim)
            min_margin = min(min_margin, self._geofence_margin(sim_pose, static))
            if min_margin < 0.0:
                return min_margin
        return min_margin

    def _integrate(self, pose: Pose, twist: Twist, dt: float) -> Pose:
        mid_theta = pose.theta + twist.omega * dt / 2.0
        return Pose(
            pose.x + twist.v * math.cos(mid_theta) * dt,
            pose.y + twist.v * math.sin(mid_theta) * dt,
            pose.theta + twist.omega * dt,
        )

    def _geofence_margin(self, pose: Pose, static: StaticInfo) -> float:
        fence = static.geofence
        if not fence:
            return math.inf
        margin = static.robot_radius_m + self._margin
        signed_distances: list[float] = []
        n = len(fence)
        for i in range(n):
            x1, y1 = fence[i]
            x2, y2 = fence[(i + 1) % n]
            ex, ey = x2 - x1, y2 - y1
            length = math.hypot(ex, ey)
            if length == 0.0:
                continue
            nx, ny = -ey / length, ex / length
            signed_distances.append(nx * (pose.x - x1) + ny * (pose.y - y1) - margin)
        return min(signed_distances) if signed_distances else math.inf
