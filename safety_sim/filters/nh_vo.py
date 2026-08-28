"""Nonholonomic velocity-obstacle filter for static geofence walls.

The paper method is adapted to safety_sim's available signals: walls are
static obstacles (v_o = 0), and the desired diff-drive command supplies the
nominal unicycle speed and heading direction.  Runtime decisions use only the
Observation pose freshness, link freshness, wheel feedback, and static map
information supplied through reset().
"""
from __future__ import annotations

import math

from ..types import Observation, Pose, SafetyDecision, StaticInfo, Twist


class NhVoFilter:
    name = "nh_vo"

    def __init__(
        self,
        *,
        margin_m: float = 0.09,
        influence_m: float = 0.55,
        psi_safe_rad: float = 0.08,
        delta_rad: float = 0.10,
        alpha: float = 1.0,
        k_theta: float = 2.2,
        low_speed_mps: float = 0.01,
        pose_age_limit_s: float = 0.5,
        link_age_limit_s: float = 0.5,
    ) -> None:
        self._margin = margin_m
        self._influence = influence_m
        self._psi_safe = psi_safe_rad
        self._delta = delta_rad
        self._alpha = alpha
        self._k_theta = k_theta
        self._low_speed = low_speed_mps
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

        raw_desired = desired
        desired = self._clamp_twist(desired, static)
        clamped = not (math.isclose(desired.v, raw_desired.v)
                       and math.isclose(desired.omega, raw_desired.omega))

        walls = self._wall_obstacles(obs.pose, static)
        if walls:
            debug["min_wall_d_m"] = min(distance for distance, _, _ in walls)

        if not walls or abs(desired.v) <= self._low_speed:
            return SafetyDecision(cmd=desired, mode="MODIFIED" if clamped else "PASS",
                                  debug=debug)

        signed_speed = 1.0 if desired.v >= 0.0 else -1.0
        travel_theta = obs.pose.theta if signed_speed > 0.0 else obs.pose.theta + math.pi
        travel_x, travel_y = math.cos(travel_theta), math.sin(travel_theta)
        d_min = static.robot_radius_m + self._margin

        speed_limit = abs(desired.v)
        target_theta = travel_theta
        target_error_abs = math.inf
        blocked = False

        for distance, rx, ry in walls:
            if distance > self._influence:
                continue
            if distance <= 1e-9:
                return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)

            axis_theta = math.atan2(ry, rx)
            axis_dot = travel_x * (rx / distance) + travel_y * (ry / distance)
            h = distance - d_min

            if distance <= d_min:
                if axis_dot <= 0.0:
                    continue
                return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)

            if axis_dot > 1e-9:
                speed_limit = min(speed_limit, max(0.0, self._alpha * h / axis_dot))

            ratio = self._clamp(d_min / distance, 1.0)
            cone_half = math.asin(ratio)
            safe_half = min(math.pi / 2.0 - 1e-6, cone_half + self._psi_safe)
            error = self._wrap(travel_theta - axis_theta)
            if abs(error) >= safe_half:
                continue

            blocked = True
            left_theta = axis_theta + safe_half + self._delta
            right_theta = axis_theta - safe_half - self._delta
            left_error = abs(self._wrap(left_theta - travel_theta))
            right_error = abs(self._wrap(right_theta - travel_theta))
            candidate_theta = left_theta if left_error <= right_error else right_theta
            candidate_error_abs = min(left_error, right_error)
            if candidate_error_abs < target_error_abs:
                target_theta = candidate_theta
                target_error_abs = candidate_error_abs

        if not blocked and speed_limit >= abs(desired.v) - 1e-9:
            return SafetyDecision(cmd=desired, mode="MODIFIED" if clamped else "PASS",
                                  debug=debug)

        if speed_limit <= self._low_speed:
            cmd = Twist(0.0, 0.0 if not blocked else desired.omega)
            if blocked:
                theta_error = self._wrap(target_theta - travel_theta)
                cmd = Twist(0.0, self._clamp(self._k_theta * theta_error,
                                             static.max_omega_rad_s))
        else:
            v_mag = min(abs(desired.v), speed_limit, static.max_v_mps)
            omega = desired.omega
            if blocked:
                theta_error = self._wrap(target_theta - travel_theta)
                omega = self._clamp(self._k_theta * theta_error,
                                    static.max_omega_rad_s)
                heading_scale = max(0.0, math.cos(theta_error))
                v_mag *= heading_scale
            cmd = self._clamp_twist(Twist(signed_speed * v_mag, omega), static)

        modified = clamped or not (math.isclose(cmd.v, desired.v)
                                   and math.isclose(cmd.omega, desired.omega))
        mode = "MODIFIED" if modified else "PASS"
        if math.isclose(cmd.v, 0.0, abs_tol=1e-12) and math.isclose(cmd.omega, 0.0, abs_tol=1e-12):
            mode = "STOP"
        return SafetyDecision(cmd=cmd, mode=mode, debug=debug)

    def _wall_obstacles(
        self, pose: Pose, static: StaticInfo
    ) -> list[tuple[float, float, float]]:
        fence = static.geofence
        if len(fence) < 3:
            return []

        walls: list[tuple[float, float, float]] = []
        n = len(fence)
        for i in range(n):
            x1, y1 = fence[i]
            x2, y2 = fence[(i + 1) % n]
            closest_x, closest_y = self._closest_point(pose.x, pose.y, x1, y1, x2, y2)
            rx, ry = closest_x - pose.x, closest_y - pose.y
            distance = math.hypot(rx, ry)
            if distance <= self._influence:
                walls.append((distance, rx, ry))
        return walls

    def _closest_point(
        self, px: float, py: float, x1: float, y1: float, x2: float, y2: float
    ) -> tuple[float, float]:
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            return x1, y1
        u = ((px - x1) * dx + (py - y1) * dy) / length_sq
        u = max(0.0, min(1.0, u))
        return x1 + u * dx, y1 + u * dy

    def _clamp_twist(self, twist: Twist, static: StaticInfo) -> Twist:
        return Twist(
            self._clamp(twist.v, static.max_v_mps),
            self._clamp(twist.omega, static.max_omega_rad_s),
        )

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _wrap(self, angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
