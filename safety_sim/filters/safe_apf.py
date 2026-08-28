"""Safe artificial-potential-field filter for geofence walls.

The original method is a local planner.  In safety_sim this is adapted as a
filter: the desired command supplies the attractive direction, while nearby
geofence walls supply repulsive inward normals.  All runtime state comes from
Observation; the geofence is static map information supplied at reset().
"""
from __future__ import annotations

import math

from ..types import Observation, Pose, SafetyDecision, StaticInfo, Twist


class SafeApfFilter:
    name = "safe_apf"

    def __init__(
        self,
        *,
        extra_safe_m: float = 0.05,
        drift_cap_m: float = 0.30,
        influence_m: float = 0.45,
        alpha: float = 1.0,
        k_theta: float = 2.0,
        theta_error_max_rad: float = math.pi / 2.0,
        pose_age_limit_s: float = 0.5,
        link_age_limit_s: float = 0.5,
    ) -> None:
        self._extra_safe = extra_safe_m
        self._drift_cap = drift_cap_m
        self._influence = influence_m
        self._alpha = alpha
        self._k_theta = k_theta
        self._theta_error_max = theta_error_max_rad
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
        walls = self._wall_distances(obs.pose, static)
        if walls:
            debug["min_wall_d_m"] = min(d for d, _, _ in walls)

        if not walls or math.isclose(desired.v, 0.0, abs_tol=1e-12):
            return SafetyDecision(cmd=desired, mode="MODIFIED" if clamped else "PASS",
                                  debug=debug)

        # 位姿不確定時安全距離等量放大：盲走越遠，離牆越遠就停。
        drift = min(max(getattr(obs, "pose_drift_m", 0.0), 0.0), self._drift_cap)
        d_safe = static.robot_radius_m + self._extra_safe + drift
        if drift > 0.0:
            debug["d_safe_eff_m"] = d_safe
        signed_speed = 1.0 if desired.v >= 0.0 else -1.0
        travel_theta = obs.pose.theta if signed_speed > 0.0 else obs.pose.theta + math.pi
        ux, uy = math.cos(travel_theta), math.sin(travel_theta)

        needs_apf = False
        speed_limit = abs(desired.v)
        for distance, nx, ny in walls:
            h = distance - d_safe
            closing = -(ux * nx + uy * ny)
            if distance <= d_safe or distance < self._influence:
                needs_apf = True
            if closing > 1e-9:
                allowed = max(0.0, self._alpha * h / closing)
                if abs(desired.v) > allowed + 1e-9:
                    speed_limit = min(speed_limit, allowed)
                    needs_apf = True

        if not needs_apf:
            return SafetyDecision(cmd=desired, mode="MODIFIED" if clamped else "PASS",
                                  debug=debug)

        cmd = self._apf_command(desired, obs.pose, walls, d_safe, signed_speed, static,
                                speed_limit)
        if cmd is None:
            return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)

        modified = clamped or not (math.isclose(cmd.v, desired.v)
                                   and math.isclose(cmd.omega, desired.omega))
        return SafetyDecision(cmd=cmd, mode="MODIFIED" if modified else "PASS",
                              debug=debug)

    def _apf_command(
        self,
        desired: Twist,
        pose: Pose,
        walls: list[tuple[float, float, float]],
        d_safe: float,
        signed_speed: float,
        static: StaticInfo,
        speed_limit: float,
    ) -> Twist | None:
        travel_theta = pose.theta if signed_speed > 0.0 else pose.theta + math.pi
        fx = abs(desired.v) * math.cos(travel_theta)
        fy = abs(desired.v) * math.sin(travel_theta)

        influence = max(self._influence, d_safe + 1e-6)
        for distance, nx, ny in walls:
            if distance >= influence:
                continue
            if distance <= d_safe:
                strength = static.max_v_mps
            else:
                rel = (influence - distance) / (influence - d_safe)
                strength = static.max_v_mps * rel * rel
            fx += strength * nx
            fy += strength * ny

        norm = math.hypot(fx, fy)
        if norm < 1e-9:
            return Twist.stop()

        target_theta = math.atan2(fy, fx)
        theta_error = self._wrap(target_theta - travel_theta)
        omega = self._clamp(self._k_theta * theta_error, static.max_omega_rad_s)

        abs_error = abs(theta_error)
        if abs_error >= self._theta_error_max:
            v_mag = 0.0
        else:
            scale = (self._theta_error_max - abs_error) / self._theta_error_max
            v_mag = min(abs(desired.v), max(0.0, speed_limit), norm, static.max_v_mps) * scale

        return self._clamp_twist(Twist(signed_speed * v_mag, omega), static)

    def _wall_distances(
        self, pose: Pose, static: StaticInfo
    ) -> list[tuple[float, float, float]]:
        fence = static.geofence
        if len(fence) < 3:
            return []
        ccw = self._signed_area(fence) >= 0.0
        walls: list[tuple[float, float, float]] = []
        n = len(fence)
        for i in range(n):
            x1, y1 = fence[i]
            x2, y2 = fence[(i + 1) % n]
            ex, ey = x2 - x1, y2 - y1
            length = math.hypot(ex, ey)
            if length <= 1e-12:
                continue
            if ccw:
                nx, ny = -ey / length, ex / length
            else:
                nx, ny = ey / length, -ex / length
            distance = nx * (pose.x - x1) + ny * (pose.y - y1)
            walls.append((distance, nx, ny))
        return walls

    def _clamp_twist(self, twist: Twist, static: StaticInfo) -> Twist:
        return Twist(
            self._clamp(twist.v, static.max_v_mps),
            self._clamp(twist.omega, static.max_omega_rad_s),
        )

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _wrap(self, angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _signed_area(self, poly: tuple[tuple[float, float], ...]) -> float:
        area = 0.0
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            area += x1 * y2 - x2 * y1
        return area / 2.0
