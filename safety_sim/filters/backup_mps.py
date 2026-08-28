"""Backup-policy MPS safety filter for diff-drive simulation.

Each tick validates one candidate trajectory: run the requested command for the
control interval, then run a braking backup policy long enough to stop.  The
filter only uses Observation pose freshness, wheel feedback, link age, and the
static geofence provided at reset.
"""
from __future__ import annotations

import math

from ..types import Observation, Pose, SafetyDecision, StaticInfo, Twist


class BackupMpsFilter:
    name = "backup_mps"

    def __init__(
        self,
        *,
        backup_horizon_s: float = 2.0,
        dt_sim_s: float = 0.05,
        margin_m: float = 0.05,
        pose_guard_margin_m: float = 0.05,
        max_decel_mps2: float = 0.5,
        pose_age_limit_s: float = 0.4,
        link_age_limit_s: float = 0.4,
    ) -> None:
        self._backup_horizon = backup_horizon_s
        self._dt_sim = dt_sim_s
        self._margin = margin_m
        self._pose_guard_margin = pose_guard_margin_m
        self._max_decel = max_decel_mps2
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

        cmd = self._clamp(desired, static)
        current = self._twist_from_wheel_feedback(obs.wheel_feedback, static)
        valid, min_margin = self._candidate_is_valid(obs.pose, cmd, current, dt, static)
        debug["min_margin_m"] = min_margin

        if valid:
            modified = not (math.isclose(cmd.v, desired.v)
                            and math.isclose(cmd.omega, desired.omega))
            return SafetyDecision(cmd=cmd, mode="MODIFIED" if modified else "PASS",
                                  debug=debug)

        backup = self._backup_step(current, dt, static)
        mode = "STOP" if math.isclose(backup.v, 0.0) and math.isclose(backup.omega, 0.0) else "MODIFIED"
        return SafetyDecision(cmd=backup, mode=mode, debug=debug)

    def _clamp(self, cmd: Twist, static: StaticInfo) -> Twist:
        return Twist(
            max(-static.max_v_mps, min(static.max_v_mps, cmd.v)),
            max(-static.max_omega_rad_s, min(static.max_omega_rad_s, cmd.omega)),
        )

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

    def _candidate_is_valid(
        self, pose: Pose, cmd: Twist, current: Twist, control_dt: float,
        static: StaticInfo
    ) -> tuple[bool, float]:
        sim_pose = pose
        min_margin = self._geofence_margin(sim_pose, static)
        if min_margin < 0.0:
            return False, min_margin

        for step_dt in self._step_durations(control_dt):
            sim_pose = self._integrate(sim_pose, cmd, step_dt)
            min_margin = min(min_margin, self._geofence_margin(sim_pose, static))
            if min_margin < 0.0:
                return False, min_margin

        # Be conservative about the speed that may need braking after accepting
        # the command: the plant can move toward cmd during the first interval.
        backup_v = self._more_dangerous_speed(current.v, cmd.v)
        backup_omega = self._more_dangerous_speed(current.omega, cmd.omega)
        backup_twist = Twist(backup_v, backup_omega)
        for step_dt in self._step_durations(self._backup_horizon):
            backup_twist = self._backup_step(backup_twist, step_dt, static)
            sim_pose = self._integrate(sim_pose, backup_twist, step_dt)
            min_margin = min(min_margin, self._geofence_margin(sim_pose, static))
            if min_margin < 0.0:
                return False, min_margin

        stopped = math.isclose(backup_twist.v, 0.0, abs_tol=1e-6) and math.isclose(
            backup_twist.omega, 0.0, abs_tol=1e-6)
        return stopped, min_margin

    def _step_durations(self, duration_s: float) -> list[float]:
        if duration_s <= 0.0:
            return []
        steps = int(math.ceil(duration_s / self._dt_sim))
        return [min(self._dt_sim, duration_s - i * self._dt_sim) for i in range(steps)]

    def _backup_step(self, current: Twist, dt: float, static: StaticInfo) -> Twist:
        v = self._decay_to_zero(current.v, self._max_decel * dt)
        omega_decel = self._max_decel / max(static.params.wheel_base_m / 2.0, 1e-9)
        omega = self._decay_to_zero(current.omega, omega_decel * dt)
        return self._clamp(Twist(v, omega), static)

    def _decay_to_zero(self, value: float, amount: float) -> float:
        if value > 0.0:
            return max(0.0, value - amount)
        if value < 0.0:
            return min(0.0, value + amount)
        return 0.0

    def _more_dangerous_speed(self, current: float, command: float) -> float:
        if current >= 0.0 and command >= 0.0:
            return max(current, command)
        if current <= 0.0 and command <= 0.0:
            return min(current, command)
        return command if abs(command) > abs(current) else current

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
        margin = static.robot_radius_m + self._margin + self._pose_guard_margin
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
