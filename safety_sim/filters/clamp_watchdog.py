"""基準 1：限幅 + 加速度 ramp + 位姿/鏈路 watchdog。

不含任何幾何或預測——不知道牆在哪裡。它代表「不用看論文就寫得出來」
的樸素安全層；任何論文方法至少要在比較表上贏過它，才值得上車。
"""
from __future__ import annotations

import math

from ..types import Observation, SafetyDecision, StaticInfo, Twist


class ClampWatchdogFilter:
    name = "clamp_watchdog"

    def __init__(
        self,
        *,
        max_accel_mps2: float = 0.5,
        pose_age_limit_s: float = 0.5,
        link_age_limit_s: float = 0.5,
    ) -> None:
        self._max_accel = max_accel_mps2
        self._pose_age_limit = pose_age_limit_s
        self._link_age_limit = link_age_limit_s
        self._max_v = math.inf
        self._max_omega = math.inf
        self._last_v = 0.0

    def reset(self, static_info: StaticInfo) -> None:
        self._max_v = static_info.max_v_mps
        self._max_omega = static_info.max_omega_rad_s
        self._last_v = 0.0

    def filter(self, desired: Twist, obs: Observation,
               t: float, dt: float) -> SafetyDecision:
        debug = {"pose_age_s": obs.pose_age_s, "link_age_s": obs.link_age_s}

        if (obs.pose is None
                or obs.pose_age_s > self._pose_age_limit
                or obs.link_age_s > self._link_age_limit):
            self._last_v = 0.0
            return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)

        v = max(-self._max_v, min(self._max_v, desired.v))
        omega = max(-self._max_omega, min(self._max_omega, desired.omega))

        # 加速度 ramp 只管 |v| 的上升（煞車不限），維持緊急停止能力。
        dv_limit = self._max_accel * dt
        if abs(v) > abs(self._last_v) + dv_limit:
            v = self._last_v + math.copysign(dv_limit, v - self._last_v)
        self._last_v = v

        modified = not (math.isclose(v, desired.v) and math.isclose(omega, desired.omega))
        return SafetyDecision(cmd=Twist(v, omega),
                              mode="MODIFIED" if modified else "PASS",
                              debug=debug)
