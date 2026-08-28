"""ArUco 定位模型：更新頻率、高斯噪聲、dropout。

行為對齊實車：marker 丟失時不會拿到「錯的位姿」，而是拿不到新位姿——
最後一筆凍結、pose_age 持續增長。安全層必須自己對 age 做反應。

預設 synthetic model 使用 15 Hz 更新、0.04 m 位置噪聲與 0.02 rad
角度噪聲。它保留 dropout/age 語意，但不模擬相機 FOV、marker 斜視角、
遮擋或特定實體場地的視野失效邊界。
"""
from __future__ import annotations

import math
import random

from .types import Pose


class ArucoLocalizer:
    def __init__(
        self,
        *,
        update_hz: float = 15.0,
        noise_xy_std: float = 0.04,
        noise_theta_std: float = 0.02,
        seed: int = 0,
    ) -> None:
        self._period = 1.0 / update_hz
        self._noise_xy = noise_xy_std
        self._noise_theta = noise_theta_std
        self._rng = random.Random(seed)
        self._last_fix: Pose | None = None
        self._last_fix_t: float = -math.inf

    def observe(self, true_pose: Pose, t: float, *, dropout: bool) -> tuple[Pose | None, float]:
        """回傳 (估計位姿或 None, pose_age_s)。"""
        due = t - self._last_fix_t >= self._period or self._last_fix is None
        if not dropout and due:
            self._last_fix = Pose(
                true_pose.x + self._rng.gauss(0.0, self._noise_xy),
                true_pose.y + self._rng.gauss(0.0, self._noise_xy),
                true_pose.theta + self._rng.gauss(0.0, self._noise_theta),
            )
            self._last_fix_t = t
        if self._last_fix is None:
            return None, math.inf
        return self._last_fix, t - self._last_fix_t
