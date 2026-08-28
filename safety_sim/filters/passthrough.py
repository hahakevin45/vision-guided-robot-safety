"""基準 0：什麼都不做。

存在的目的：(1) 驗證情境本身真的危險（passthrough 必須撞），
(2) 活性指標的分母（完成時間倍率以它為 1.0）。
"""
from __future__ import annotations

from ..types import Observation, SafetyDecision, StaticInfo, Twist


class PassthroughFilter:
    name = "passthrough"

    def reset(self, static_info: StaticInfo) -> None:
        pass

    def filter(self, desired: Twist, obs: Observation,
               t: float, dt: float) -> SafetyDecision:
        return SafetyDecision(cmd=desired, mode="PASS")
