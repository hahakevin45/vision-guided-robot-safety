"""名目命令來源（Nav 端）。安全層的輸入，不是被測物。

情境用它模擬正常導航、也模擬失控導航（超速、突然反向、高頻振盪）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Protocol

from .types import Observation, Twist


class NavSource(Protocol):
    def command(self, obs: Observation, t: float) -> Twist: ...


@dataclass(frozen=True)
class ScriptedNav:
    """分段常值命令：[(t_start, Twist), ...]，取 t_start <= t 的最後一段。"""

    segments: tuple[tuple[float, Twist], ...]

    def command(self, obs: Observation, t: float) -> Twist:
        current = Twist.stop()
        for t_start, twist in self.segments:
            if t >= t_start:
                current = twist
        return current


@dataclass(frozen=True)
class WaypointNav:
    """P 控制器 waypoint 追蹤：先轉向、對準後前進，到點即停。

    只吃 Observation 裡的估計位姿——位姿凍結時它會被騙，這正是
    S2/S4 類情境要暴露的行為。沒有位姿時輸出停止。
    """

    goal: tuple[float, float]
    max_v_mps: float = 0.15
    k_heading: float = 2.0
    k_v: float = 0.5
    arrive_radius_m: float = 0.05

    def command(self, obs: Observation, t: float) -> Twist:
        if obs.pose is None:
            return Twist.stop()
        dx = self.goal[0] - obs.pose.x
        dy = self.goal[1] - obs.pose.y
        dist = math.hypot(dx, dy)
        if dist < self.arrive_radius_m:
            return Twist.stop()
        heading_err = _wrap_angle(math.atan2(dy, dx) - obs.pose.theta)
        v = min(self.max_v_mps, self.k_v * dist) * max(0.0, math.cos(heading_err))
        return Twist(v, self.k_heading * heading_err)


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class FunctionNav:
    """任意時間函數命令，給振盪/失控之類寫不成分段常值的情境。"""

    fn: Callable[[Observation, float], Twist]

    def command(self, obs: Observation, t: float) -> Twist:
        return self.fn(obs, t)
