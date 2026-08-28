"""故障排程：時間視窗式注入，作用在 sensors / link / nav / vehicle。

kind 目前使用的值：
- "aruco_dropout"：定位量測停止更新（marker 丟失）。
- "link_drop"：host 下行全部丟包。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FaultWindow:
    t0: float
    t1: float
    kind: str

    def active(self, t: float) -> bool:
        return self.t0 <= t < self.t1


@dataclass(frozen=True)
class FaultSchedule:
    windows: tuple[FaultWindow, ...] = ()

    def active(self, kind: str, t: float) -> bool:
        return any(w.kind == kind and w.active(t) for w in self.windows)
