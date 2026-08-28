"""host↔MCU 鏈路模型。

語意對齊 docs/protocol_v1.md 的責任分界：
- host 下行命令可能被丟（link_drop 故障）。
- 板端 watchdog：超過 timeout_s 沒收到任何命令 → 自動 STOP。
  這是「最後一道防線」，安全層不應依賴它，但模擬裡必須存在，
  才能看出 host 端安全層與板端 watchdog 誰先動作。
- host 端以 age_s() 得知距上次成功下行多久（進 Observation.link_age_s）。
"""
from __future__ import annotations

import math

from .types import Twist
from .vehicle import DiffDriveVehicle


class CommandLink:
    def __init__(self, *, timeout_s: float = 0.5) -> None:
        self._timeout = timeout_s
        self._pending: Twist | None = None
        self._last_delivered_t: float | None = None   # None = 從未下行（車仍閒置，鏈路視為正常）
        self._watchdog_fired = False

    def send(self, twist: Twist, t: float, *, dropped: bool) -> None:
        if not dropped:
            self._pending = twist
            self._last_delivered_t = t

    def poll(self, vehicle: DiffDriveVehicle, t: float) -> None:
        """把已送達的命令套到車上，並執行板端 watchdog。"""
        if self._pending is not None:
            vehicle.set_command(self._pending)
            self._pending = None
            self._watchdog_fired = False
        if (self._last_delivered_t is not None
                and t - self._last_delivered_t > self._timeout
                and not self._watchdog_fired):
            vehicle.stop()
            self._watchdog_fired = True

    def age_s(self, t: float) -> float:
        if self._last_delivered_t is None:
            return 0.0
        return t - self._last_delivered_t

    @property
    def watchdog_fired(self) -> bool:
        return self._watchdog_fired
