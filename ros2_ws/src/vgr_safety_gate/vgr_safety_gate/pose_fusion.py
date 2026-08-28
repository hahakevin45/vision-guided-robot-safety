"""PoseFuser: continuous odometry backbone with timestamped visual corrections.

Design rules:
- odom 連續積分是位姿主幹；視覺只產生 map→odom 修正量 T_corr，
  使 map 位姿 = T_corr ∘ odom 位姿。
- 視覺量測用**它自己的擷取時間戳**對齊 odom 歷史（線性內插），
  不用到達時間——延遲被時間對齊吸收，不會變成位置誤差。
- 一致性門：候選修正與現行修正套在同一 odom 位姿上，差超過門檻
  就拒收（平面 PnP 翻解等單幀離群自然被濾掉）；連續拒收達
  reloc_after_rejects 筆視為真的重定位（車被搬動），無條件接受。
- 純 Python、無 ROS/numpy 依賴，可直接單元測試。
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class FusedPose:
    x: float
    y: float
    yaw: float
    drift_m: float        # 自上次被接受的視覺修正以來 odom 走的路徑長
    corr_age_s: float     # now_s - 上次被接受視覺量測的 stamp_s


class PoseFuser:
    def __init__(
        self,
        *,
        gate_dist_m: float = 0.15,
        gate_yaw_rad: float = 0.175,
        blend: float = 0.3,
        buffer_s: float = 2.0,
        reloc_after_rejects: int = 20,
        max_future_s: float = 0.2,
    ) -> None:
        self._gate_dist_m = gate_dist_m
        self._gate_yaw_rad = gate_yaw_rad
        self._blend = blend
        self._buffer_s = buffer_s
        self._reloc_after_rejects = reloc_after_rejects
        self._max_future_s = max_future_s
        self._odom: deque[tuple[float, float, float, float]] = deque()  # (t,x,y,yaw)
        self._corr: tuple[float, float, float] | None = None  # (tx, ty, dyaw)
        self._dist_since_corr = 0.0
        self._consec_rejects = 0
        self._last_vision_stamp_s: float | None = None

    # ---------- odom 主幹 ----------

    def update_odom(self, x: float, y: float, yaw: float, stamp_s: float) -> None:
        if self._odom and stamp_s < self._odom[-1][0]:
            return  # 亂序樣本忽略
        if self._odom:
            _, px, py, _ = self._odom[-1]
            self._dist_since_corr += math.hypot(x - px, y - py)
        self._odom.append((stamp_s, x, y, yaw))
        cutoff = stamp_s - self._buffer_s
        while len(self._odom) > 2 and self._odom[0][0] < cutoff:
            self._odom.popleft()

    def _odom_at(self, stamp_s: float) -> tuple[float, float, float] | None:
        """odom 歷史在 stamp_s 的線性內插；超出範圍回 None。"""
        if len(self._odom) < 2:
            return None
        if stamp_s < self._odom[0][0]:
            return None
        newest = self._odom[-1]
        if stamp_s >= newest[0]:
            if stamp_s - newest[0] > self._max_future_s:
                return None
            return (newest[1], newest[2], newest[3])
        lo = None
        for i in range(len(self._odom) - 1, 0, -1):
            if self._odom[i - 1][0] <= stamp_s <= self._odom[i][0]:
                lo, hi = self._odom[i - 1], self._odom[i]
                break
        if lo is None:
            return None
        span = hi[0] - lo[0]
        f = 0.0 if span <= 0.0 else (stamp_s - lo[0]) / span
        yaw = _wrap(lo[3] + f * _wrap(hi[3] - lo[3]))
        return (lo[1] + f * (hi[1] - lo[1]), lo[2] + f * (hi[2] - lo[2]), yaw)

    # ---------- 視覺修正 ----------

    @staticmethod
    def _solve_corr(vision: tuple[float, float, float],
                    odom: tuple[float, float, float]) -> tuple[float, float, float]:
        """解 T_corr 使 vision = T_corr ∘ odom。"""
        vx, vy, vyaw = vision
        ox, oy, oyaw = odom
        dyaw = _wrap(vyaw - oyaw)
        c, s = math.cos(dyaw), math.sin(dyaw)
        return (vx - (c * ox - s * oy), vy - (s * ox + c * oy), dyaw)

    @staticmethod
    def _apply(corr: tuple[float, float, float],
               odom: tuple[float, float, float]) -> tuple[float, float, float]:
        tx, ty, dyaw = corr
        ox, oy, oyaw = odom
        c, s = math.cos(dyaw), math.sin(dyaw)
        return (tx + c * ox - s * oy, ty + s * ox + c * oy, _wrap(dyaw + oyaw))

    def update_vision(self, x: float, y: float, yaw: float, stamp_s: float) -> bool:
        odom_then = self._odom_at(stamp_s)
        if odom_then is None:
            return False
        cand = self._solve_corr((x, y, yaw), odom_then)
        if self._corr is not None:
            a = self._apply(cand, odom_then)
            b = self._apply(self._corr, odom_then)
            dist = math.hypot(a[0] - b[0], a[1] - b[1])
            dyaw = abs(_wrap(a[2] - b[2]))
            if dist > self._gate_dist_m or dyaw > self._gate_yaw_rad:
                self._consec_rejects += 1
                if self._consec_rejects < self._reloc_after_rejects:
                    return False
                # 連續大量不一致：世界真的變了（重定位），接受。
        if self._corr is None:
            self._corr = cand
        else:
            tx, ty, dyaw_c = self._corr
            b = self._blend
            self._corr = (
                tx + b * (cand[0] - tx),
                ty + b * (cand[1] - ty),
                _wrap(dyaw_c + b * _wrap(cand[2] - dyaw_c)),
            )
        self._dist_since_corr = 0.0
        self._consec_rejects = 0
        self._last_vision_stamp_s = stamp_s
        return True

    # ---------- 輸出 ----------

    def estimate(self, now_s: float) -> FusedPose | None:
        if self._corr is None or not self._odom:
            return None
        newest = self._odom[-1]
        x, y, yaw = self._apply(self._corr, (newest[1], newest[2], newest[3]))
        corr_age = (
            now_s - self._last_vision_stamp_s
            if self._last_vision_stamp_s is not None else math.inf
        )
        return FusedPose(x=x, y=y, yaw=yaw,
                         drift_m=self._dist_since_corr, corr_age_s=corr_age)
