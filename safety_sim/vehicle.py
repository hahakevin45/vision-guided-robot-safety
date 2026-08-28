"""差速車 plant（ground truth）。

命令路徑對齊實車：Twist → twist_to_wheel_counts（含飽和與 int 量化）→
目標 counts/s。實際輪速以一階延遲趨近目標，模擬馬達/PID 的反應時間；
left/right_speed_scale 模擬打滑或左右不對稱。位姿以中點法積分 unicycle。
"""
from __future__ import annotations

import math

from vgr_core.motion import DiffDriveParams, twist_to_wheel_counts

from .types import Pose, Twist


class DiffDriveVehicle:
    def __init__(
        self,
        params: DiffDriveParams | None = None,
        *,
        pose: Pose = Pose(0.0, 0.0, 0.0),
        # Representative raised-wheel coast decay: tau≈0.08 s reaches <5%
        # roughly 0.25 s after STOP. Ground-contact dynamics require separate
        # validation.
        motor_time_constant_s: float = 0.08,
        left_speed_scale: float = 1.0,
        right_speed_scale: float = 1.0,
    ) -> None:
        self.params = params or DiffDriveParams()
        self._pose = pose
        self._tau = motor_time_constant_s
        self._left_scale = left_speed_scale
        self._right_scale = right_speed_scale
        self._target_cps = (0.0, 0.0)   # 命令端目標（已量化/飽和）
        self._actual_cps = (0.0, 0.0)   # 實際輪速（含打滑係數）

    # --- 命令端 ---

    def set_command(self, twist: Twist) -> None:
        left, right = twist_to_wheel_counts(twist.v, twist.omega, self.params)
        self._target_cps = (float(left), float(right))

    def stop(self) -> None:
        """板端 STOP：目標輪速歸零（馬達仍以一階延遲滑行到停）。"""
        self._target_cps = (0.0, 0.0)

    # --- 模擬推進 ---

    def step(self, dt: float) -> None:
        # 一階趨近：alpha = 1 - exp(-dt/tau)，tau=0 時瞬時到位。
        alpha = 1.0 if self._tau <= 0 else 1.0 - math.exp(-dt / self._tau)
        goal_l = self._target_cps[0] * self._left_scale
        goal_r = self._target_cps[1] * self._right_scale
        cur_l, cur_r = self._actual_cps
        cur_l += (goal_l - cur_l) * alpha
        cur_r += (goal_r - cur_r) * alpha
        self._actual_cps = (cur_l, cur_r)

        v, omega = self._twist_from_cps(cur_l, cur_r)
        # 中點法積分：以半步後的航向走這一步，減少直行時的數值偏移。
        mid_theta = self._pose.theta + omega * dt / 2.0
        self._pose = Pose(
            self._pose.x + v * math.cos(mid_theta) * dt,
            self._pose.y + v * math.sin(mid_theta) * dt,
            self._pose.theta + omega * dt,
        )

    # --- 狀態讀取 ---

    @property
    def pose(self) -> Pose:
        return self._pose

    @property
    def wheel_counts_per_s(self) -> tuple[float, float]:
        return self._actual_cps

    @property
    def twist_actual(self) -> Twist:
        v, omega = self._twist_from_cps(*self._actual_cps)
        return Twist(v, omega)

    def _twist_from_cps(self, left_cps: float, right_cps: float) -> tuple[float, float]:
        circumference = math.pi * self.params.wheel_diameter_m
        v_l = left_cps / self.params.left_counts_per_rev * circumference
        v_r = right_cps / self.params.right_counts_per_rev * circumference
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / self.params.wheel_base_m
        return v, omega
