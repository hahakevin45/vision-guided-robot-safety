"""大場地實驗用的定位誤差模型（wrapper，不改核心 sensors.py）。

盲段語意：「信念-真實分岔」—— 盲段中信念位姿繼續按 odom 積分前進
（含誤差：偏差=盲走里程×24%、方向 per-seed 隨機、疊 σ2cm 噪聲），
真實位姿=實際運動。濾波器吃信念距離，碰撞用真實位姿判。

非盲段：視覺定位 fresh，pose_drift_m=0。
盲段：odom 信念更新（不凍結），pose_drift_m=0.10+0.30×盲走里程
（實車契約）。safe_apf 會讀它膨脹安全距離；cbf 天生忽略（實驗點）。
"""
from __future__ import annotations

import math
import random

from ..types import Pose


class FieldLocalizer:
    """實測誤差模型：非盲段視覺更新 + 盲段 odom 積分（信念-真實分岔）。

    介面比核心 ArucoLocalizer 多回傳一個 pose_drift_m（3-tuple），因此本
    實驗自帶 runner（run_e1e2.py）而不套用核心 runner。
    """

    def __init__(
        self,
        *,
        update_hz: float = 15.0,
        noise_xy_std: float = 0.02,
        noise_theta_std: float = 0.02,
        systematic_bias_m: float = 0.04,
        drift_rate_per_m: float = 0.24,
        seed: int = 0,
        blind_max_s: float = 60.0,
        blind_max_dist_m: float = 2.0,
    ) -> None:
        self._period = 1.0 / update_hz
        self._noise_xy = noise_xy_std
        self._noise_theta = noise_theta_std
        self._bias_m = systematic_bias_m
        self._drift_rate = drift_rate_per_m
        self._blind_max_s = blind_max_s
        self._blind_max_dist_m = blind_max_dist_m
        self._rng = random.Random(seed)
        # 系統偏差：固定量值、方向由 seed 決定，整段 run 不變。
        bias_dir = self._rng.uniform(-math.pi, math.pi)
        self._bias_dx = systematic_bias_m * math.cos(bias_dir)
        self._bias_dy = systematic_bias_m * math.sin(bias_dir)
        # 盲段 odom 誤差方向：per-seed 隨機（固定方向，整段 run 不變）。
        self._error_dir = self._rng.uniform(-math.pi, math.pi)

        self._last_fix: Pose | None = None
        self._last_fix_t: float = -math.inf
        # 盲段狀態
        self._blind_active: bool = False
        self._blind_anchor_est: Pose | None = None
        self._blind_anchor_true: Pose | None = None
        self._blind_path_m: float = 0.0
        self._prev_true: Pose | None = None

    def observe(
        self, true_pose: Pose, t: float, *, dropout: bool
    ) -> tuple[Pose | None, float, float]:
        """回傳 (估計位姿或 None, pose_age_s, pose_drift_m)。"""
        if not dropout:
            # 離開盲段：清除盲段累積狀態，回歸視覺定位。
            self._blind_active = False
            self._blind_anchor_est = None
            self._blind_anchor_true = None
            self._blind_path_m = 0.0

            due = t - self._last_fix_t >= self._period or self._last_fix is None
            if due:
                self._last_fix = Pose(
                    true_pose.x + self._bias_dx + self._rng.gauss(0.0, self._noise_xy),
                    true_pose.y + self._bias_dy + self._rng.gauss(0.0, self._noise_xy),
                    true_pose.theta + self._rng.gauss(0.0, self._noise_theta),
                )
                self._last_fix_t = t
            self._prev_true = true_pose

            if self._last_fix is None:
                return None, math.inf, math.inf
            return self._last_fix, t - self._last_fix_t, self._bias_m

        # ─── 盲段：信念-真實分岔 ───
        if not self._blind_active:
            # 剛進盲段：記錄錨點
            self._blind_active = True
            self._blind_anchor_est = self._last_fix
            self._blind_anchor_true = true_pose
            self._blind_path_m = 0.0

        # 累積本 tick 的 true 位移（計算盲走里程）
        if self._prev_true is not None:
            dx = true_pose.x - self._prev_true.x
            dy = true_pose.y - self._prev_true.y
            self._blind_path_m += math.hypot(dx, dy)
        self._prev_true = true_pose

        anchor_est = self._blind_anchor_est
        anchor_true = self._blind_anchor_true
        if anchor_est is None:
            return None, math.inf, math.inf

        # 信念位姿：從錨點積分 true 位移 + 誤差（偏差=盲走里程×24%、方向固定）
        error_mag = self._blind_path_m * self._drift_rate
        error_x = error_mag * math.cos(self._error_dir) + self._rng.gauss(0.0, self._noise_xy)
        error_y = error_mag * math.sin(self._error_dir) + self._rng.gauss(0.0, self._noise_xy)

        true_dx = true_pose.x - anchor_true.x
        true_dy = true_pose.y - anchor_true.y

        belief = Pose(
            anchor_est.x + true_dx + error_x,
            anchor_est.y + true_dy + error_y,
            true_pose.theta,
        )

        # pose_drift_m：實車契約公式
        pose_drift_m = 0.10 + 0.30 * self._blind_path_m

        # 把信念更新為最新位置（供下次盲段外視覺更新參考）
        self._last_fix = belief
        self._last_fix_t = t

        return belief, 0.0, pose_drift_m
