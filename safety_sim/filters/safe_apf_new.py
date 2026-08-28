"""SAPF planner filter: runs the full Szczepanski 2023 field every authorized tick.

Unlike the pass-through correction filters, `/cmd_vel_nav` is motion
authorization and liveness only: its direction never substitutes for the
attractive target. The target is `Observation.goal` (fixed scenario goal or
Nav2 `/plan` lookahead supplied by the gate core). Every tick with fresh
pose, goal, and link, and with forward motion authorized, computes the full
field from geofence walls and static circles, then converts the gradient to
(v*, omega*) using the paper's Eq (10)-(11). Active output is MODIFIED;
any invalid input, goal reached, zero/reverse authorization, or singular
geometry fails closed with STOP. Reverse SAPF is outside the paper's method
and is not invented here.
"""
from __future__ import annotations

import math

from vgr_core.geometry.arena_geometry import Box2D

from ..sapf_field import (
    ObstacleSample,
    command_from_gradient,
    compute_analytic_gains,
    compute_sapf_field,
    wrap_angle,
)
from ..types import Observation, SafetyDecision, StaticInfo, Twist


def _geofence_walls(fence):
    """Yield (x1, y1, x2, y2, nx, ny) per edge with the inward unit normal.

    `fence` must be counter-clockwise; ARENA in safety_sim.scenarios.basic and
    vgr_core.geometry.ARENA both are. The inward normal is (-ey, ex) / |e|.
    """
    n = len(fence)
    for i in range(n):
        x1, y1 = fence[i]
        x2, y2 = fence[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        length = math.hypot(ex, ey)
        if length == 0.0:
            continue
        yield x1, y1, x2, y2, -ey / length, ex / length


class SafeApfNewFilter:
    name = "safe_apf_new"

    def __init__(
        self,
        *,
        d_safe_m: float = 0.28,
        d_vort_m: float = 0.40,
        Q_star_m: float = 0.80,
        d_g_star_m: float = 0.30,
        a_max_mps2: float = 0.50,
        theta_error_max_rad: float = math.pi / 4.0,
        # α_th = 120°：論文「α 是車頭與障礙的夾角、α_th 抑制接近零角時的
        # CW/CCW 切換」。徑向逼近（障礙在 goal 方向正前方，S8/GS3 幾何）時，
        # 小 α_th 會讓 D(α) 在 ±α_th 邊緣反覆翻轉、θ* 在 ±118° 間跳動，
        # 車鎖死在 shadow 點原地打轉（2026-08-09 實測 α_th=5..60° 全失敗）。
        # α_th=120° 覆蓋整個前半球，D(α) 只在障礙位於車後時翻轉，車才能
        # 連續轉向並沿障礙繞行——這正是 α_th 參數的用途，不是改公式。
        alpha_th_deg: float = 120.0,
        k_omega: float = 1.5,
        goal_tolerance_m: float = 0.05,
        pose_age_limit_s: float = 0.40,
        goal_age_limit_s: float = 0.50,
        link_age_limit_s: float = 0.50,
        # 實驗參數（Batch 1）：忽略位姿漂移 / 固定安全半徑。
        ignore_pose_drift: bool = False,
        fixed_d_safe_m: float | None = None,
    ) -> None:
        if not (
            math.isfinite(d_safe_m)
            and math.isfinite(d_vort_m)
            and math.isfinite(Q_star_m)
            and math.isfinite(d_g_star_m)
            and math.isfinite(a_max_mps2)
            and math.isfinite(theta_error_max_rad)
            and math.isfinite(alpha_th_deg)
            and math.isfinite(k_omega)
            and math.isfinite(goal_tolerance_m)
        ):
            raise ValueError("SAPF parameters must be finite")
        if not (d_safe_m > 0.0 and d_vort_m > d_safe_m):
            raise ValueError("require 0 < d_safe < d_vort")
        if 2.0 * d_vort_m - d_safe_m > Q_star_m:
            raise ValueError("require 2*d_vort - d_safe <= Q*")
        if a_max_mps2 <= 0.0 or k_omega <= 0.0 or goal_tolerance_m <= 0.0:
            raise ValueError("a_max, k_omega, and goal tolerance must be positive")
        if fixed_d_safe_m is not None and fixed_d_safe_m <= 0.0:
            raise ValueError("fixed_d_safe_m must be positive when provided")
        if not (0.0 < theta_error_max_rad < math.pi):
            raise ValueError("theta_error_max must be in (0, pi)")
        self._d_safe = d_safe_m
        self._d_vort = d_vort_m
        self._Q_star = Q_star_m
        self._d_g_star = d_g_star_m
        self._a_max = a_max_mps2
        self._theta_error_max = theta_error_max_rad
        self._alpha_th = math.radians(alpha_th_deg)
        self._k_omega = k_omega
        self._goal_tolerance = goal_tolerance_m
        self._pose_age_limit = pose_age_limit_s
        self._goal_age_limit = goal_age_limit_s
        self._link_age_limit = link_age_limit_s
        self._static: StaticInfo | None = None
        self._zeta = 0.0
        self._eta = 0.0
        self._ignore_pose_drift = ignore_pose_drift
        self._fixed_d_safe = fixed_d_safe_m

    def reset(self, static_info: StaticInfo) -> None:
        v_max = static_info.max_v_mps
        a_max = self._a_max
        if self._d_g_star <= v_max * v_max / (2.0 * a_max):
            raise ValueError(
                "d_g_star must exceed the stopping distance v_max^2/(2*a_max)"
            )
        if self._fixed_d_safe is not None:
            # 實驗固定半徑：d_safe=0.77 會破壞 0<d_safe<d_vort<=Q* 不變式，
            # 故同步拉開 d_vort 與 Q*（實驗專用）。
            self._d_safe = self._fixed_d_safe
            self._d_vort = self._fixed_d_safe + 0.12
            self._Q_star = 2.0 * self._d_vort - self._d_safe + 0.5
        self._zeta, self._eta = compute_analytic_gains(
            d_g_star=self._d_g_star, a_max=a_max, v_max=v_max,
            d_safe=self._d_safe, Q_star=self._Q_star,
        )
        self._static = static_info

    def filter(self, desired: Twist, obs: Observation,
               t: float, dt: float) -> SafetyDecision:
        st = self._static
        if st is None:
            return SafetyDecision(Twist.stop(), "STOP", {"reason": "not_reset"})
        pose = obs.pose
        if pose is None or not (
            math.isfinite(pose.x) and math.isfinite(pose.y) and math.isfinite(pose.theta)
        ):
            return SafetyDecision(Twist.stop(), "STOP", {"reason": "missing_pose"})
        if obs.pose_age_s > self._pose_age_limit:
            return SafetyDecision(Twist.stop(), "STOP", {"reason": "stale_pose"})
        if obs.link_age_s > self._link_age_limit:
            return SafetyDecision(Twist.stop(), "STOP", {"reason": "stale_link"})
        goal = obs.goal
        if goal is None or not (
            math.isfinite(goal[0]) and math.isfinite(goal[1])
        ):
            return SafetyDecision(Twist.stop(), "STOP", {"reason": "missing_goal"})
        if obs.goal_age_s > self._goal_age_limit:
            return SafetyDecision(Twist.stop(), "STOP", {"reason": "stale_goal"})
        if desired.v < 0.0:
            return SafetyDecision(Twist.stop(), "STOP",
                                  {"reason": "unsupported_reverse"})
        # B 版：SAPF 是自治局部規劃器（論文式），cmd_vel 不再授權——
        # desired.v==0 不停止；任務活性由 goal 生命週期表達
        # （missing/stale goal、goal_reached）。
        if math.hypot(pose.x - goal[0], pose.y - goal[1]) <= self._goal_tolerance:
            return SafetyDecision(Twist.stop(), "STOP", {"reason": "goal_reached"})

        drift = 0.0 if self._ignore_pose_drift else max(min(obs.pose_drift_m, 0.5), 0.0)
        # 實驗 2（a/c 臂）：位姿信心度膨脹必須同時撐開場參數。
        # 舊行為只膨脹 STOP 閾值（effective=dist-drift），場的
        # Q*/zeta/eta 固定——mu=0.03 頂箱實測車在 effective≈d_safe 處
        # 只受中等斥力、場平衡貼箱（clearance 0.2 < footprint 0.23）。
        # 這裡把 drift 加進 d_safe/d_vort/Q* 並重算解析增益，盲走越深
        # 斥力場越早生效。fixed_d_safe（d 臂）固定不膨脹。
        if self._fixed_d_safe is not None:
            d_safe, d_vort = self._d_safe, self._d_vort
            q_star, zeta, eta = self._Q_star, self._zeta, self._eta
        else:
            d_safe = self._d_safe + drift
            d_vort = self._d_vort + drift
            q_star = 2.0 * d_vort - d_safe + 0.5
            zeta, eta = compute_analytic_gains(
                d_g_star=self._d_g_star, a_max=self._a_max,
                v_max=st.max_v_mps,
                d_safe=d_safe, Q_star=q_star,
            )
        # D(alpha) 的 α 以「goal 方向」為基準，不是車頭方向：vortex 場必須是
        # 位置函數（論文 Fig. 2 的 vector field map）。若 α 隨車頭轉動，徑向
        # 逼近障礙時 D 會在 ±α_th 邊緣反覆翻轉，θ* 在 ±100° 間跳動，車永遠
        # 轉不到切線狀態（2026-08-09 S8 實測鎖死）。goal-relative 下 D 穩定，
        # 車能連續轉向並沿障礙繞行，符合論文 Fig. 3 展示的行為。
        goal_dir = math.atan2(goal[1] - pose.y, goal[0] - pose.x)
        samples: list[ObstacleSample] = []
        for x1, y1, _x2, _y2, nx, ny in _geofence_walls(st.geofence):
            signed = nx * (pose.x - x1) + ny * (pose.y - y1)
            if signed <= 0.0:
                return SafetyDecision(Twist.stop(), "STOP",
                                      {"reason": "outside_geofence"})
            effective = signed - drift
            if effective <= 0.0:
                return SafetyDecision(Twist.stop(), "STOP",
                                      {"reason": "obstacle_too_close"})
            if effective <= q_star:
                # 牆上最近點方向（clamped projection），供 α 使用
                ex, ey = _x2 - x1, _y2 - y1
                length_sq = ex * ex + ey * ey
                t = 0.0 if length_sq == 0.0 else min(
                    1.0, max(0.0, ((pose.x - x1) * ex + (pose.y - y1) * ey) / length_sq)
                )
                cx, cy = x1 + t * ex, y1 + t * ey
                samples.append(ObstacleSample(
                    effective, nx, ny,
                    wrap_angle(math.atan2(cy - pose.y, cx - pose.x) - goal_dir),
                ))
        for circle in obs.obstacles:
            if isinstance(circle, Box2D):
                if circle.contains(pose.x, pose.y):
                    return SafetyDecision(Twist.stop(), "STOP",
                                          {"reason": "inside_obstacle"})
                min_x, max_x, min_y, max_y = circle.bounds
                cx = min(max(pose.x, min_x), max_x)
                cy = min(max(pose.y, min_y), max_y)
                dx, dy = pose.x - cx, pose.y - cy
                dist = math.hypot(dx, dy)
                if dist <= 1e-9:
                    return SafetyDecision(Twist.stop(), "STOP",
                                          {"reason": "inside_obstacle"})
                effective = dist - drift
                if effective <= 0.0:
                    return SafetyDecision(Twist.stop(), "STOP",
                                          {"reason": "obstacle_too_close"})
                if effective <= q_star:
                    samples.append(ObstacleSample(
                        effective, dx / dist, dy / dist,
                        wrap_angle(math.atan2(cy - pose.y,
                                              cx - pose.x) - goal_dir),
                    ))
                continue
            dx, dy = pose.x - circle.x, pose.y - circle.y
            center_dist = math.hypot(dx, dy)
            if center_dist <= 1e-9:
                return SafetyDecision(Twist.stop(), "STOP",
                                      {"reason": "inside_obstacle"})
            effective = center_dist - circle.radius - drift
            if effective <= 0.0:
                return SafetyDecision(Twist.stop(), "STOP",
                                      {"reason": "obstacle_too_close"})
            if effective <= q_star:
                samples.append(ObstacleSample(
                    effective, dx / center_dist, dy / center_dist,
                    wrap_angle(math.atan2(circle.y - pose.y,
                                          circle.x - pose.x) - goal_dir),
                ))

        try:
            field = compute_sapf_field(
                pose.x, pose.y, goal, tuple(samples),
                d_g_star=self._d_g_star, zeta=zeta, Q_star=q_star,
                eta=eta, d_safe=d_safe, d_vort=d_vort,
                alpha_th=self._alpha_th,
            )
        except ValueError as exc:
            return SafetyDecision(Twist.stop(), "STOP",
                                  {"reason": f"invalid_geometry:{exc}"})
        if not (math.isfinite(field.gradient_x) and math.isfinite(field.gradient_y)):
            return SafetyDecision(Twist.stop(), "STOP", {"reason": "non_finite_field"})

        theta_star = math.atan2(-field.gradient_y, -field.gradient_x)
        v_star, omega_star = command_from_gradient(
            field.gradient_x, field.gradient_y,
            pose_theta=pose.theta,
            v_max=st.max_v_mps,
            omega_max=st.max_omega_rad_s,
            theta_error_max=self._theta_error_max,
            k_omega=self._k_omega,
        )
        cmd = Twist(v_star, omega_star)
        return SafetyDecision(cmd, "MODIFIED", {
            "goal_x": goal[0],
            "goal_y": goal[1],
            "gradient_x": field.gradient_x,
            "gradient_y": field.gradient_y,
            "min_obstacle_distance_m": field.min_obstacle_distance_m,
            "max_abs_gamma_rad": field.max_abs_gamma_rad,
            "zeta": zeta,
            "eta": eta,
            "d_safe_m": d_safe,
        })
