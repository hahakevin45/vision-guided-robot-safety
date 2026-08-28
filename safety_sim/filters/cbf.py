"""簡化版 diff-drive CBF 安全濾波器。理論出處與簡化見 cbf_notes.md。

結構：watchdog（同 clamp_watchdog 的 pose/link age 檢查）→ 速度限幅 →
CBF 投影。CBF 對每條 geofence 邊與每個圓障礙建立 barrier h(x) ≥ 0，
以 look-ahead 點把非完整約束（diff-drive 不能側移）化成對 u=(v, ω)
線性的約束 ḣ ≥ -α·h，然後把名目命令逐約束投影到可行半平面；
投影失敗（可行集為空）就退回 STOP。純解析解，不需要 QP solver。
"""
from __future__ import annotations

import math

from vgr_core.geometry.arena_geometry import Box2D, box_edges

from ..types import Observation, Pose, SafetyDecision, StaticInfo, Twist


class CbfFilter:
    name = "cbf"

    def __init__(
        self,
        *,
        alpha: float = 1.0,              # class-K 增益：v ≤ α·h 的煞車陡峭度
        lookahead_m: float = 0.10,       # 非完整化約束用的前視點距離
        buffer_m: float = 0.08,          # 安全裕度（吸收馬達延遲與模型誤差）
        pose_age_limit_s: float = 0.5,
        link_age_limit_s: float = 0.5,
        max_iterations: int = 30,
    ) -> None:
        self._alpha = alpha
        self._lookahead = lookahead_m
        self._buffer = buffer_m
        self._pose_age_limit = pose_age_limit_s
        self._link_age_limit = link_age_limit_s
        self._max_iter = max_iterations
        self._static: StaticInfo | None = None

    def reset(self, static_info: StaticInfo) -> None:
        self._static = static_info

    def filter(self, desired: Twist, obs: Observation,
               t: float, dt: float) -> SafetyDecision:
        static = self._static
        assert static is not None, "reset() must be called before filter()"
        debug: dict[str, float] = {"pose_age_s": obs.pose_age_s,
                                   "link_age_s": obs.link_age_s}

        # 1) watchdog：沒有可信位姿/鏈路就不該動。
        if (obs.pose is None
                or obs.pose_age_s > self._pose_age_limit
                or obs.link_age_s > self._link_age_limit):
            return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)

        # 2) 限幅。
        v = max(-static.max_v_mps, min(static.max_v_mps, desired.v))
        omega = max(-static.max_omega_rad_s, min(static.max_omega_rad_s, desired.omega))

        # 3) CBF 投影。
        constraints = self._build_constraints(obs.pose, obs, static)
        if constraints:
            debug["min_h"] = min(h for _, _, h in constraints)
        u = self._project(v, omega, constraints, static)
        if u is None:
            return SafetyDecision(cmd=Twist.stop(), mode="STOP", debug=debug)
        v, omega = u

        modified = not (math.isclose(v, desired.v) and math.isclose(omega, desired.omega))
        return SafetyDecision(cmd=Twist(v, omega),
                              mode="MODIFIED" if modified else "PASS",
                              debug=debug)

    # --- 內部 ---

    def _build_constraints(self, pose: Pose, obs: Observation,
                           static: StaticInfo) -> list[tuple[float, float, float]]:
        """回傳 (a_v, a_omega, h)：約束為 a_v·v + a_omega·ω ≥ -α·h。"""
        margin = static.robot_radius_m + self._buffer
        cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
        # look-ahead 點及其速度：ṗ = v·e_theta + l·ω·e_perp
        px = pose.x + self._lookahead * cos_t
        py = pose.y + self._lookahead * sin_t

        constraints: list[tuple[float, float, float]] = []

        def add(nx: float, ny: float, h: float) -> None:
            a_v = nx * cos_t + ny * sin_t
            a_w = self._lookahead * (-nx * sin_t + ny * cos_t)
            constraints.append((a_v, a_w, h))

        fence = static.geofence
        if fence:
            n = len(fence)
            for i in range(n):
                x1, y1 = fence[i]
                x2, y2 = fence[(i + 1) % n]
                ex, ey = x2 - x1, y2 - y1
                length = math.hypot(ex, ey)
                if length == 0.0:
                    continue
                # CCW 多邊形的內向法線在邊方向的左側。
                nx, ny = -ey / length, ex / length
                h = nx * (px - x1) + ny * (py - y1) - margin
                add(nx, ny, h)

        for ob in obs.obstacles:
            if isinstance(ob, Box2D):
                if ob.contains(px, py):
                    # 車在箱內：不可行約束（a_v=a_w=0 使投影失敗 → STOP）
                    constraints.append((0.0, 0.0, -math.inf))
                    continue
                for x1, y1, x2, y2, _nx, _ny in box_edges(ob):
                    ex, ey = x2 - x1, y2 - y1
                    length_sq = ex * ex + ey * ey
                    t = 0.0 if length_sq == 0.0 else min(
                        1.0, max(0.0, ((px - x1) * ex + (py - y1) * ey)
                                 / length_sq))
                    cx, cy = x1 + t * ex, y1 + t * ey
                    dist = math.hypot(px - cx, py - cy)
                    if dist < 1e-9:
                        constraints.append((0.0, 0.0, -math.inf))
                        continue
                    h = dist - margin
                    add((px - cx) / dist, (py - cy) / dist, h)
                continue
            dx, dy = px - ob.x, py - ob.y
            dist = math.hypot(dx, dy)
            if dist < 1e-9:
                continue
            h = dist - ob.radius - margin
            add(dx / dist, dy / dist, h)

        return constraints

    def _project(self, v: float, omega: float,
                 constraints: list[tuple[float, float, float]],
                 static: StaticInfo) -> tuple[float, float] | None:
        """把 (v, ω) 逐約束投影進可行集；失敗回 None（呼叫端 STOP）。"""
        for _ in range(self._max_iter):
            worst = None
            worst_violation = -1e-9
            for a_v, a_w, h in constraints:
                violation = -(self._alpha * h) - (a_v * v + a_w * omega)
                if violation > worst_violation:
                    worst_violation = violation
                    worst = (a_v, a_w, h)
            if worst is None:
                break
            a_v, a_w, h = worst
            norm_sq = a_v * a_v + a_w * a_w
            if norm_sq < 1e-12:
                return None
            step = worst_violation / norm_sq
            v += a_v * step
            omega += a_w * step

        # 收尾：夾回速度盒內，再檢查一次可行性（夾制可能重新違反）。
        v = max(-static.max_v_mps, min(static.max_v_mps, v))
        omega = max(-static.max_omega_rad_s, min(static.max_omega_rad_s, omega))
        for a_v, a_w, h in constraints:
            if a_v * v + a_w * omega < -(self._alpha * h) - 1e-6:
                return None
        return v, omega
