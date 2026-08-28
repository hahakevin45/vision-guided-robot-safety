"""Safety gate core: pure decision logic, no ROS/Gazebo dependencies.

The core accepts a safety_filter, maintains latest nav/aruco/odom state,
and produces GateOutput on each tick. No rclpy, no Gazebo.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from vgr_core.geometry import ARENA
from vgr_core.motion import DiffDriveParams
from vgr_core.safety import Observation, Pose, SafetyDecision, StaticInfo, Twist
from vgr_core.safety.path_target import select_path_lookahead


DEFAULT_ROBOT_RADIUS_M = 0.23  # matches safety_sim.scenario.DEFAULT_ROBOT_RADIUS_M


@dataclass(frozen=True)
class GateOutput:
    cmd: Twist
    mode: str
    debug: dict[str, Any]


class SafetyGateCore:
    """Safety gate 的純核心：保存最新 topic 狀態並在 tick 時呼叫 filter。

    盲走預算（dead-reckoning budget，2026-07-14 策略檢討）：ArUco 不可能
    時時可見（轉彎、視野邊緣），丟失視覺不再立即 STOP——改用「最後一次
    視覺錨點＋encoder odom 增量」推算位姿餵給 filter（odom 實測準度 ~2%），
    直到盲走距離或時間超過預算才讓 pose_age 過期、由 filter fail-closed。
    blind_max_dist_m=0 或無 odom 時退回舊行為（視覺一斷即停）。
    """

    def __init__(
        self,
        safety_filter,
        *,
        max_v_mps: float = 0.15,
        max_omega_rad_s: float = 1.5,
        control_hz: float = 20.0,
        nav_timeout_s: float = 0.2,
        robot_radius_m: float = DEFAULT_ROBOT_RADIUS_M,
        geofence: tuple[tuple[float, float], ...] = ARENA,
        aruco_fresh_s: float = 0.4,
        blind_max_dist_m: float = 0.5,
        blind_max_s: float = 5.0,
        drift_base_m: float = 0.10,
        drift_rate: float = 0.30,
        fixed_goal: tuple[float, float] | None = None,
        obstacles: tuple = (),
        plan_lookahead_m: float = 0.35,
        plan_timeout_s: float = 0.5,
    ) -> None:
        self._filter = safety_filter
        self._dt = 1.0 / control_hz
        self._nav_timeout_s = nav_timeout_s
        self._aruco_fresh_s = aruco_fresh_s
        self._blind_max_dist_m = blind_max_dist_m
        self._blind_max_s = blind_max_s
        self._drift_base_m = drift_base_m
        self._drift_rate = drift_rate
        self._desired = Twist.stop()
        self._nav_stamp_s: float | None = None
        self._pose: Pose | None = None
        self._pose_stamp_s: float | None = None
        self._wheel_feedback = (0.0, 0.0)
        self._odom_pose: Pose | None = None
        self._odom_stamp_s: float | None = None
        self._anchor_odom_pose: Pose | None = None
        self._blind_dist_m = 0.0
        self._fused: tuple[float, float] | None = None
        self._fixed_goal = fixed_goal
        self._obstacles = tuple(obstacles)
        self._plan_lookahead_m = plan_lookahead_m
        self._plan_timeout_s = plan_timeout_s
        self._plan_points: tuple[tuple[float, float], ...] | None = None
        self._plan_stamp_s: float | None = None
        safety_filter.reset(StaticInfo(
            params=DiffDriveParams(),
            robot_radius_m=robot_radius_m,
            geofence=geofence,
            max_v_mps=max_v_mps,
            max_omega_rad_s=max_omega_rad_s,
        ))

    def update_nav(self, desired: Twist, *, stamp_s: float) -> None:
        self._desired = desired
        self._nav_stamp_s = stamp_s

    def update_aruco_pose(self, pose: Pose, *, stamp_s: float) -> None:
        # 凍結重發保護：pseudo_aruco 在 dropout 時會以「凍結 stamp」重發
        # 最後一筆 pose。stamp 不更新時不得重置盲走累積（_blind_dist_m）
        # Preserve monotonic measurement stamps; restamping a frozen pose would
        # reset the visual anchor and suppress dead-reckoning distance.
        if self._pose_stamp_s is not None and stamp_s <= self._pose_stamp_s:
            return
        self._pose = pose
        self._pose_stamp_s = stamp_s
        self._anchor_odom_pose = self._odom_pose
        self._blind_dist_m = 0.0

    def update_fused_pose(
        self, pose: Pose, *, drift_m: float, corr_age_s: float, stamp_s: float
    ) -> None:
        """上游 pose_fusion 節點的融合位姿（odom 主幹＋視覺修正）。"""
        self._pose = pose
        self._pose_stamp_s = stamp_s
        self._fused = (drift_m, corr_age_s)

    def update_odom_pose(self, pose: Pose, *, stamp_s: float) -> None:
        """encoder odom 位姿（odom frame）。累積自錨點以來的路徑長。"""
        if self._odom_pose is not None:
            self._blind_dist_m += math.hypot(
                pose.x - self._odom_pose.x, pose.y - self._odom_pose.y
            )
        self._odom_pose = pose
        self._odom_stamp_s = stamp_s

    def update_wheel_feedback(self, left_mps: float, right_mps: float) -> None:
        self._wheel_feedback = (left_mps, right_mps)

    def update_obstacles(self, obstacles) -> None:
        """更新動態障礙集（資訊對等：與 DWB/RPP 同源的視覺量測）。"""
        self._obstacles = tuple(obstacles)

    def update_plan(self, points, *, stamp_s: float) -> None:
        """記住最新 Nav2 `/plan`；frame 檢查由 ROS wrapper 負責。"""
        self._plan_points = tuple(points)
        self._plan_stamp_s = stamp_s

    def update_plan_clear(self) -> None:
        """清除 plan：下一個 tick 起沒有可用的吸引目標。"""
        self._plan_points = None
        self._plan_stamp_s = None

    def _current_goal(self, now_s: float) -> tuple[tuple[float, float] | None, float]:
        """回傳 (goal, goal_age_s)。固定 goal 永不老化；plan 會老化。"""
        if self._fixed_goal is not None:
            return self._fixed_goal, 0.0
        if self._plan_points is None or self._plan_stamp_s is None:
            return None, math.inf
        if now_s - self._plan_stamp_s > self._plan_timeout_s:
            return None, math.inf
        if self._pose is None:
            return None, math.inf
        goal = select_path_lookahead(
            self._plan_points, (self._pose.x, self._pose.y), self._plan_lookahead_m
        )
        if goal is None:
            return None, math.inf
        return goal, max(0.0, now_s - self._plan_stamp_s)

    def _dead_reckoned_pose(self) -> Pose | None:
        """最後視覺錨點 ∘ (odom 自錨點以來的增量)。無 odom/錨點時回 None。"""
        if (
            self._pose is None
            or self._anchor_odom_pose is None
            or self._odom_pose is None
        ):
            return None
        a, o = self._anchor_odom_pose, self._odom_pose
        ca, sa = math.cos(-a.theta), math.sin(-a.theta)
        dx_local = ca * (o.x - a.x) - sa * (o.y - a.y)
        dy_local = sa * (o.x - a.x) + ca * (o.y - a.y)
        dtheta = o.theta - a.theta
        p = self._pose
        cp, sp = math.cos(p.theta), math.sin(p.theta)
        return Pose(
            p.x + cp * dx_local - sp * dy_local,
            p.y + sp * dx_local + cp * dy_local,
            math.atan2(math.sin(p.theta + dtheta), math.cos(p.theta + dtheta)),
        )

    def build_observation(self, now_s: float) -> tuple[Observation, dict[str, Any]]:
        if self._fused is not None:
            return self._build_fused_observation(now_s)
        if self._pose is None or self._pose_stamp_s is None:
            aruco_age_s = math.inf
        else:
            aruco_age_s = max(0.0, now_s - self._pose_stamp_s)

        pose = self._pose
        pose_age_s = aruco_age_s
        dead_reckoning = 0.0
        pose_drift_m = 0.0
        blind_time_s = aruco_age_s
        if aruco_age_s > self._aruco_fresh_s and self._blind_max_dist_m > 0.0:
            estimate = self._dead_reckoned_pose()
            within_budget = (
                estimate is not None
                and self._blind_dist_m <= self._blind_max_dist_m
                and blind_time_s <= self._blind_max_s
                and self._odom_stamp_s is not None
                and now_s - self._odom_stamp_s <= self._aruco_fresh_s
            )
            if within_budget:
                pose = estimate
                pose_age_s = max(0.0, now_s - self._odom_stamp_s)
                dead_reckoning = 1.0
                pose_drift_m = self._drift_base_m + self._drift_rate * self._blind_dist_m

        core_debug = {
            "aruco_age_s": aruco_age_s,
            "dead_reckoning": dead_reckoning,
            "blind_dist_m": self._blind_dist_m,
            "blind_time_s": blind_time_s if blind_time_s != math.inf else -1.0,
            "pose_drift_m": pose_drift_m,
            "estimated_x": pose.x if pose is not None else math.nan,
            "estimated_y": pose.y if pose is not None else math.nan,
        }
        goal, goal_age_s = self._current_goal(now_s)
        obs = Observation(
            pose=pose,
            pose_age_s=pose_age_s,
            wheel_feedback=self._wheel_feedback,
            obstacles=self._obstacles,
            link_age_s=0.0,
            pose_drift_m=pose_drift_m,
            goal=goal,
            goal_age_s=goal_age_s,
        )
        return obs, core_debug

    def _build_fused_observation(
        self, now_s: float
    ) -> tuple[Observation, dict[str, Any]]:
        """fused 模式：位姿連續（odom 主幹），預算判準改為
        「距上次被接受視覺修正的里程/時間」，超額 → 位姿視為過期 fail-closed。"""
        drift_m, corr_age_s = self._fused  # type: ignore[misc]
        pose_age_s = max(0.0, now_s - (self._pose_stamp_s or 0.0))
        budget_ok = (
            drift_m <= self._blind_max_dist_m and corr_age_s <= self._blind_max_s
        )
        if not budget_ok:
            pose_age_s = math.inf
        pose_drift_m = 0.0
        if corr_age_s > self._aruco_fresh_s:
            pose_drift_m = self._drift_base_m + self._drift_rate * drift_m
        core_debug = {
            "aruco_age_s": corr_age_s,
            "dead_reckoning": 1.0 if corr_age_s > self._aruco_fresh_s else 0.0,
            "blind_dist_m": drift_m,
            "blind_time_s": corr_age_s,
            "pose_drift_m": pose_drift_m,
            "fused": 1.0,
            "estimated_x": self._pose.x if self._pose is not None else math.nan,
            "estimated_y": self._pose.y if self._pose is not None else math.nan,
        }
        goal, goal_age_s = self._current_goal(now_s)
        obs = Observation(
            pose=self._pose,
            pose_age_s=pose_age_s,
            wheel_feedback=self._wheel_feedback,
            obstacles=self._obstacles,
            link_age_s=0.0,
            pose_drift_m=pose_drift_m,
            goal=goal,
            goal_age_s=goal_age_s,
        )
        return obs, core_debug

    def tick(self, now_s: float) -> GateOutput:
        obs, core_debug = self.build_observation(now_s)
        desired = self._desired
        if self._nav_stamp_s is None or now_s - self._nav_stamp_s > self._nav_timeout_s:
            desired = Twist.stop()
        decision: SafetyDecision = self._filter.filter(desired, obs, now_s, self._dt)
        debug = dict(decision.debug)
        debug.update(core_debug)
        return GateOutput(cmd=decision.cmd, mode=decision.mode, debug=debug)
