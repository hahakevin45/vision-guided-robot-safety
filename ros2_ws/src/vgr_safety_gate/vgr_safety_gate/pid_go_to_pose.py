"""短程 go-to-pose PID 控制器（Nav2 RPP 的輕量替代）。

2026-07-15 檢討：RPP 帶著 progress checker／Spin 恢復／costmap，在
0.08 m/s 低速小車上反覆互咬（判死蠕行、恢復旋轉把 marker 轉出視野）。
短程點到點在空走廊不需要那些，改用教科書 ρ-α-β 控制律：

    ρ = 到 goal 的距離，α = 車頭對 goal 方位的誤差，β 到點後的朝向誤差
    v = Kρ·ρ（鉗 v_max、地板 v_min 高於馬達靜摩擦）
    ω = Kα·α（鉗 ω_max；旋轉階段地板 ω_min）

行為約束（實車教訓）：
- 禁止倒車：前萬向輪倒車要甩 180°，路徑嚴重不可預測 → v 永遠 ≥ 0。
- |α| 大時原地轉、對準了才前進（帶遲滯避免抖振）。
- 視覺斷線時用「最後視覺錨點 ∘ odom 增量」推算（同 safety gate 的
  盲走推算；預算的強制執行仍在 safety gate，這裡只求命令連續）。

責任鏈不變：本節點發 /cmd_vel_nav → safety_gate → /cmd_vel_safe →
bridge。掛 NavigateToPose action server，pi_nav2_ground_goal harness
與 PASS 標準原樣沿用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class PidParams:
    v_max_mps: float = 0.08
    omega_max_rad_s: float = 0.25
    k_rho: float = 0.6
    k_alpha: float = 1.2
    k_yaw: float = 1.0
    # 馬達靜摩擦下限：<~20 cps（≈0.005 m/s 線速）起不動；命令地板要留裕度。
    v_min_mps: float = 0.02
    omega_min_rad_s: float = 0.12
    pos_tol_m: float = 0.05
    yaw_tol_rad: float = 0.20
    final_yaw_align: bool = True   # False=位置到即完成（終段搖頭修正 2026-07-19：
                                   # 貼牆/盲區時視覺不穩，原地追朝向會左右振盪且
                                   # 原地旋轉是編碼器盲區——衝牆類實驗朝向無意義）
    align_enter_rad: float = 0.50
    align_exit_rad: float = 0.15


@dataclass(frozen=True)
class PidState:
    aligning: bool = False
    done: bool = False


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _floored(value: float, floor: float, limit: float) -> float:
    """鉗到 ±limit，且絕對值不低於 floor（保持方向）。"""
    clamped = _clamp(value, limit)
    if clamped == 0.0:
        return 0.0
    return math.copysign(max(abs(clamped), floor), clamped)


def pid_step(
    pose: tuple[float, float, float],
    goal: tuple[float, float, float],
    state: PidState,
    params: PidParams,
) -> tuple[float, float, PidState]:
    """一步控制：回傳 (v, omega, 新 state)。v 永遠 ≥ 0（禁止倒車）。"""
    if state.done:
        return 0.0, 0.0, state

    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]
    rho = math.hypot(dx, dy)

    if rho <= params.pos_tol_m:
        # 到點：原地對正最終朝向（final_yaw_align=False 時位置到即完成）
        if not params.final_yaw_align:
            return 0.0, 0.0, replace(state, done=True)
        yaw_err = _wrap(goal[2] - pose[2])
        if abs(yaw_err) <= params.yaw_tol_rad:
            return 0.0, 0.0, replace(state, done=True)
        omega = _floored(params.k_yaw * yaw_err,
                         params.omega_min_rad_s, params.omega_max_rad_s)
        return 0.0, omega, replace(state, aligning=False)

    alpha = _wrap(math.atan2(dy, dx) - pose[2])
    aligning = state.aligning
    if aligning and abs(alpha) <= params.align_exit_rad:
        aligning = False
    elif not aligning and abs(alpha) >= params.align_enter_rad:
        aligning = True

    if aligning:
        omega = _floored(params.k_alpha * alpha,
                         params.omega_min_rad_s, params.omega_max_rad_s)
        return 0.0, omega, replace(state, aligning=True)

    v = min(max(params.k_rho * rho, params.v_min_mps), params.v_max_mps)
    omega = _clamp(params.k_alpha * alpha, params.omega_max_rad_s)
    return v, omega, replace(state, aligning=False)


def compose_blind_pose(
    anchor: tuple[float, float, float],
    odom_at_anchor: tuple[float, float, float],
    odom_now: tuple[float, float, float],
) -> tuple[float, float, float]:
    """最後視覺錨點 ∘ odom 增量（增量以錨點時車體座標表達，同 gate 推算）。"""
    dx = odom_now[0] - odom_at_anchor[0]
    dy = odom_now[1] - odom_at_anchor[1]
    dtheta = _wrap(odom_now[2] - odom_at_anchor[2])
    c0 = math.cos(-odom_at_anchor[2])
    s0 = math.sin(-odom_at_anchor[2])
    body_dx = c0 * dx - s0 * dy
    body_dy = s0 * dx + c0 * dy
    ca = math.cos(anchor[2])
    sa = math.sin(anchor[2])
    return (
        anchor[0] + ca * body_dx - sa * body_dy,
        anchor[1] + sa * body_dx + ca * body_dy,
        _wrap(anchor[2] + dtheta),
    )


def main() -> None:
    import threading
    import time

    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, Path
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    def _yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    class PidGoToPoseNode(Node):
        def __init__(self) -> None:
            super().__init__("pid_go_to_pose")
            self.declare_parameter("pose_topic", "/aruco/pose")
            # "aruco"=直接吃視覺＋自家 anchor∘odom 推算（舊行為）；
            # "fused"=吃 pose_fusion 的 /pose_fused（odom 主幹，連續、
            # 時間已對齊、離群已拒收），不再自己推算。
            self.declare_parameter("pose_source", "aruco")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("cmd_topic", "/cmd_vel_nav")
            self.declare_parameter("control_hz", 20.0)
            self.declare_parameter("aruco_fresh_s", 0.4)
            defaults = PidParams()
            for name in defaults.__dataclass_fields__:
                v = getattr(defaults, name)
                self.declare_parameter(name, v if isinstance(v, bool) else float(v))
            self._params = PidParams(**{
                name: (bool(self.get_parameter(name).value)
                       if isinstance(getattr(defaults, name), bool)
                       else float(self.get_parameter(name).value))
                for name in defaults.__dataclass_fields__
            })
            self._aruco_fresh_s = float(self.get_parameter("aruco_fresh_s").value)

            self._lock = threading.Lock()
            self._vision: tuple[float, float, float] | None = None
            self._vision_mono_s: float | None = None
            self._fused_pose: tuple[float, float, float] | None = None
            self._fused_mono_s: float | None = None
            self._odom: tuple[float, float, float] | None = None
            self._odom_mono_s: float | None = None
            self._odom_at_anchor: tuple[float, float, float] | None = None
            self._goal: tuple[float, float, float] | None = None
            self._state = PidState()
            self._done_event = threading.Event()

            group = ReentrantCallbackGroup()
            cmd_topic = str(self.get_parameter("cmd_topic").value)
            self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
            # 兩點直線 Path：給 harness 的 plan_published 檢查當證據
            self._plan_pub = self.create_publisher(Path, "/plan", 10)
            self._pose_source = str(self.get_parameter("pose_source").value)
            if self._pose_source == "fused":
                self.create_subscription(
                    PoseWithCovarianceStamped, "/pose_fused",
                    self._on_fused, 10, callback_group=group)
                self.get_logger().info("pose source: /pose_fused（odom 主幹）")
            else:
                self.create_subscription(
                    PoseStamped, str(self.get_parameter("pose_topic").value),
                    self._on_pose, 10, callback_group=group)
            self.create_subscription(
                Odometry, str(self.get_parameter("odom_topic").value),
                self._on_odom, 10, callback_group=group)
            control_hz = float(self.get_parameter("control_hz").value)
            self.create_timer(1.0 / control_hz, self._on_timer,
                              callback_group=group)
            self._server = ActionServer(
                self, NavigateToPose, "navigate_to_pose",
                execute_callback=self._execute,
                cancel_callback=lambda _req: CancelResponse.ACCEPT,
                callback_group=group)
            self.get_logger().info(
                f"pid_go_to_pose ready: {self._params}")

        def _on_pose(self, msg: PoseStamped) -> None:
            p = msg.pose.position
            with self._lock:
                self._vision = (p.x, p.y, _yaw_from_quaternion(msg.pose.orientation))
                self._vision_mono_s = time.monotonic()
                self._odom_at_anchor = self._odom

        def _on_fused(self, msg) -> None:
            p = msg.pose.pose.position
            drift_sq = float(msg.pose.covariance[0])
            with self._lock:
                self._fused_pose = (
                    p.x, p.y, _yaw_from_quaternion(msg.pose.pose.orientation))
                self._fused_mono_s = time.monotonic()
                self._fused_drift_m = drift_sq ** 0.5 if drift_sq > 0.0 else 0.0

        def _on_odom(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            with self._lock:
                self._odom = (p.x, p.y,
                              _yaw_from_quaternion(msg.pose.pose.orientation))
                self._odom_mono_s = time.monotonic()

        def _estimate(self) -> tuple[float, float, float] | None:
            now = time.monotonic()
            if self._pose_source == "fused":
                # 融合位姿本身就是 odom 主幹（連續）；停止發布（安全層
                # 判定位姿不可用）超過 fresh 視為斷線。
                if (self._fused_pose is not None
                        and self._fused_mono_s is not None
                        and now - self._fused_mono_s <= self._aruco_fresh_s):
                    return self._fused_pose
                return None
            if self._vision is None or self._vision_mono_s is None:
                return None
            if now - self._vision_mono_s <= self._aruco_fresh_s:
                return self._vision
            # 視覺斷線：odom 推算（強制停止的預算在 safety gate）
            if (
                self._odom is not None
                and self._odom_mono_s is not None
                and now - self._odom_mono_s <= self._aruco_fresh_s
                and self._odom_at_anchor is not None
            ):
                return compose_blind_pose(self._vision, self._odom_at_anchor,
                                          self._odom)
            return None

        def _on_timer(self) -> None:
            cmd = Twist()
            with self._lock:
                goal = self._goal
                estimate = self._estimate() if goal is not None else None
                if goal is not None and estimate is not None and not self._state.done:
                    v, omega, self._state = pid_step(
                        estimate, goal, self._state, self._params)
                    # 盲走凍結朝向（2026-07-19，開發紀錄 07-16 遺留正主）：
                    # 定位漂移大時瞄準角不可信，原地旋轉修正只會追噪聲、
                    # 且原地旋轉是編碼器盲區——漂移中的純旋轉命令歸零。
                    drift = getattr(self, "_fused_drift_m", 0.0)
                    if v == 0.0 and omega != 0.0 and drift > 0.12:
                        omega = 0.0
                    cmd.linear.x = v
                    cmd.angular.z = omega
                    if self._state.done:
                        self._done_event.set()
                if goal is None:
                    return  # 無任務不發命令（避免搶 /cmd_vel_nav）
            self._cmd_pub.publish(cmd)

        def _execute(self, goal_handle):
            pose = goal_handle.request.pose.pose
            goal = (pose.position.x, pose.position.y,
                    _yaw_from_quaternion(pose.orientation))
            with self._lock:
                self._goal = goal
                self._state = PidState()
                self._done_event.clear()
            self.get_logger().info(f"goal accepted: {goal}")
            plan = Path()
            plan.header.frame_id = "map"
            plan.header.stamp = self.get_clock().now().to_msg()
            with self._lock:
                start = self._estimate()
            for px, py in ([start[:2]] if start else []) + [goal[:2]]:
                ps = PoseStamped()
                ps.header = plan.header
                ps.pose.position.x = float(px)
                ps.pose.position.y = float(py)
                plan.poses.append(ps)
            self._plan_pub.publish(plan)
            try:
                while rclpy.ok():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return NavigateToPose.Result()
                    if self._done_event.wait(timeout=0.1):
                        break
                self.get_logger().info("goal reached")
                goal_handle.succeed()
                return NavigateToPose.Result()
            finally:
                with self._lock:
                    self._goal = None
                    self._state = PidState()
                # 收尾送零命令，讓 gate 的 nav_timeout 之前輪上已是 0
                for _ in range(3):
                    self._cmd_pub.publish(Twist())
                    time.sleep(0.05)

    rclpy.init()
    node = PidGoToPoseNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
