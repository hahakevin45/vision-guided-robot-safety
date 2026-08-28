"""Pseudo ArUco ROS2 節點。

訂閱 Gazebo ground truth `/sim/true_pose`,用 `safety_sim.sensors.ArucoLocalizer`
重現固定更新率、量測噪聲與 dropout 凍結最後定位值的語意,再發佈
`/aruco/pose`。核心類別不依賴 ROS,方便用純 Python 測試。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from safety_sim.sensors import ArucoLocalizer
from vgr_core.safety import Pose


@dataclass(frozen=True)
class ArucoMeasurement:
    pose: Pose
    stamp_s: float
    age_s: float


class PseudoArucoCore:
    """Pseudo ArUco 的純核心：保存真值、dropout 狀態並產生凍結量測。"""

    def __init__(
        self,
        update_hz: float,
        noise_xy_std: float,
        noise_theta_std: float,
        seed: int,
        dropout_after_x: float | None = None,
        resume_after_x: float | None = None,
    ) -> None:
        self.update_hz = update_hz
        self._interval_s = 1.0 / update_hz
        self._noise_theta_std = noise_theta_std
        self._dropout_after_x = dropout_after_x
        self._resume_after_x = resume_after_x
        self._localizer = ArucoLocalizer(
            update_hz=update_hz,
            noise_xy_std=noise_xy_std,
            noise_theta_std=noise_theta_std,
            seed=seed,
        )
        self._true_pose: Pose | None = None
        self._last_update_stamp_s: float | None = None
        self._dropout = False
        # 注入視窗事件：[(t0, t1)]；t1=None = 視窗開到 run 結束
        self._window_events: list[tuple[float, float | None]] = []
        self._positional_active = False
        self._pending_t0: float | None = None

    def set_dropout(self, enabled: bool) -> None:
        self._dropout = enabled

    def pop_window_events(self) -> list[tuple[float, float | None]]:
        events = self._window_events
        self._window_events = []
        return events

    def update_true_pose(self, pose: Pose) -> None:
        self._true_pose = pose

    def tick(self, now_s: float) -> ArucoMeasurement | None:
        if self._true_pose is None:
            return None
        # 位置型切斷/接回：車過 dropout_after_x 後視覺失效（模擬走出
        # marker 覆蓋區）；過 resume_after_x 後恢復（看到下一個 marker，
        # 定位校正、誤差歸零）。觸發/恢復時記錄視窗起訖。
        if (self._dropout_after_x is not None
                and not self._positional_active
                and self._true_pose.x > self._dropout_after_x
                and (self._resume_after_x is None
                     or self._true_pose.x < self._resume_after_x)):
            self._positional_active = True
            if self._resume_after_x is None:
                # 永久切斷：立即 emit (t0, None)
                self._window_events.append((now_s, None))
            else:
                # 可接回：延遲 emit，等 resume 時補 t1（node 每拍 pop，
                # 若觸發當下就 emit，t1 永遠補不上——2026-08-11 實測）。
                self._pending_t0 = now_s
        elif (self._positional_active
              and self._resume_after_x is not None
              and self._true_pose.x > self._resume_after_x):
            self._positional_active = False
            if self._pending_t0 is not None:
                self._window_events.append((self._pending_t0, now_s))
                self._pending_t0 = None
        dropout = self._dropout or self._positional_active
        raw_pose, age_s = self._localizer.observe(
            self._true_pose, now_s, dropout=dropout)
        if raw_pose is None:
            return None
        # Canonicalise to vgr_core.safety.Pose (ArucoLocalizer returns safety_sim.types.Pose)
        pose = Pose(raw_pose.x, raw_pose.y, raw_pose.theta)
        return ArucoMeasurement(pose=pose, stamp_s=now_s - age_s, age_s=age_s)


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _quaternion_from_yaw(theta: float) -> tuple[float, float, float, float]:
    half = theta / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def main() -> None:
    """啟動 ROS2 節點；ROS import 僅限這層薄包裝。"""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    class PseudoArucoNode(Node):
        """ROS topic 包裝；ArUco 模擬委派給 `PseudoArucoCore`。"""

        def __init__(self) -> None:
            super().__init__("pseudo_aruco")
            self.declare_parameter("update_hz", 10.0)
            self.declare_parameter("noise_xy_std", 0.0)
            self.declare_parameter("noise_theta_std", 0.0)
            self.declare_parameter("seed", 0)
            self.declare_parameter("dropout_after_x", -1.0)
            self.declare_parameter("resume_after_x", -1.0)
            self._core = PseudoArucoCore(
                update_hz=float(self.get_parameter("update_hz").value),
                noise_xy_std=float(self.get_parameter("noise_xy_std").value),
                noise_theta_std=float(self.get_parameter("noise_theta_std").value),
                seed=int(self.get_parameter("seed").value),
                dropout_after_x=(
                    float(self.get_parameter("dropout_after_x").value)
                    if float(self.get_parameter("dropout_after_x").value) > 0.0
                    else None
                ),
                resume_after_x=(
                    float(self.get_parameter("resume_after_x").value)
                    if float(self.get_parameter("resume_after_x").value) > 0.0
                    else None
                ),
            )
            self._pub = self.create_publisher(PoseStamped, "/aruco/pose", 10)
            self._window_pub = self.create_publisher(
                String, "/aruco/dropout_window", 10)
            # GS bridge 把 Gazebo /sim/true_pose 轉成 nav_msgs/Odometry；
            # 型別必須與 bridge 一致，否則 callback 永遠不會被呼叫。
            self.create_subscription(Odometry, "/sim/true_pose", self._on_odom, 10)
            self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
            self.create_timer(1.0 / 30.0, self._on_timer)
            self._dropout_srv = self.create_service(
                SetBool, "/aruco/set_dropout", self._on_set_dropout
            )

        def _on_set_dropout(self, request, response) -> SetBool.Response:
            """GS2 故障注入：`data=true` 讓 core 凍結最後位姿、age 增長。"""
            self._core.set_dropout(bool(request.data))
            response.success = True
            response.message = ""
            return response

        def _on_odom(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self._core.update_true_pose(
                Pose(p.x, p.y, _yaw_from_quaternion(q))
            )

        def _on_scan(self, msg: LaserScan) -> None:
            pass  # future: interrupt-driven update

        def _on_timer(self) -> None:
            now_s = self.get_clock().now().nanoseconds / 1e9
            # 注入視窗事件：trace 記錄 /aruco/dropout_window 供分析對齊
            # （stale_pose 是否由 dropout 造成，而不是 missing_goal 連帶）。
            for t0, t1 in self._core.pop_window_events():
                event = String()
                event.data = (f'{{"t0": {t0}, "t1": '
                              + (f'{t1}' if t1 is not None else 'null') + '}')
                self._window_pub.publish(event)
                self.get_logger().info(f"dropout window opened at t={t0}")
            meas = self._core.tick(now_s)
            if meas is None:
                return
            msg = PoseStamped()
            msg.header.frame_id = "map"
            # header.stamp 必須是量測時間（meas.stamp_s），不是發布時間：
            # dropout 時 core 凍結 stamp，gate 的 pose_age = now − stamp 才會
            # 增長並觸發 stale_pose fail-closed。若用 clock.now()，age 永遠
            # ≈0，dropout 對安全層完全不可見。
            msg.header.stamp = rclpy.time.Time(seconds=meas.stamp_s).to_msg()
            msg.pose.position.x = meas.pose.x
            msg.pose.position.y = meas.pose.y
            q = _quaternion_from_yaw(meas.pose.theta)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            self._pub.publish(msg)

    rclpy.init()
    node = PseudoArucoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
