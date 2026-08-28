"""R1 盲走距離驅動器（spec 8.2/6.3）。

Core 純 Python：以 odom 路徑長（非弦長）累積盲走里程；pose 不新鮮即 STOP；
到達 target 後 latched STOP；0m cell 永不發非零。速度/距離限制於 ctor
拒絕（spec 8.3：0.05/0.15/0.22 m/s；0.5/1/2/3 m，0m 為 0 速度）。
Node：sub `/pose_fused`（odom 主幹＋視覺修正，R1 中視覺已斷 → odom 主幹），
pub `/cmd_vel_nav`（進入既有 safety gate 鏈）。
"""
from __future__ import annotations

import math

from vgr_core.safety import Twist

from safety_sim.experiments.physical_contract import R1_MAX_SPEED_MPS

_VALID_SPEEDS = (0.0, 0.05, 0.15, 0.22)
_VALID_DISTANCES = (0.0, 0.5, 1.0, 2.0, 3.0)


class BlindDistanceDriverCore:
    def __init__(
        self,
        *,
        target_distance_m: float,
        speed_mps: float,
        pose_fresh_s: float = 0.4,
    ) -> None:
        if speed_mps not in _VALID_SPEEDS:
            raise ValueError(
                f"speed {speed_mps} not in spec set {_VALID_SPEEDS}; "
                f"R1 cap is {R1_MAX_SPEED_MPS} m/s (platform ceiling ~0.245 m/s)")
        if speed_mps >= 0.24:  # 防呆：任何接近/超過天花板的值一律拒絕
            raise ValueError("speed must stay below the platform ceiling")
        if target_distance_m not in _VALID_DISTANCES:
            raise ValueError(
                f"target distance {target_distance_m} not in spec set "
                f"{_VALID_DISTANCES}")
        if speed_mps == 0.0 and target_distance_m != 0.0:
            raise ValueError("zero speed is only valid for the 0 m cell")
        self._target = float(target_distance_m)
        self._speed = float(speed_mps)
        self._fresh_s = float(pose_fresh_s)
        self._anchor: tuple[float, float] | None = None
        self._last_pose: tuple[float, float] | None = None
        self._last_stamp_s: float | None = None
        self._path_m = 0.0
        self._reached = False

    def reset(self, anchor: tuple[float, float]) -> None:
        self._anchor = anchor
        self._last_pose = anchor
        self._last_stamp_s = None
        self._path_m = 0.0
        self._reached = False

    def update_pose(self, pose: tuple[float, float], stamp_s: float) -> None:
        if self._last_pose is None:
            self.reset(pose)
            return
        if self._last_stamp_s is not None and stamp_s <= self._last_stamp_s:
            return  # 亂序/重複樣本忽略
        dx = pose[0] - self._last_pose[0]
        dy = pose[1] - self._last_pose[1]
        self._path_m += math.hypot(dx, dy)
        self._last_pose = pose
        self._last_stamp_s = stamp_s

    @property
    def blind_distance_m(self) -> float:
        return self._path_m

    def command(self, now_s: float) -> Twist:
        if self._last_pose is None:
            raise RuntimeError("reset() must be called before command()")
        fresh = (
            self._last_stamp_s is not None
            and (now_s - self._last_stamp_s) <= self._fresh_s
        )
        if not fresh:
            return Twist.stop()
        if self._reached or self._path_m >= self._target - 1e-9:
            self._reached = True
            return Twist.stop()
        if self._speed == 0.0:
            return Twist.stop()
        return Twist(self._speed, 0.0)


def main() -> None:  # pragma: no cover - ROS node wrapper
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node

    rclpy.init()

    class BlindDistanceDriverNode(Node):
        def __init__(self) -> None:
            super().__init__("blind_distance_driver")
            self.declare_parameter("target_distance_m", 1.0)
            self.declare_parameter("speed_mps", 0.15)
            self.declare_parameter("pose_topic", "/pose_fused")
            self.declare_parameter("cmd_topic", "/cmd_vel_nav")
            self.declare_parameter("control_hz", 20.0)
            self.declare_parameter("pose_fresh_s", 0.4)

            self._core = BlindDistanceDriverCore(
                target_distance_m=float(self.get_parameter("target_distance_m").value),
                speed_mps=float(self.get_parameter("speed_mps").value),
                pose_fresh_s=float(self.get_parameter("pose_fresh_s").value),
            )
            self._pub = self.create_publisher(
                Twist, str(self.get_parameter("cmd_topic").value), 10)
            self.create_subscription(
                PoseWithCovarianceStamped,
                str(self.get_parameter("pose_topic").value),
                self._on_pose, 10)
            hz = float(self.get_parameter("control_hz").value)
            self.create_timer(1.0 / hz, self._on_timer)

        def _on_pose(self, msg) -> None:
            p = msg.pose.pose.position
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._core.update_pose((p.x, p.y), stamp_s=stamp)

        def _on_timer(self) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            try:
                cmd = self._core.command(now_s=now)
            except RuntimeError:
                # 尚未 reset（discovery 競態）：fail-safe STOP（同 sapf_nominal）。
                cmd = Twist(0.0, 0.0)
            self._pub.publish(cmd)

    node = BlindDistanceDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
