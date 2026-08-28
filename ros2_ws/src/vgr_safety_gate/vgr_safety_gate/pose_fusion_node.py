"""Pose fusion node: continuous odometry backbone with visual corrections.

訂 `/odom`（連續主幹）與 `/aruco/pose`（偶發量測，用 header.stamp 對齊
odom 歷史），跑 `pose_fusion.PoseFuser`，以 control_hz 發布
`/pose_fused`（PoseWithCovarianceStamped）：

- pose：map frame 融合位姿（視覺全斷時 = 最後修正 ∘ 最新 odom，不中斷）。
- covariance[0]=covariance[7]=drift_m²（自上次被接受視覺修正以來的
  odom 路徑長平方）——下游把 drift 當位置不確定度上界。
- covariance[35]=corr_age_s（距上次被接受視覺量測多久）。

從未接受過任何視覺 → 不發布（沒有 map 座標可言）。
"""
from __future__ import annotations

import math

from vgr_safety_gate.pose_fusion import PoseFuser


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry

    class PoseFusionNode(Node):
        def __init__(self) -> None:
            super().__init__("pose_fusion")
            self.declare_parameter("control_hz", 20.0)
            self.declare_parameter("gate_dist_m", 0.15)
            self.declare_parameter("gate_yaw_rad", 0.175)
            self.declare_parameter("blend", 0.3)
            self.declare_parameter("reloc_after_rejects", 20)
            self._fuser = PoseFuser(
                gate_dist_m=float(self.get_parameter("gate_dist_m").value),
                gate_yaw_rad=float(self.get_parameter("gate_yaw_rad").value),
                blend=float(self.get_parameter("blend").value),
                reloc_after_rejects=int(
                    self.get_parameter("reloc_after_rejects").value),
            )
            self._accepted = 0
            self._rejected = 0
            self.create_subscription(Odometry, "/odom", self._on_odom, 50)
            self.create_subscription(PoseStamped, "/aruco/pose", self._on_vision, 10)
            self._pub = self.create_publisher(
                PoseWithCovarianceStamped, "/pose_fused", 10)
            hz = float(self.get_parameter("control_hz").value)
            self.create_timer(1.0 / hz, self._on_timer)
            self.create_timer(5.0, self._log_stats)
            self.get_logger().info("pose_fusion up: odom 主幹＋視覺修正")

        def _on_odom(self, msg) -> None:
            p = msg.pose.pose.position
            self._fuser.update_odom(
                p.x, p.y, _yaw_from_quaternion(msg.pose.pose.orientation),
                _stamp_to_seconds(msg.header.stamp),
            )

        def _on_vision(self, msg) -> None:
            p = msg.pose.position
            ok = self._fuser.update_vision(
                p.x, p.y, _yaw_from_quaternion(msg.pose.orientation),
                _stamp_to_seconds(msg.header.stamp),
            )
            if ok:
                self._accepted += 1
            else:
                self._rejected += 1

        def _on_timer(self) -> None:
            now_s = self.get_clock().now().nanoseconds / 1e9
            est = self._fuser.estimate(now_s)
            if est is None:
                return
            out = PoseWithCovarianceStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "map"
            out.pose.pose.position.x = est.x
            out.pose.pose.position.y = est.y
            out.pose.pose.orientation.z = math.sin(est.yaw / 2.0)
            out.pose.pose.orientation.w = math.cos(est.yaw / 2.0)
            out.pose.covariance[0] = est.drift_m ** 2
            out.pose.covariance[7] = est.drift_m ** 2
            out.pose.covariance[35] = (
                est.corr_age_s if math.isfinite(est.corr_age_s) else -1.0)
            self._pub.publish(out)

        def _log_stats(self) -> None:
            self.get_logger().info(
                f"vision accepted={self._accepted} rejected={self._rejected}")
            self._accepted = 0
            self._rejected = 0

    rclpy.init()
    node = PoseFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
