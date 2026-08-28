"""R3 共用 obstacle-free nominal controller（spec 4.2）。

Core 純 Python：純吸引場（attractive_gradient）→ Eq (10)-(11) command。
與 `safe_apf_new` 共用同一 `command_from_gradient`——當所有障礙/牆在 Q* 外時，
兩者命令必須逐 tick 相等（spec 4.2 前提測試）。
Node：sub `/pose_fused` → pub `/cmd_vel_nav`（passthrough arm 使用）。
"""
from __future__ import annotations

import math

from vgr_core.safety import Twist

from safety_sim.sapf_field import attractive_command


class SapfNominalCore:
    """Obstacle-free goal-attraction controller（spec 4.2）。

    與 `safe_apf_new` 共用 `command_from_gradient`；無障礙/牆在 Q* 內時兩者
    命令逐 tick 相等。pose 不新鮮或距 goal ≤ stop_radius → STOP。
    """

    def __init__(
        self,
        *,
        goal: tuple[float, float],
        d_g_star: float,
        zeta: float,
        v_max: float,
        omega_max: float,
        theta_error_max: float,
        k_omega: float,
        pose_fresh_s: float = 0.4,
        stop_radius_m: float = 0.05,
    ) -> None:
        if not (math.isfinite(goal[0]) and math.isfinite(goal[1])):
            raise ValueError("goal must be finite")
        self._goal = (float(goal[0]), float(goal[1]))
        self._d_g_star = float(d_g_star)
        self._zeta = float(zeta)
        self._v_max = float(v_max)
        self._omega_max = float(omega_max)
        self._theta_error_max = float(theta_error_max)
        self._k_omega = float(k_omega)
        self._fresh_s = float(pose_fresh_s)
        self._stop_radius = float(stop_radius_m)
        self._pose: tuple[float, float, float] | None = None
        self._stamp_s: float | None = None

    @property
    def zeta(self) -> float:
        return self._zeta

    def update_pose(self, pose: tuple[float, float, float], stamp_s: float) -> None:
        if not all(math.isfinite(v) for v in pose):
            raise ValueError("pose must be finite")
        self._pose = pose
        self._stamp_s = float(stamp_s)

    def command(self, now_s: float) -> Twist:
        if self._pose is None or self._stamp_s is None:
            raise RuntimeError("update_pose() must be called before command()")
        if now_s - self._stamp_s > self._fresh_s:
            return Twist.stop()
        x, y, theta = self._pose
        if math.hypot(x - self._goal[0], y - self._goal[1]) <= self._stop_radius:
            return Twist.stop()
        v, w = attractive_command(
            self._pose, self._goal,
            d_g_star=self._d_g_star, zeta=self._zeta,
            v_max=self._v_max, omega_max=self._omega_max,
            theta_error_max=self._theta_error_max, k_omega=self._k_omega,
        )
        return Twist(v, w)


def _quat_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def main() -> None:  # pragma: no cover - ROS node wrapper
    import rclpy
    from geometry_msgs.msg import Twist as RosTwist
    from rclpy.node import Node

    rclpy.init()

    class SapfNominalNode(Node):
        def __init__(self) -> None:
            super().__init__("sapf_nominal")
            self.declare_parameter("goal_x", 3.0)
            self.declare_parameter("goal_y", 0.0)
            self.declare_parameter("d_g_star", 0.30)
            self.declare_parameter("a_max", 0.50)
            self.declare_parameter("v_max", 0.15)
            self.declare_parameter("omega_max", 0.25)
            self.declare_parameter("theta_error_max", math.pi / 4.0)
            self.declare_parameter("k_omega", 1.5)
            self.declare_parameter("pose_topic", "/pose_fused")
            self.declare_parameter("pose_msg_type", "fused")
            self.declare_parameter("cmd_topic", "/cmd_vel_nav")
            self.declare_parameter("control_hz", 20.0)
            self.declare_parameter("pose_fresh_s", 0.4)
            self.declare_parameter("stop_radius_m", 0.05)

            v_max = float(self.get_parameter("v_max").value)
            from safety_sim.sapf_field import compute_analytic_gains

            zeta, _ = compute_analytic_gains(
                d_g_star=float(self.get_parameter("d_g_star").value),
                a_max=float(self.get_parameter("a_max").value),
                v_max=v_max, d_safe=0.28, Q_star=0.80,
            )
            self._core = SapfNominalCore(
                goal=(float(self.get_parameter("goal_x").value),
                      float(self.get_parameter("goal_y").value)),
                d_g_star=float(self.get_parameter("d_g_star").value),
                zeta=zeta, v_max=v_max,
                omega_max=float(self.get_parameter("omega_max").value),
                theta_error_max=float(self.get_parameter("theta_error_max").value),
                k_omega=float(self.get_parameter("k_omega").value),
                pose_fresh_s=float(self.get_parameter("pose_fresh_s").value),
                stop_radius_m=float(self.get_parameter("stop_radius_m").value),
            )
            self._pub = self.create_publisher(
                RosTwist, str(self.get_parameter("cmd_topic").value), 10)
            pose_type = str(self.get_parameter("pose_msg_type").value)
            if pose_type == "aruco":
                from geometry_msgs.msg import PoseStamped

                self.create_subscription(
                    PoseStamped, str(self.get_parameter("pose_topic").value),
                    self._on_pose_stamped, 10)
            else:
                from geometry_msgs.msg import PoseWithCovarianceStamped

                self.create_subscription(
                    PoseWithCovarianceStamped,
                    str(self.get_parameter("pose_topic").value),
                    self._on_pose, 10)
            hz = float(self.get_parameter("control_hz").value)
            self.create_timer(1.0 / hz, self._on_timer)

        def _on_pose(self, msg) -> None:
            p = msg.pose.pose.position
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q = msg.pose.pose.orientation
            self._core.update_pose(
                (p.x, p.y, _quat_yaw(q)), stamp_s=stamp)

        def _on_pose_stamped(self, msg) -> None:
            p = msg.pose.position
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._core.update_pose(
                (p.x, p.y, _quat_yaw(msg.pose.orientation)), stamp_s=stamp)

        def _on_timer(self) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            try:
                cmd = self._core.command(now_s=now)
            except RuntimeError:
                # Discovery may deliver pose after the first timer tick;
                # fail-safe STOP until a valid pose arrives.
                cmd = Twist(0.0, 0.0)
            msg = RosTwist()
            msg.linear.x = cmd.v
            msg.angular.z = cmd.omega
            self._pub.publish(msg)

    node = SapfNominalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
