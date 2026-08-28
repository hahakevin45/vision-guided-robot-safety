"""Scripted nav ROS2 節點。

依 `profile` 參數選擇 GS1/GS2 對應的 safety_sim 情境 nav 定義，並以
20 Hz 發佈 `/cmd_vel_nav`。GS2 的 dropout 注入由情境腳本呼叫 service，
不在此節點處理。
"""
from __future__ import annotations

import math

from safety_sim.scenarios import get_scenario
from vgr_core.safety import Observation, Twist


_PROFILE_TO_SCENARIO = {
    "gs1_wall_rush": "S1",
    "gs2_blackout": "S2",
    "gs3_sapf_single_obstacle": "S8",
}


class ScriptedNavCore:
    """Scripted nav 純核心：直接重用 `safety_sim.scenarios` 的 nav factory。"""

    def __init__(self, profile: str) -> None:
        try:
            scenario_name = _PROFILE_TO_SCENARIO[profile]
        except KeyError:
            raise ValueError(
                f"unknown profile {profile!r}; available: {sorted(_PROFILE_TO_SCENARIO)}"
            ) from None
        self.profile = profile
        self._nav = get_scenario(scenario_name).make_nav()

    def command(self, now_s: float) -> Twist:
        obs = Observation(
            pose=None,
            pose_age_s=math.inf,
            wheel_feedback=(0.0, 0.0),
            obstacles=(),
            link_age_s=0.0,
        )
        raw = self._nav.command(obs, now_s)  # safety_sim.types.Twist
        return Twist(raw.v, raw.omega)  # canonical vgr_core.safety.Twist


def main() -> None:
    """啟動 ROS2 節點；ROS import 僅限此薄包裝。"""
    import rclpy
    from geometry_msgs.msg import Twist as RosTwist
    from rclpy.node import Node

    class ScriptedNavNode(Node):
        """ROS timer 包裝；命令內容委派給 `ScriptedNavCore`。"""

        def __init__(self) -> None:
            super().__init__("scripted_nav")
            self.declare_parameter("profile", "gs1_wall_rush")
            self._core = ScriptedNavCore(str(self.get_parameter("profile").value))
            self._pub = self.create_publisher(RosTwist, "/cmd_vel_nav", 10)
            self.create_timer(1.0 / 20.0, self._on_timer)

        def _on_timer(self) -> None:
            twist = self._core.command(self.get_clock().now().nanoseconds / 1e9)
            msg = RosTwist()
            msg.linear.x = twist.v
            msg.angular.z = twist.omega
            self._pub.publish(msg)

    rclpy.init()
    node = ScriptedNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
