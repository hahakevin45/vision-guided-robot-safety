"""nav2_integration/follow_path_client.py
把穿箱直線 path 送成 FollowPath action goal——controller_only 架構沒有
bt/planner 發 action，需要這個薄 client 讓 controller 開始跟 path。
直線定義與場景一致（start 0.7 → goal 3.5, map frame），不訂閱 /plan。
"""
from __future__ import annotations


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.node import Node

    rclpy.init()

    class FollowPathClient(Node):
        def __init__(self) -> None:
            super().__init__("follow_path_client")
            self.declare_parameter("start_x", 0.7)
            self.declare_parameter("goal_x", 3.5)
            self.declare_parameter("spacing_m", 0.1)
            self._client = ActionClient(self, FollowPath, "/follow_path")
            self._sent = False
            self.create_timer(0.5, self._on_timer)

        def _make_path(self) -> NavPath:
            start = float(self.get_parameter("start_x").value)
            goal = float(self.get_parameter("goal_x").value)
            spacing = float(self.get_parameter("spacing_m").value)
            path = NavPath()
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            x = start
            while x <= goal + 1e-6:
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x = x
                p.pose.position.y = 0.0
                p.pose.orientation.w = 1.0
                path.poses.append(p)
                x += spacing
            return path

        def _on_timer(self) -> None:
            if self._sent:
                return
            if not self._client.server_is_ready():
                self.get_logger().warn("follow_path server not ready")
                return
            self._sent = True
            goal = FollowPath.Goal()
            goal.path = self._make_path()
            self.get_logger().info(
                f"send follow_path goal with {len(goal.path.poses)} poses")
            future = self._client.send_goal_async(goal)
            future.add_done_callback(self._on_goal_response)

        def _on_goal_response(self, future) -> None:
            try:
                handle = future.result()
            except Exception as exc:  # noqa: BLE001 - rclpy 例外種類多
                self.get_logger().warn(f"goal send failed ({exc}); retry")
                self._sent = False
                return
            if handle.accepted:
                self.get_logger().info("follow_path goal accepted")
            else:
                self.get_logger().warn("goal rejected; retry")
                self._sent = False

    node = FollowPathClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
