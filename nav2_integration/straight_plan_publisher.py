"""nav2_integration/straight_plan_publisher.py
發布穿箱直線 /plan（map frame）：隱藏障礙比較場景的統一 plan 輸入。
"""


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath
    from rclpy.node import Node

    rclpy.init()

    class StraightPlanPublisher(Node):
        def __init__(self) -> None:
            super().__init__("straight_plan_publisher")
            self.declare_parameter("start_x", 0.7)
            self.declare_parameter("goal_x", 3.5)
            self.declare_parameter("spacing_m", 0.1)
            self.declare_parameter("rate_hz", 2.0)
            self._pub = self.create_publisher(NavPath, "/plan", 10)
            self.create_timer(1.0 / float(self.get_parameter("rate_hz").value),
                              self._on_timer)

        def _on_timer(self) -> None:
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
            self._pub.publish(path)

    node = StraightPlanPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
