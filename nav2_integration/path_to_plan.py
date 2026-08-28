"""Nav2 planner-only 橋接：ComputePathToPose → /plan 發布。

SAPF 取代 Nav2 controller 的比較 arm 用：planner_server 仍在（用隱藏 map
規劃穿障礙路線），但 controller_server 停用。本節點呼叫 /compute_path_to_pose
取得路徑後發布到 /plan（map frame），safety_gate 的 update_plan 消費它，
SAPF 從路徑 lookahead 取 goal——「全局規劃給路徑、SAPF 局部執行」。

用法：
  python3 -m nav2_integration.path_to_plan --ros-args \
    -p start_x:=0.7 -p start_y:=0.0 -p goal_x:=3.5 -p goal_y:=0.0 \
    -p plan_topic:=/plan -p rate_hz:=1.0
"""
from __future__ import annotations

import math

from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavPath


def main() -> None:  # pragma: no cover - ROS node
    import rclpy
    from geometry_msgs.msg import PoseStamped as RosPoseStamped
    from rclpy.action import ActionClient
    from rclpy.node import Node

    rclpy.init()

    class PathToPlan(Node):
        def __init__(self) -> None:
            super().__init__("path_to_plan")
            self.declare_parameter("start_x", 0.7)
            self.declare_parameter("start_y", 0.0)
            self.declare_parameter("goal_x", 3.5)
            self.declare_parameter("goal_y", 0.0)
            self.declare_parameter("plan_topic", "/plan")
            self.declare_parameter("rate_hz", 1.0)
            self._pub = self.create_publisher(
                NavPath, str(self.get_parameter("plan_topic").value), 10)
            self._client = ActionClient(
                self, ComputePathToPose, "/compute_path_to_pose")
            hz = float(self.get_parameter("rate_hz").value)
            self.create_timer(1.0 / hz, self._on_timer)

        def _goal(self) -> ComputePathToPose.Goal:
            g = ComputePathToPose.Goal()
            g.goal.header.frame_id = "map"
            g.goal.header.stamp = self.get_clock().now().to_msg()
            g.goal.pose.position.x = float(self.get_parameter("goal_x").value)
            g.goal.pose.position.y = float(self.get_parameter("goal_y").value)
            g.goal.pose.orientation.w = 1.0
            g.start.header.frame_id = "map"
            g.start.header.stamp = self.get_clock().now().to_msg()
            g.start.pose.position.x = float(self.get_parameter("start_x").value)
            g.start.pose.position.y = float(self.get_parameter("start_y").value)
            g.start.pose.orientation.w = 1.0
            return g

        def _on_timer(self) -> None:
            if not self._client.server_is_ready():
                return
            goal = self._goal()
            future = self._client.send_goal_async(goal)

            def on_goal(fut):
                goal_handle = fut.result()
                if goal_handle is None or not goal_handle.accepted:
                    self.get_logger().warn("compute_path_to_pose rejected")
                    return
                result_fut = goal_handle.get_result_async()

                def on_result(rfut):
                    result = rfut.result().result
                    if result is None or not result.path.poses:
                        return
                    path = NavPath()
                    path.header.frame_id = "map"
                    path.header.stamp = self.get_clock().now().to_msg()
                    for pose in result.path.poses:
                        p = RosPoseStamped()
                        p.header = path.header
                        p.pose = pose.pose
                        path.poses.append(p)
                    self._pub.publish(path)
                    self.get_logger().info(
                        f"published /plan with {len(path.poses)} poses")

                result_fut.add_done_callback(on_result)

            future.add_done_callback(on_goal)

    node = PathToPlan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
