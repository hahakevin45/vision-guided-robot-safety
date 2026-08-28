"""Bench 專用 pseudo-pose 節點：/odom → /aruco/pose。

架空驗收（SG-A/SG-B）時 safety_gate 的位姿來源。以 identity map→odom 假設
把 encoder odometry 轉發成 PoseStamped，時戳沿用 odom 時戳；提供
/aruco/set_dropout（SetBool）注入斷鏈，與 Gazebo 驗證的 pseudo_aruco 同介面。

說明：pose 派生自 /odom，這裡驗的是安全層管線與時序行為（含斷鏈
急停），不是視覺定位品質。真 ArUco 上線（SG-C）後由定位節點取代本節點。
"""
from __future__ import annotations


def odom_to_pose_fields(px: float, py: float, q) -> tuple[float, float, object]:
    """identity map→odom：位置與姿態原樣轉發（q 為 quaternion message）。"""
    return px, py, q


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_srvs.srv import SetBool

    class BenchPseudoPoseNode(Node):
        def __init__(self) -> None:
            super().__init__("bench_pseudo_pose")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("pose_topic", "/aruco/pose")
            odom_topic = str(self.get_parameter("odom_topic").value)
            pose_topic = str(self.get_parameter("pose_topic").value)
            self._dropout = False
            self._pub = self.create_publisher(PoseStamped, pose_topic, 10)
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
            self.create_service(SetBool, "/aruco/set_dropout", self._on_dropout)

        def _on_odom(self, msg: Odometry) -> None:
            if self._dropout:
                return
            p = msg.pose.pose.position
            x, y, q = odom_to_pose_fields(p.x, p.y, msg.pose.pose.orientation)
            out = PoseStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = "map"
            out.pose.position.x = x
            out.pose.position.y = y
            out.pose.orientation = q
            self._pub.publish(out)

        def _on_dropout(self, request, response):
            self._dropout = bool(request.data)
            self.get_logger().warn(f"pseudo-pose dropout={self._dropout}")
            response.success = True
            response.message = f"dropout={self._dropout}"
            return response

    rclpy.init()
    node = BenchPseudoPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
