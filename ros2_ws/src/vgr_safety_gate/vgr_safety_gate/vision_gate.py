"""R1 視覺開關（spec 8.2：fresh anchor 後明確關閉視覺）。

Core 純 Python：`forward` 在 dropout 啟用時回 None（切斷），否則原樣轉發
（內容與量測時戳一律不改寫——stamp 語意由上游 aruco_camera_pose 負責）。
Node 薄包裝：sub `/aruco/pose_raw` → pub `/aruco/pose`，
service `/experiment/set_vision_dropout` (std_srvs/SetBool)。
"""
from __future__ import annotations


class VisionGateCore:
    def __init__(self) -> None:
        self._dropout = False

    def set_dropout(self, enabled: bool) -> None:
        self._dropout = bool(enabled)

    def dropout_active(self) -> bool:
        return self._dropout

    def forward(self, payload, stamp_s: float):
        """dropout 啟用時回 None；否則回 (payload, stamp_s) 原樣。"""
        if self._dropout:
            return None
        return payload, stamp_s


def main() -> None:  # pragma: no cover - ROS node wrapper
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from std_srvs.srv import SetBool

    rclpy.init()
    gate = VisionGateCore()

    class VisionGateNode(Node):
        def __init__(self) -> None:
            super().__init__("vision_gate")
            self.declare_parameter("in_topic", "/aruco/pose_raw")
            self.declare_parameter("out_topic", "/aruco/pose")
            self._pub = self.create_publisher(
                PoseStamped, str(self.get_parameter("out_topic").value), 10)
            self.create_subscription(
                PoseStamped, str(self.get_parameter("in_topic").value),
                self._on_pose, 10)
            self.create_service(
                SetBool, "/experiment/set_vision_dropout", self._on_dropout)

        def _on_pose(self, msg: PoseStamped) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            out = gate.forward(msg, stamp)
            if out is not None:
                self._pub.publish(out[0])

        def _on_dropout(self, request, response):
            gate.set_dropout(request.data)
            response.success = True
            response.message = f"vision dropout={'on' if gate.dropout_active() else 'off'}"
            return response

    node = VisionGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
