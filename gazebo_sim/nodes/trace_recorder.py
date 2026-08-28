"""Trace recorder ROS2 節點。

訂閱 Gazebo/安全層 topic，依 sim time 寫 JSONL。每行是一個 topic event，
欄位保持足以由 `gazebo_sim.trace_adapter.load_trace()` 還原成
`safety_sim.runner.TraceSample` 的最小集合。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from vgr_core.safety import Pose, Twist


def _pose_dict(pose: Pose) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "theta": pose.theta}


def _twist_dict(twist: Twist) -> dict[str, float]:
    return {"v": twist.v, "omega": twist.omega}


class TraceRecorderCore:
    """JSONL trace 的純核心：接收各 topic event 並序列化為 dict。"""

    def __init__(self) -> None:
        self.rows: list[dict] = []

    def record_true_pose(self, t: float, pose: Pose, actual_twist: Twist | None = None) -> None:
        row = {"topic": "/sim/true_pose", "t": t, "true_pose": _pose_dict(pose)}
        if actual_twist is not None:
            row["actual_twist"] = _twist_dict(actual_twist)
        self.rows.append(row)

    def record_twist(self, topic: str, t: float, twist: Twist) -> None:
        self.rows.append({"topic": topic, "t": t, "twist": _twist_dict(twist)})

    def record_aruco_pose(self, t: float, pose: Pose, *, stamp_s: float,
                          topic: str = "/aruco/pose") -> None:
        if topic not in {"/aruco/pose", "/aruco/pose_raw"}:
            raise ValueError(f"unsupported ArUco pose topic: {topic}")
        self.rows.append({"topic": topic, "t": t, "stamp_s": stamp_s,
                          "pose": _pose_dict(pose)})

    def record_odom(self, t: float, pose: Pose, twist: Twist) -> None:
        self.rows.append({"topic": "/odom", "t": t,
                          "pose": _pose_dict(pose), "twist": _twist_dict(twist)})

    def record_marker_ids(self, t: float, *, stamp_s: float,
                          ids: tuple[int, ...]) -> None:
        self.rows.append({"topic": "/aruco/marker_ids", "t": t,
                          "stamp_s": stamp_s, "ids": list(ids)})

    def record_status(self, t: float, mode: str, debug: dict[str, float]) -> None:
        self.rows.append({"topic": "/safety_gate/status", "t": t, "mode": mode, "debug": debug})

    def record_event(self, t: float, topic: str, payload: dict) -> None:
        row = {"topic": topic, "t": t}
        row.update(payload)
        self.rows.append(row)

    def to_jsonl(self) -> str:
        return "\n".join(json.dumps(row, sort_keys=True) for row in self.rows)

    def write_jsonl(self, path: str | Path) -> None:
        text = self.to_jsonl()
        Path(path).write_text(text + ("\n" if text else ""), encoding="utf-8")


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    """啟動 ROS2 recorder；ROS 型別只在此包裝層出現。"""
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist as RosTwist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    class TraceRecorderNode(Node):
        """ROS topic 包裝；JSONL 格式委派給 `TraceRecorderCore`。"""

        def __init__(self) -> None:
            super().__init__("trace_recorder")
            self.declare_parameter("output_path", "gazebo_trace.jsonl")
            self._output_path = Path(str(self.get_parameter("output_path").value))
            self._core = TraceRecorderCore()
            self._obstacles_present: bool | None = None
            self.create_subscription(Odometry, "/sim/true_pose", self._on_true_pose, 50)
            self.create_subscription(RosTwist, "/cmd_vel_nav", self._on_nav, 50)
            self.create_subscription(RosTwist, "/cmd_vel_safe", self._on_safe, 50)
            self.create_subscription(PoseStamped, "/aruco/pose", self._on_aruco, 50)
            self.create_subscription(PoseStamped, "/aruco/pose_raw", self._on_aruco_raw, 50)
            self.create_subscription(Odometry, "/odom", self._on_odom, 50)
            self.create_subscription(String, "/aruco/marker_ids", self._on_marker_ids, 50)
            self.create_subscription(String, "/safety_gate/status", self._on_status, 50)
            self.create_subscription(String, "/aruco/dropout_window", self._on_window, 50)
            self.create_subscription(
                String, "/obstacles_measured", self._on_obstacles, 10)

        def _now_s(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _on_true_pose(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            twist = msg.twist.twist
            self._core.record_true_pose(
                self._now_s(),
                Pose(p.x, p.y, _yaw_from_quaternion(msg.pose.pose.orientation)),
                Twist(twist.linear.x, twist.angular.z),
            )
            self._flush()

        def _on_nav(self, msg: RosTwist) -> None:
            self._core.record_twist("/cmd_vel_nav", self._now_s(), Twist(msg.linear.x, msg.angular.z))
            self._flush()

        def _on_safe(self, msg: RosTwist) -> None:
            self._core.record_twist("/cmd_vel_safe", self._now_s(), Twist(msg.linear.x, msg.angular.z))
            self._flush()

        def _on_aruco(self, msg: PoseStamped) -> None:
            self._record_aruco(msg, "/aruco/pose")

        def _on_aruco_raw(self, msg: PoseStamped) -> None:
            self._record_aruco(msg, "/aruco/pose_raw")

        def _record_aruco(self, msg: PoseStamped, topic: str) -> None:
            p = msg.pose.position
            self._core.record_aruco_pose(
                self._now_s(),
                Pose(p.x, p.y, _yaw_from_quaternion(msg.pose.orientation)),
                stamp_s=_stamp_to_seconds(msg.header.stamp),
                topic=topic,
            )
            self._flush()

        def _on_odom(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            twist = msg.twist.twist
            self._core.record_odom(
                self._now_s(),
                Pose(p.x, p.y, _yaw_from_quaternion(msg.pose.pose.orientation)),
                Twist(twist.linear.x, twist.angular.z),
            )
            self._flush()

        def _on_marker_ids(self, msg: String) -> None:
            # 嚴格解析：stamp_s 須為有限數值、ids 須為整數 list（bool 視為無效）。
            # 任何 malformed 訊息一律 log + 跳過；空 ids list 是合法觀察。
            try:
                payload = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                self.get_logger().warn("marker_ids: malformed JSON, skipping")
                return
            if not isinstance(payload, dict):
                self.get_logger().warn("marker_ids: expected JSON object, skipping")
                return
            stamp_s = payload.get("stamp_s")
            if isinstance(stamp_s, bool) or not isinstance(stamp_s, (int, float)) \
                    or not math.isfinite(float(stamp_s)):
                self.get_logger().warn("marker_ids: invalid stamp_s, skipping")
                return
            ids = payload.get("ids")
            if not isinstance(ids, list):
                self.get_logger().warn("marker_ids: ids not a list, skipping")
                return
            parsed_ids: list[int] = []
            for item in ids:
                if isinstance(item, bool) or not isinstance(item, int):
                    self.get_logger().warn("marker_ids: non-integer id, skipping")
                    return
                parsed_ids.append(item)
            self._core.record_marker_ids(
                self._now_s(), stamp_s=float(stamp_s), ids=tuple(parsed_ids))
            self._flush()

        def _on_obstacles(self, msg: String) -> None:
            try:
                obstacles = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            present = bool(obstacles)
            if present == self._obstacles_present:
                return
            self._obstacles_present = present
            self._core.record_event(
                self._now_s(),
                "/obstacles_measured",
                {
                    "present": present,
                    "count": len(obstacles) if isinstance(obstacles, list) else int(present),
                },
            )
            self._flush()

        def _on_window(self, msg: String) -> None:
            # 注入視窗事件：分析時對齊 stale_pose 成因（dropout vs 其他）。
            payload = json.loads(msg.data)
            self._core.record_event(
                self._now_s(), "/aruco/dropout_window", payload)
            self._flush()

        def _on_status(self, msg: String) -> None:
            payload = json.loads(msg.data)
            self._core.record_status(self._now_s(), str(payload.get("mode", "UNKNOWN")),
                                     dict(payload.get("debug", {})))
            self._flush()

        def _flush(self) -> None:
            self._core.write_jsonl(self._output_path)

    rclpy.init()
    node = TraceRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node._flush()
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
