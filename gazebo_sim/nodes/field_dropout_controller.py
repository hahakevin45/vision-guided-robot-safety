"""One-shot controlled dropout scheduler for the active field safety plan.

Subscribes to the simulated true pose (nav_msgs/Odometry) and, when the robot
enters the decreasing-x dropout window, calls the vision-gate dropout service
(`/experiment/set_vision_dropout`) exactly once to open the window and again
to close it. The true pose is consumed here and never republished anywhere.

Pure core (`PositionDropoutWindow`, `DropoutTransition`) lives at module top so
it is unit-testable without ROS; ROS imports are confined to `main()`.
"""
from __future__ import annotations

from dataclasses import dataclass
import json

from vgr_core.safety import Pose


@dataclass(frozen=True)
class DropoutTransition:
    dropout: bool
    request_t_s: float
    pose: Pose

    def payload(self, applied_t_s: float) -> dict:
        return {
            "event": "dropout_start" if self.dropout else "dropout_end",
            "dropout": self.dropout,
            "request_t_s": self.request_t_s,
            "applied_t_s": float(applied_t_s),
            "pose": {"x": self.pose.x, "y": self.pose.y, "theta": self.pose.theta},
        }


class PositionDropoutWindow:
    def __init__(self, *, enabled: bool, dropout_x: float, resume_x: float):
        if enabled and not resume_x < dropout_x:
            raise ValueError("decreasing-x window requires resume_x < dropout_x")
        self.enabled = bool(enabled)
        self.dropout_x = float(dropout_x)
        self.resume_x = float(resume_x)
        self.phase = "await_start"

    def observe(self, pose: Pose, stamp_s: float) -> DropoutTransition | None:
        if not self.enabled:
            return None
        if self.phase == "await_start" and pose.x <= self.dropout_x:
            return DropoutTransition(True, float(stamp_s), pose)
        if self.phase == "blind" and pose.x <= self.resume_x:
            return DropoutTransition(False, float(stamp_s), pose)
        return None

    def commit(self, transition: DropoutTransition) -> None:
        expected = (self.phase == "await_start" and transition.dropout) or (
            self.phase == "blind" and not transition.dropout)
        if not expected:
            raise ValueError("transition does not match current phase")
        self.phase = "blind" if transition.dropout else "done"


def main() -> None:
    """Start the ROS wrapper; ROS types appear only in this thin layer."""
    import math

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    def _yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    class FieldDropoutControllerNode(Node):
        def __init__(self) -> None:
            super().__init__("field_dropout_controller")
            self.declare_parameter("enabled", False)
            self.declare_parameter("dropout_x", 1.25)
            self.declare_parameter("resume_x", 0.70)
            self.declare_parameter("pose_topic", "/sim/true_pose")
            self.declare_parameter("service", "/experiment/set_vision_dropout")
            self._core = PositionDropoutWindow(
                enabled=bool(self.get_parameter("enabled").value),
                dropout_x=float(self.get_parameter("dropout_x").value),
                resume_x=float(self.get_parameter("resume_x").value),
            )
            self._client = self.create_client(
                SetBool, str(self.get_parameter("service").value))
            # At most one pending future + its transition at a time.
            self._pending = None
            self._pending_transition = None
            self._window_pub = self.create_publisher(
                String, "/aruco/dropout_window", 10)
            self.create_subscription(
                Odometry, str(self.get_parameter("pose_topic").value),
                self._on_odom, 10)

        def _on_odom(self, msg: Odometry) -> None:
            # Never republish true pose: it is consumed here and only a
            # dropout_window JSON event is ever emitted.
            if self._pending is not None:
                return  # one in flight; retry/handle on completion
            p = msg.pose.pose.position
            pose = Pose(p.x, p.y, _yaw(msg.pose.pose.orientation))
            stamp_s = (msg.header.stamp.sec
                       + msg.header.stamp.nanosec * 1e-9)
            transition = self._core.observe(pose, stamp_s)
            if transition is None:
                return
            self._pending_transition = transition
            self._pending = self._client.call_async(
                SetBool.Request(data=transition.dropout))
            self._pending.add_done_callback(self._on_service_done)

        def _on_service_done(self, future) -> None:
            transition = self._pending_transition
            self._pending_transition = None
            self._pending = None
            try:
                response = future.result()
            except Exception:
                # Failed service response: clear pending state and let the next
                # true pose retry the same transition (phase did not advance).
                return
            if not response.success:
                # Service refused: do not commit; retry on the next pose.
                return
            self._core.commit(transition)
            applied_s = self.get_clock().now().nanoseconds / 1e9
            msg = String()
            msg.data = json.dumps(transition.payload(applied_t_s=applied_s))
            self._window_pub.publish(msg)
            self.get_logger().info(
                f"dropout window {'opened' if transition.dropout else 'closed'}")

    rclpy.init()
    node = FieldDropoutControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
