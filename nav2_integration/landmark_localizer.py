"""Publish map-to-odom corrections from the fused localization source.

Nav2 receives the continuous fused localization chain rather than raw,
single-frame planar PnP observations.
"""
from __future__ import annotations

from nav2_integration.ros_helpers import quaternion_from_yaw, yaw_from_quaternion
from vgr_core.motion import Pose2D, map_to_odom


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException

    class LandmarkLocalizer(Node):
        def __init__(self) -> None:
            super().__init__("landmark_localizer")
            self._buffer = Buffer()
            self._listener = TransformListener(self._buffer, self)
            self._broadcaster = TransformBroadcaster(self)
            self._last_correction = None
            pose_source = str(self.declare_parameter("pose_source", "fused").value)
            if pose_source not in ("fused", "aruco"):
                raise ValueError(
                    f"pose_source must be 'fused' or 'aruco', got {pose_source!r}"
                )
            if pose_source == "fused":
                self.create_subscription(
                    PoseWithCovarianceStamped, "/pose_fused", self._on_fused_pose, 10
                )
            else:
                self.create_subscription(
                    PoseStamped, "/aruco/pose", self._on_aruco_pose, 10
                )
            self.create_timer(1.0 / 20.0, self._publish_latest)

        def _on_aruco_pose(self, msg: PoseStamped) -> None:
            self._on_pose(msg.pose, msg.header.frame_id)

        def _on_fused_pose(self, msg: PoseWithCovarianceStamped) -> None:
            self._on_pose(msg.pose.pose, msg.header.frame_id)

        def _on_pose(self, pose, frame_id: str) -> None:
            if frame_id != "map":
                self.get_logger().warning("ignoring localization pose outside map frame")
                return
            try:
                local = self._buffer.lookup_transform(
                    "odom", "base_link", Time(),
                    timeout=Duration(seconds=0.1),
                )
            except TransformException as exc:
                self.get_logger().warning(f"odom to base_link unavailable: {exc}")
                return
            mp = pose
            oq = local.transform.rotation
            correction = map_to_odom(
                Pose2D(mp.position.x, mp.position.y, yaw_from_quaternion(
                    mp.orientation.x, mp.orientation.y, mp.orientation.z, mp.orientation.w)),
                Pose2D(local.transform.translation.x, local.transform.translation.y,
                       yaw_from_quaternion(oq.x, oq.y, oq.z, oq.w)),
            )
            self._last_correction = correction

        def _publish_latest(self) -> None:
            correction = self._last_correction
            if correction is None:
                return
            qx, qy, qz, qw = quaternion_from_yaw(correction.theta)
            out = TransformStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "map"
            out.child_frame_id = "odom"
            out.transform.translation.x = correction.x
            out.transform.translation.y = correction.y
            out.transform.rotation.x = qx
            out.transform.rotation.y = qy
            out.transform.rotation.z = qz
            out.transform.rotation.w = qw
            self._broadcaster.sendTransform(out)

    rclpy.init()
    node = LandmarkLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
