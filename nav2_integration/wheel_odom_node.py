"""Publish Nav2 odometry from Gazebo wheel joint positions."""
from __future__ import annotations

from vgr_core.motion import DifferentialOdometry, EncoderConfig
from nav2_integration.ros_helpers import joint_radians_to_counts, quaternion_from_yaw


LEFT_JOINT = "left_wheel_joint"
RIGHT_JOINT = "right_wheel_joint"


def main() -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster

    class WheelOdomNode(Node):
        def __init__(self) -> None:
            super().__init__("wheel_odom")
            self.declare_parameter("wheel_base_m", 0.165)
            self.declare_parameter("wheel_diameter_m", 0.065)
            self.declare_parameter("left_counts_per_rev", 750.0)
            self.declare_parameter("right_counts_per_rev", 749.0)
            self.declare_parameter("left_joint_sign", 1)
            self.declare_parameter("right_joint_sign", -1)
            self._left_cpr = float(self.get_parameter("left_counts_per_rev").value)
            self._right_cpr = float(self.get_parameter("right_counts_per_rev").value)
            self._left_joint_sign = int(self.get_parameter("left_joint_sign").value)
            self._right_joint_sign = int(self.get_parameter("right_joint_sign").value)
            # Joint radians are normalized by the adapter, so the core sees +/+ counts.
            self._odom = DifferentialOdometry(EncoderConfig(
                wheel_base_m=float(self.get_parameter("wheel_base_m").value),
                wheel_diameter_m=float(self.get_parameter("wheel_diameter_m").value),
                left_counts_per_rev=self._left_cpr,
                right_counts_per_rev=self._right_cpr,
                left_sign=1,
                right_sign=1,
            ))
            self._publisher = self.create_publisher(Odometry, "/odom", 20)
            self._tf = TransformBroadcaster(self)
            self._last_state = None
            self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
            self.create_timer(1.0 / 20.0, self._publish_latest)

        def _on_joints(self, msg: JointState) -> None:
            positions = dict(zip(msg.name, msg.position))
            if LEFT_JOINT not in positions or RIGHT_JOINT not in positions:
                return
            left, right = joint_radians_to_counts(
                left_rad=positions[LEFT_JOINT],
                right_rad=positions[RIGHT_JOINT],
                left_counts_per_rev=self._left_cpr,
                right_counts_per_rev=self._right_cpr,
                left_joint_sign=self._left_joint_sign,
                right_joint_sign=self._right_joint_sign,
            )
            stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
            try:
                self._last_state = self._odom.update(left, right, stamp_s)
            except ValueError as exc:
                self.get_logger().warning(str(exc))

        def _publish_latest(self) -> None:
            state = self._last_state
            if state is None:
                return
            now = self.get_clock().now()
            now_s = now.nanoseconds / 1e9
            moving = now_s - state.stamp_s <= 0.2
            qx, qy, qz, qw = quaternion_from_yaw(state.theta)
            out = Odometry()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "odom"
            out.child_frame_id = "base_link"
            out.pose.pose.position.x = state.x
            out.pose.pose.position.y = state.y
            out.pose.pose.orientation.x = qx
            out.pose.pose.orientation.y = qy
            out.pose.pose.orientation.z = qz
            out.pose.pose.orientation.w = qw
            out.twist.twist.linear.x = state.linear_mps if moving else 0.0
            out.twist.twist.angular.z = state.angular_rad_s if moving else 0.0
            self._publisher.publish(out)

            transform = TransformStamped()
            transform.header = out.header
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = state.x
            transform.transform.translation.y = state.y
            transform.transform.rotation = out.pose.pose.orientation
            self._tf.sendTransform(transform)

    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
