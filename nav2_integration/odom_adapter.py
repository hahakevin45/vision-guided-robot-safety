"""Normalize Gazebo ground-truth odometry into the Nav2 odometry contract."""
from __future__ import annotations


def main() -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    class GroundTruthOdomAdapter(Node):
        def __init__(self) -> None:
            super().__init__("ground_truth_odom_adapter")
            self._publisher = self.create_publisher(Odometry, "/odom", 20)
            self._tf = TransformBroadcaster(self)
            self.create_subscription(Odometry, "/sim/true_pose_raw", self._on_odom, 20)

        def _on_odom(self, source: Odometry) -> None:
            out = Odometry()
            out.header = source.header
            out.header.frame_id = "odom"
            out.child_frame_id = "base_link"
            out.pose = source.pose
            out.twist = source.twist
            self._publisher.publish(out)

            transform = TransformStamped()
            transform.header = out.header
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = out.pose.pose.position.x
            transform.transform.translation.y = out.pose.pose.position.y
            transform.transform.translation.z = out.pose.pose.position.z
            transform.transform.rotation = out.pose.pose.orientation
            self._tf.sendTransform(transform)

    rclpy.init()
    node = GroundTruthOdomAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
