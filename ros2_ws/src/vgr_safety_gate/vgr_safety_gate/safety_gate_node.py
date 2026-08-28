"""Real-robot safety gate ROS2 node.

Reads parameters, subscribes to topics, and delegates safety logic to SafetyGateCore.
"""
from __future__ import annotations

import json
import math

from gazebo_sim.nodes.safety_gate import (
    SafetyGateCore,
    _yaw_from_quaternion,
    _stamp_to_seconds,
    parse_obstacles_json,
)
from safety_sim.filters import make_filter
from vgr_core.motion import DiffDriveParams
from safety_sim.scenarios.basic import ARENA
from vgr_core.safety import Twist, Pose


def parse_geofence(values: list[float] | str) -> tuple[tuple[float, float], ...]:
    if isinstance(values, str):
        s = values.strip()
        if s.startswith('[') and s.endswith(']'):
            s = s[1:-1]
        val_list = [float(x.strip()) for x in s.split(',') if x.strip()]
    else:
        val_list = [float(x) for x in values]

    if len(val_list) % 2 != 0:
        raise ValueError(f"Geofence coordinate list must have an even length, got {len(val_list)}")
    num_points = len(val_list) // 2
    if num_points < 3:
        raise ValueError(f"Geofence must have at least 3 points (6 values), got {num_points} points ({len(val_list)} values)")
    return tuple((val_list[i], val_list[i + 1]) for i in range(0, len(val_list), 2))


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import (
        PoseStamped, PoseWithCovarianceStamped, Twist as RosTwist)
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    class SafetyGateNode(Node):
        def __init__(self) -> None:
            super().__init__("safety_gate")
            self.declare_parameter("filter_name", "safe_apf")
            self.declare_parameter("max_v_mps", 0.15)
            self.declare_parameter("max_omega_rad_s", 1.5)
            self.declare_parameter("nav_timeout_s", 0.2)
            self.declare_parameter("control_hz", 20.0)

            # Default geofence
            default_geofence = [float(x) for point in ARENA for x in point]
            self.declare_parameter("geofence", default_geofence)

            self.declare_parameter("pose_topic", "/aruco/pose")
            # "aruco" consumes direct vision; "fused" consumes `/pose_fused`,
            # which combines continuous odometry with timestamped corrections.
            self.declare_parameter("pose_source", "aruco")
            self.declare_parameter("nav_topic", "/cmd_vel_nav")
            self.declare_parameter("cmd_out_topic", "/cmd_vel_safe")
            self.declare_parameter("odom_topic", "/odom")
            # R3 等情境需要的固定 goal 與 static obstacles（spec R3 虛擬 geofence）。
            self.declare_parameter("fixed_goal_enabled", False)
            self.declare_parameter("goal_x", 0.0)
            self.declare_parameter("goal_y", 0.0)
            self.declare_parameter("obstacles_json", "")
            # 盲走預算（2026-07-14 安全策略檢討）：視覺丟失後改用
            # odom 推算，超過距離/時間預算才 fail-closed。0 = 舊行為。
            self.declare_parameter("blind_max_dist_m", 0.5)
            self.declare_parameter("blind_max_s", 5.0)
            self.declare_parameter("aruco_fresh_s", 0.4)
            # safe_apf influence band; <=0 uses the filter default.
            self.declare_parameter("apf_influence_m", 0.0)
            # d_safe = robot_radius + extra_safe. Negative values use filter defaults.
            self.declare_parameter("robot_radius_m", -1.0)
            self.declare_parameter("apf_extra_safe_m", -1.0)
            self.declare_parameter("cbf_alpha", -1.0)
            self.declare_parameter("cbf_buffer_m", -1.0)

            filter_name = str(self.get_parameter("filter_name").value)
            max_v_mps = float(self.get_parameter("max_v_mps").value)
            max_omega_rad_s = float(self.get_parameter("max_omega_rad_s").value)
            nav_timeout_s = float(self.get_parameter("nav_timeout_s").value)
            control_hz = float(self.get_parameter("control_hz").value)

            geofence_raw = self.get_parameter("geofence").value
            geofence_tuple = parse_geofence(geofence_raw)

            pose_topic = str(self.get_parameter("pose_topic").value)
            nav_topic = str(self.get_parameter("nav_topic").value)
            cmd_out_topic = str(self.get_parameter("cmd_out_topic").value)
            odom_topic = str(self.get_parameter("odom_topic").value)

            filter_kwargs = {}
            apf_influence_m = float(self.get_parameter("apf_influence_m").value)
            if filter_name == "safe_apf" and apf_influence_m > 0.0:
                filter_kwargs["influence_m"] = apf_influence_m
            apf_extra_safe_m = float(self.get_parameter("apf_extra_safe_m").value)
            if filter_name == "safe_apf" and apf_extra_safe_m >= 0.0:
                filter_kwargs["extra_safe_m"] = apf_extra_safe_m
            # E1 等激進度對比需要把模擬校準的 cbf 參數帶上車（2026-07-19）
            cbf_alpha = float(self.get_parameter("cbf_alpha").value)
            if filter_name == "cbf" and cbf_alpha > 0.0:
                filter_kwargs["alpha"] = cbf_alpha
            cbf_buffer_m = float(self.get_parameter("cbf_buffer_m").value)
            if filter_name == "cbf" and cbf_buffer_m >= 0.0:
                filter_kwargs["buffer_m"] = cbf_buffer_m

            core_kwargs = {}
            robot_radius_m = float(self.get_parameter("robot_radius_m").value)
            if robot_radius_m > 0.0:
                core_kwargs["robot_radius_m"] = robot_radius_m

            fixed_goal: tuple[float, float] | None = None
            if bool(self.get_parameter("fixed_goal_enabled").value):
                fixed_goal = (
                    float(self.get_parameter("goal_x").value),
                    float(self.get_parameter("goal_y").value),
                )
            obstacles = parse_obstacles_json(
                str(self.get_parameter("obstacles_json").value))

            self._core = SafetyGateCore(
                make_filter(filter_name, **filter_kwargs),
                **core_kwargs,
                max_v_mps=max_v_mps,
                max_omega_rad_s=max_omega_rad_s,
                control_hz=control_hz,
                nav_timeout_s=nav_timeout_s,
                geofence=geofence_tuple,
                aruco_fresh_s=float(self.get_parameter("aruco_fresh_s").value),
                blind_max_dist_m=float(self.get_parameter("blind_max_dist_m").value),
                blind_max_s=float(self.get_parameter("blind_max_s").value),
                fixed_goal=fixed_goal,
                obstacles=obstacles,
            )

            self._cmd_pub = self.create_publisher(RosTwist, cmd_out_topic, 10)
            self._status_pub = self.create_publisher(String, "/safety_gate/status", 10)

            self.create_subscription(RosTwist, nav_topic, self._on_nav, 10)
            pose_source = str(self.get_parameter("pose_source").value)
            if pose_source == "fused":
                self.create_subscription(
                    PoseWithCovarianceStamped, "/pose_fused", self._on_fused, 10)
                self.get_logger().info("pose source: /pose_fused（odom 主幹）")
            else:
                self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)

            if odom_topic != "":
                self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

            self.create_timer(1.0 / control_hz, self._on_timer)

        def _on_nav(self, msg: RosTwist) -> None:
            now_s = self.get_clock().now().nanoseconds / 1e9
            self._core.update_nav(Twist(msg.linear.x, msg.angular.z), stamp_s=now_s)

        def _on_pose(self, msg: PoseStamped) -> None:
            p = msg.pose.position
            self._core.update_aruco_pose(
                Pose(p.x, p.y, _yaw_from_quaternion(msg.pose.orientation)),
                stamp_s=_stamp_to_seconds(msg.header.stamp),
            )

        def _on_fused(self, msg) -> None:
            p = msg.pose.pose.position
            drift_m = math.sqrt(max(0.0, float(msg.pose.covariance[0])))
            corr_age_raw = float(msg.pose.covariance[35])
            corr_age_s = corr_age_raw if corr_age_raw >= 0.0 else math.inf
            self._core.update_fused_pose(
                Pose(p.x, p.y, _yaw_from_quaternion(msg.pose.pose.orientation)),
                drift_m=drift_m,
                corr_age_s=corr_age_s,
                stamp_s=_stamp_to_seconds(msg.header.stamp),
            )

        def _on_odom(self, msg: Odometry) -> None:
            v = msg.twist.twist.linear.x
            omega = msg.twist.twist.angular.z
            params = DiffDriveParams()
            w = params.wheel_base_m
            v_l = v - omega * w / 2.0
            v_r = v + omega * w / 2.0
            self._core.update_wheel_feedback(v_l, v_r)
            p = msg.pose.pose.position
            self._core.update_odom_pose(
                Pose(p.x, p.y, _yaw_from_quaternion(msg.pose.pose.orientation)),
                stamp_s=_stamp_to_seconds(msg.header.stamp),
            )

        def _on_timer(self) -> None:
            now_s = self.get_clock().now().nanoseconds / 1e9
            out = self._core.tick(now_s)
            cmd = RosTwist()
            cmd.linear.x = out.cmd.v
            cmd.angular.z = out.cmd.omega
            self._cmd_pub.publish(cmd)
            status = String()
            status.data = json.dumps({"mode": out.mode, "debug": out.debug}, sort_keys=True)
            self._status_pub.publish(status)

    rclpy.init()
    node = SafetyGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
