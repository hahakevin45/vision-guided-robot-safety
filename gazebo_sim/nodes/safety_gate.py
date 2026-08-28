"""Safety gate ROS2 節點（薄包裝）。

SafetyGateCore 現在位於 vgr_core.safety.safety_gate，此檔只做 ROS topic 包裝。
"""
from __future__ import annotations

import json
import math

# ARENA is a simulation scenario constant; keep importing from safety_sim.scenarios.basic
from safety_sim.scenarios.basic import ARENA  # noqa: N812

from vgr_core.geometry.arena_geometry import Box2D
from vgr_core.motion import DiffDriveParams
from vgr_core.safety import Circle, Pose, Twist
from vgr_core.safety.safety_gate import (
    DEFAULT_ROBOT_RADIUS_M,
    GateOutput,
    SafetyGateCore,
)
from safety_sim.filters import make_filter


def parse_obstacles_json(text: str) -> tuple[Circle | Box2D, ...]:
    """Parse validated static obstacles from JSON; invalid input raises.

    `text` is a JSON list of entries. Circle: {"x": .., "y": .., "radius": ..}
    (legacy, no "type"). Box: {"type": "box", "x": .., "y": ..,
    "size_x": .., "size_y": ..}. Any non-finite field or non-positive
    dimension fails loudly: an invalid map must never silently become an
    empty map.
    """
    if not text.strip():
        return ()
    data = json.loads(text)
    if not isinstance(data, list):
        raise ValueError("obstacles_json must be a JSON list")
    obstacles: list[Circle | Box2D] = []
    for entry in data:
        if not isinstance(entry, dict):
            raise ValueError(f"obstacle entry is not an object: {entry!r}")
        kind = entry.get("type", "circle")
        if kind == "circle":
            x, y, radius = entry.get("x"), entry.get("y"), entry.get("radius")
            values = [v for v in (x, y, radius) if v is not None]
            if len(values) != 3 or not all(
                isinstance(v, (int, float)) and math.isfinite(float(v))
                for v in values
            ):
                raise ValueError(f"non-finite or missing obstacle fields: {entry!r}")
            if float(radius) <= 0.0:
                raise ValueError(f"non-positive obstacle radius: {entry!r}")
            obstacles.append(Circle(float(x), float(y), float(radius)))
        elif kind == "box":
            x, y = entry.get("x"), entry.get("y")
            sx, sy = entry.get("size_x"), entry.get("size_y")
            values = [v for v in (x, y, sx, sy) if v is not None]
            if len(values) != 4 or not all(
                isinstance(v, (int, float)) and math.isfinite(float(v))
                for v in values
            ):
                raise ValueError(f"non-finite or missing obstacle fields: {entry!r}")
            if float(sx) <= 0.0 or float(sy) <= 0.0:
                raise ValueError(f"non-positive box size: {entry!r}")
            obstacles.append(Box2D(float(x), float(y), float(sx), float(sy)))
        else:
            raise ValueError(f"unknown obstacle type: {kind!r}")
    return tuple(obstacles)


def plan_points_from_path(frame_id: str, xy_points) -> tuple[tuple[float, float], ...] | None:
    """Path points when frame is `map`; None when the frame is rejected.

    A plan in any other frame is never adopted, so a misconfigured TF tree
    cannot feed a goal in the wrong coordinate system.
    """
    if frame_id != "map":
        return None
    return tuple((float(x), float(y)) for x, y in xy_points)


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    """啟動 ROS2 節點；ROS 型別與 rclpy 僅在薄包裝中使用。"""
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist as RosTwist
    from nav_msgs.msg import Odometry, Path as NavPath
    from rclpy.node import Node
    from std_msgs.msg import String

    class SafetyGateNode(Node):
        """ROS topic 包裝；安全決策委派給 `SafetyGateCore`。"""

        def __init__(self) -> None:
            super().__init__("safety_gate")
            self.declare_parameter("filter_name", "safe_apf")
            self.declare_parameter("max_v_mps", 0.15)
            self.declare_parameter("max_omega_rad_s", 1.5)
            self.declare_parameter("nav_timeout_s", 0.2)
            self.declare_parameter("fixed_goal_enabled", False)
            self.declare_parameter("goal_x", 0.0)
            self.declare_parameter("goal_y", 0.0)
            self.declare_parameter("plan_lookahead_m", 0.35)
            self.declare_parameter("plan_timeout_s", 0.5)
            self.declare_parameter("obstacles_json", "")
            # R3 公平性：CBF 需用 shared clearance（buffer 0.05），非預設 0.08。
            self.declare_parameter("cbf_alpha", -1.0)
            self.declare_parameter("cbf_buffer_m", -1.0)
            # 盲走預算（實驗可設大：取消距離限制 → 安全半徑隨信心度增長）
            self.declare_parameter("blind_max_dist_m", 0.5)
            self.declare_parameter("blind_max_s", 5.0)
            # SAPF filter kwargs：忽略定位漂移（R3 漂移實驗）與固定安全半徑。
            self.declare_parameter("filter_kwargs_ignore_pose_drift", False)
            self.declare_parameter("filter_kwargs_fixed_d_safe_m", -1.0)
            # Humble rclpy：空 list 一律宣告為 BYTE_ARRAY（descriptor 被忽略），
            # CLI 傳 DOUBLE_ARRAY 會型別衝突而崩潰。用 [0.0] 哨兵：
            # 非空 list → DOUBLE_ARRAY；[0.0] 單元素 = 未設定（core 預設 ARENA）。
            self.declare_parameter("geofence", [0.0])
            filter_name = str(self.get_parameter("filter_name").value)
            fixed_goal: tuple[float, float] | None = None
            if bool(self.get_parameter("fixed_goal_enabled").value):
                fixed_goal = (
                    float(self.get_parameter("goal_x").value),
                    float(self.get_parameter("goal_y").value),
                )
            obstacles = parse_obstacles_json(
                str(self.get_parameter("obstacles_json").value)
            )
            core_kwargs = {}
            geofence_vals = list(self.get_parameter("geofence").value)
            if geofence_vals != [0.0]:
                if len(geofence_vals) % 2 != 0 or len(geofence_vals) < 6:
                    raise ValueError(
                        f"geofence needs >=3 (x,y) points, got "
                        f"{len(geofence_vals)//2}")
                core_kwargs["geofence"] = tuple(
                    (float(geofence_vals[i]), float(geofence_vals[i + 1]))
                    for i in range(0, len(geofence_vals), 2))
            filter_kwargs = {}
            cbf_alpha = float(self.get_parameter("cbf_alpha").value)
            if filter_name == "cbf" and cbf_alpha > 0.0:
                filter_kwargs["alpha"] = cbf_alpha
            cbf_buffer_m = float(self.get_parameter("cbf_buffer_m").value)
            if filter_name == "cbf" and cbf_buffer_m >= 0.0:
                filter_kwargs["buffer_m"] = cbf_buffer_m
            if bool(self.get_parameter("filter_kwargs_ignore_pose_drift").value):
                filter_kwargs["ignore_pose_drift"] = True
            fds = float(self.get_parameter("filter_kwargs_fixed_d_safe_m").value)
            if fds > 0.0:
                filter_kwargs["fixed_d_safe_m"] = fds
            self._core = SafetyGateCore(
                make_filter(filter_name, **filter_kwargs),
                max_v_mps=float(self.get_parameter("max_v_mps").value),
                max_omega_rad_s=float(self.get_parameter("max_omega_rad_s").value),
                nav_timeout_s=float(self.get_parameter("nav_timeout_s").value),
                fixed_goal=fixed_goal,
                obstacles=obstacles,
                plan_lookahead_m=float(self.get_parameter("plan_lookahead_m").value),
                plan_timeout_s=float(self.get_parameter("plan_timeout_s").value),
                blind_max_dist_m=float(self.get_parameter("blind_max_dist_m").value),
                blind_max_s=float(self.get_parameter("blind_max_s").value),
                **core_kwargs,
            )
            self._cmd_pub = self.create_publisher(RosTwist, "/cmd_vel_safe", 10)
            self._status_pub = self.create_publisher(String, "/safety_gate/status", 10)
            self.create_subscription(RosTwist, "/cmd_vel_nav", self._on_nav, 10)
            self.create_subscription(PoseStamped, "/aruco/pose", self._on_pose, 10)
            self.create_subscription(NavPath, "/plan", self._on_plan, 10)
            # 動態障礙（視覺量測，資訊對等）：String JSON → core 更新。
            self.create_subscription(
                String, "/obstacles_measured", self._on_obstacles_measured, 10)
            # dead-reckoning（盲走）需要 odom：車走出視覺覆蓋區後，
            # core 用最後 anchor + odom 增量推算，直到 blind budget 超額。
            self.create_subscription(Odometry, "/odom", self._on_odom, 10)
            self.create_timer(1.0 / 20.0, self._on_timer)

        def _on_nav(self, msg: RosTwist) -> None:
            now_s = self.get_clock().now().nanoseconds / 1e9
            self._core.update_nav(Twist(msg.linear.x, msg.angular.z), stamp_s=now_s)

        def _on_pose(self, msg: PoseStamped) -> None:
            p = msg.pose.position
            self._core.update_aruco_pose(
                Pose(p.x, p.y, _yaw_from_quaternion(msg.pose.orientation)),
                stamp_s=_stamp_to_seconds(msg.header.stamp),
            )

        def _on_odom(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self._core.update_odom_pose(
                Pose(p.x, p.y, _yaw_from_quaternion(q)),
                stamp_s=_stamp_to_seconds(msg.header.stamp),
            )

        def _on_plan(self, msg) -> None:
            points = plan_points_from_path(
                msg.header.frame_id,
                [(p.pose.position.x, p.pose.position.y) for p in msg.poses],
            )
            if points is None:
                return  # 非 map frame：不採納，保留先前 plan（或無 plan）
            self._core.update_plan(points, stamp_s=_stamp_to_seconds(msg.header.stamp))

        def _on_obstacles_measured(self, msg: String) -> None:
            """動態障礙（視覺量測）：JSON → obstacles → core 更新。"""
            obstacles = parse_obstacles_json(msg.data)
            self._core.update_obstacles(obstacles)

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
