"""gazebo_sim/nodes/scan_to_obstacles.py
LaserScan（/visual_obstacles, base_link）→ map frame 圓障礙（Circle）。
SAPF 臂的 obstacles 與 DWB/RPP 同源（視覺量測），消除資訊不對等。
"""
import math
from vgr_core.safety import Circle


def scan_to_obstacles(robot_pose, *, ranges, angle_min, angle_increment,
                      min_range, max_range, obstacle_radius,
                      merge_m=0.15):
    """scan 命中點 → map frame 圓障礙（鄰近點合併）。

    robot_pose = (x, y, theta) map frame。ranges: 每角度距離。
    命中點 (min_range <= r <= max_range) 轉 map 座標；
    與既有圓障礙距離 <= merge_m 時合併（平均），否則新增。
    """
    x, y, theta = robot_pose
    obstacles = []
    for i, r in enumerate(ranges):
        if not (min_range <= r <= max_range):
            continue
        a = theta + angle_min + i * angle_increment
        px, py = x + r * math.cos(a), y + r * math.sin(a)
        merged = False
        for ob in obstacles:
            if math.hypot(px - ob.x, py - ob.y) <= merge_m:
                obstacles[obstacles.index(ob)] = Circle(
                    (ob.x + px) / 2.0, (ob.y + py) / 2.0, obstacle_radius)
                merged = True
                break
        if not merged:
            obstacles.append(Circle(px, py, obstacle_radius))
    return tuple(obstacles)


def main() -> None:  # pragma: no cover - ROS node
    """發布 /obstacles_measured（String JSON）：LaserScan → map frame 圓障礙。

    車體位姿從 /sim/true_pose_raw（Odometry）讀取；/visual_obstacles
    （base_link frame LaserScan）命中點轉成 map frame Circle 障礙，以
    parse_obstacles_json 相容 JSON 發布，讓 SAPF 臂的 obstacles 與
    DWB/RPP 同源（視覺量測）。沒有 pose 或 scan 時不發布。
    """
    import json

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    rclpy.init()

    class ScanToObstaclesNode(Node):
        def __init__(self) -> None:
            super().__init__("scan_to_obstacles")
            self.declare_parameter("update_hz", 10.0)
            self.declare_parameter("pose_topic", "/sim/true_pose_raw")
            self.declare_parameter("out_topic", "/obstacles_measured")
            self.declare_parameter("min_range", 0.1)
            self.declare_parameter("max_range", 3.0)
            self.declare_parameter("obstacle_radius", 0.05)
            self.declare_parameter("merge_m", 0.15)
            self._pose: tuple[float, float, float] | None = None
            self._scan: tuple[list[float], float, float] | None = None
            self._pub = self.create_publisher(
                String, str(self.get_parameter("out_topic").value), 10)
            self.create_subscription(
                Odometry, str(self.get_parameter("pose_topic").value),
                self._on_pose, 10)
            self.create_subscription(
                LaserScan, "/visual_obstacles", self._on_scan,
                qos_profile_sensor_data)
            hz = float(self.get_parameter("update_hz").value)
            self.create_timer(1.0 / hz, self._on_timer)

        def _on_pose(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._pose = (p.x, p.y, math.atan2(siny, cosy))

        def _on_scan(self, msg: LaserScan) -> None:
            self._scan = (list(msg.ranges), msg.angle_min, msg.angle_increment)

        def _on_timer(self) -> None:
            if self._pose is None or self._scan is None:
                return
            ranges, angle_min, angle_increment = self._scan
            obstacles = scan_to_obstacles(
                self._pose, ranges=ranges,
                angle_min=angle_min, angle_increment=angle_increment,
                min_range=float(self.get_parameter("min_range").value),
                max_range=float(self.get_parameter("max_range").value),
                obstacle_radius=float(self.get_parameter("obstacle_radius").value),
                merge_m=float(self.get_parameter("merge_m").value),
            )
            payload = json.dumps([
                {"type": "circle", "x": ob.x, "y": ob.y, "radius": ob.radius}
                for ob in obstacles
            ])
            msg = String()
            msg.data = payload
            self._pub.publish(msg)

    node = ScanToObstaclesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
