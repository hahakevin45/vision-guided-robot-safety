"""Publish one marker-derived obstacle measurement for both local planners.

Once a box marker is visible, SAPF receives its Box2D geometry and Nav2
receives a dense PointCloud2 of the same boundary. A LaserScan is also
published for diagnostics. All representations share one detection event.
"""
from __future__ import annotations

import json
from collections.abc import Iterable
import math

from vgr_core.geometry.arena_geometry import Box2D


def box_ray_intersection(
    origin: tuple[float, float], angle_rad: float, box: Box2D
) -> float | None:
    """射線 (origin, angle) 與軸對齊箱的最短距離；未擊中/在箱內回 None。"""
    ox, oy = origin
    dx, dy = math.cos(angle_rad), math.sin(angle_rad)
    min_x, max_x, min_y, max_y = box.bounds
    # slab method（軸對齊矩形）
    t_near = 0.0
    t_far = math.inf
    for o, d, lo, hi in ((ox, dx, min_x, max_x), (oy, dy, min_y, max_y)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1 = (lo - o) / d
            t2 = (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            t_near = max(t_near, t1)
            t_far = min(t_far, t2)
            if t_near > t_far:
                return None
    if t_near <= 1e-9:
        return None  # 起點在箱內或貼邊
    return t_near


def box_to_laser_scan(
    robot_pose: tuple[float, float, float],
    box: Box2D,
    *,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    max_range: float,
) -> tuple[list[float], float, float]:
    """車體座標 LaserScan：每角度一條射線，擊中箱回距離，否則 inf。

    `robot_pose` = (x, y, theta) map frame。回傳 (ranges, angle_min, angle_increment)。
    """
    x, y, theta = robot_pose
    n = int(round((angle_max - angle_min) / angle_increment)) + 1
    ranges: list[float] = []
    for i in range(n):
        scan_angle = angle_min + i * angle_increment
        world_angle = theta + scan_angle
        dist = box_ray_intersection((x, y), world_angle, box)
        if dist is None or dist > max_range:
            ranges.append(math.inf)
        else:
            ranges.append(dist)
    return ranges, angle_min, angle_increment


def laser_scan_hits_world(
    robot_pose: tuple[float, float, float],
    ranges: list[float],
    *,
    angle_min: float,
    angle_increment: float,
    min_range: float,
    max_range: float,
) -> list[tuple[float, float, float]]:
    """Convert valid base_link scan hits to odom-frame XYZ points."""
    x, y, theta = robot_pose
    points: list[tuple[float, float, float]] = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not min_range <= distance <= max_range:
            continue
        angle = theta + angle_min + index * angle_increment
        points.append((
            x + distance * math.cos(angle),
            y + distance * math.sin(angle),
            0.0,
        ))
    return points


def detection_latched(previously_detected: bool, ranges: Iterable[float]) -> bool:
    """Retain a marker-derived object pose after its first visible ray."""
    return previously_detected or any(math.isfinite(distance) for distance in ranges)


def box_boundary_points(
    box: Box2D, *, spacing_m: float = 0.05
) -> list[tuple[float, float, float]]:
    """Sample the full marker-known box boundary in the odom plane."""
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be positive")

    min_x, max_x, min_y, max_y = box.bounds

    def samples(low: float, high: float) -> list[float]:
        count = max(1, math.ceil((high - low) / spacing_m))
        return [low + (high - low) * index / count for index in range(count + 1)]

    xs = samples(min_x, max_x)
    ys = samples(min_y, max_y)
    points = [(x, min_y, 0.0) for x in xs]
    points.extend((x, max_y, 0.0) for x in xs)
    points.extend((min_x, y, 0.0) for y in ys[1:-1])
    points.extend((max_x, y, 0.0) for y in ys[1:-1])
    return points


def boxes_to_json(boxes: Iterable[Box2D]) -> str:
    """Serialize marker-known boxes for SafetyGateCore.update_obstacles."""
    return json.dumps([
        {
            "type": "box",
            "x": box.x,
            "y": box.y,
            "size_x": box.size_x,
            "size_y": box.size_y,
        }
        for box in boxes
    ], separators=(",", ":"))


def main() -> None:  # pragma: no cover - ROS node
    """Publish equivalent Box2D, boundary-cloud, and scan representations.

    Parameters: obstacles_json, update_hz, max_range, fov_rad, and pose_topic.
    """

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String

    rclpy.init()

    class VisualObstacleScanNode(Node):
        def __init__(self) -> None:
            super().__init__("visual_obstacle_scan")
            self.declare_parameter("obstacles_json", "[]")
            self.declare_parameter("update_hz", 10.0)
            self.declare_parameter("max_range", 3.0)
            self.declare_parameter("fov_rad", 2.0)
            self.declare_parameter("pose_topic", "/sim/true_pose_raw")
            from gazebo_sim.nodes.safety_gate import parse_obstacles_json

            self._boxes = tuple(
                ob for ob in parse_obstacles_json(
                    str(self.get_parameter("obstacles_json").value))
                if isinstance(ob, Box2D))
            self._max_range = float(self.get_parameter("max_range").value)
            self._fov = float(self.get_parameter("fov_rad").value)
            self._pose: tuple[float, float, float] | None = None
            self._pose_stamp = None
            self._detected = False
            self._scan_pub = self.create_publisher(
                LaserScan, "/visual_obstacles", qos_profile_sensor_data)
            self._points_pub = self.create_publisher(
                PointCloud2, "/visual_obstacle_points", qos_profile_sensor_data)
            # Safety gate uses reliable depth-10 QoS for std_msgs/String.
            self._obstacles_pub = self.create_publisher(
                String, "/obstacles_measured", 10)
            self.create_subscription(
                Odometry, str(self.get_parameter("pose_topic").value),
                self._on_pose, 10)
            hz = float(self.get_parameter("update_hz").value)
            self.create_timer(1.0 / hz, self._on_timer)

        def _on_pose(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._pose = (p.x, p.y, math.atan2(siny, cosy))
            self._pose_stamp = msg.header.stamp

        def _on_timer(self) -> None:
            if self._pose is None or not self._boxes:
                return
            ranges, a_min, a_inc = box_to_laser_scan(
                self._pose, self._boxes[0],
                angle_min=-self._fov / 2.0,
                angle_max=self._fov / 2.0,
                angle_increment=math.radians(1.0),
                max_range=self._max_range,
            )
            scan = LaserScan()
            scan.header = Header()
            if self._pose_stamp is not None:
                scan.header.stamp = self._pose_stamp
            else:
                scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = "base_link"
            scan.angle_min = a_min
            scan.angle_max = -a_min
            scan.angle_increment = a_inc
            scan.range_min = 0.05
            scan.range_max = self._max_range
            scan.ranges = ranges
            self._scan_pub.publish(scan)

            point_header = Header()
            point_header.stamp = scan.header.stamp
            point_header.frame_id = "odom"
            self._detected = detection_latched(self._detected, ranges)
            points = (
                box_boundary_points(self._boxes[0])
                if self._detected else []
            )
            self._points_pub.publish(
                point_cloud2.create_cloud_xyz32(point_header, points))
            measured = String()
            measured.data = boxes_to_json(
                [self._boxes[0]] if self._detected else [])
            self._obstacles_pub.publish(measured)

    node = VisualObstacleScanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
