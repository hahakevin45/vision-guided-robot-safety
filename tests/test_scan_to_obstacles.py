"""LaserScan → map frame 圓障礙轉換測試。

/visual_obstacles（base_link frame）的 LaserScan 命中點轉成 map frame 的
Circle 障礙，讓 SAPF 臂的 obstacles 與 DWB/RPP 同源（視覺量測）。
"""
from __future__ import annotations

import math

import pytest

from gazebo_sim.nodes.scan_to_obstacles import scan_to_obstacles


def test_scan_point_becomes_obstacle():
    # robot 在 (1.0, 0) 朝 +x：ranges[60] 正前方 1.0m → map 上 x≈2.0
    ranges = [math.inf] * 121
    ranges[60] = 1.0
    obstacles = scan_to_obstacles(
        (1.0, 0.0, 0.0), ranges=ranges,
        angle_min=-1.0, angle_increment=0.017453,
        min_range=0.1, max_range=3.0, obstacle_radius=0.05,
    )
    assert len(obstacles) == 1
    ob = obstacles[0]
    assert ob.x == pytest.approx(2.0, abs=0.1)
    assert ob.y == pytest.approx(0.0, abs=0.1)
    assert ob.radius == pytest.approx(0.05)


def test_out_of_range_points_ignored():
    # 距離超過 max_range=3.0 的命中點不產生障礙
    ranges = [math.inf] * 121
    ranges[60] = 4.0
    obstacles = scan_to_obstacles(
        (1.0, 0.0, 0.0), ranges=ranges,
        angle_min=-1.0, angle_increment=0.017453,
        min_range=0.1, max_range=3.0, obstacle_radius=0.05,
    )
    assert len(obstacles) == 0


def test_adjacent_points_merge():
    # 相鄰角度的兩個命中點距離 < merge_m → 合併成 1 個障礙
    ranges = [math.inf] * 121
    ranges[59] = 1.0
    ranges[61] = 1.0
    obstacles = scan_to_obstacles(
        (1.0, 0.0, 0.0), ranges=ranges,
        angle_min=-1.0, angle_increment=0.017453,
        min_range=0.1, max_range=3.0, obstacle_radius=0.05,
        merge_m=0.15,
    )
    assert len(obstacles) == 1


def test_robot_heading_rotation():
    # robot 在 (2.0, 1.0) 朝 -y（θ=-π/2）：正前方 0.7m → map 上 y≈0.3
    ranges = [math.inf] * 121
    ranges[60] = 0.7
    obstacles = scan_to_obstacles(
        (2.0, 1.0, -math.pi / 2.0), ranges=ranges,
        angle_min=-1.0, angle_increment=0.017453,
        min_range=0.1, max_range=3.0, obstacle_radius=0.05,
    )
    assert len(obstacles) == 1
    ob = obstacles[0]
    assert ob.y == pytest.approx(0.3, abs=0.1)
    assert ob.x == pytest.approx(2.0, abs=0.1)


def test_scan_to_obstacles_json_serializable():
    """scan_to_obstacles 結果可轉成 parse_obstacles_json 相容 JSON 並解析回同型別。

    發布端（/obstacles_measured）用 {"type":"circle",x,y,radius} 序列化，
    消費端（safety_gate.parse_obstacles_json）解析回 Circle；驗證 round-trip。
    """
    import json

    from gazebo_sim.nodes.safety_gate import parse_obstacles_json

    ranges = [math.inf] * 121
    ranges[60] = 1.0
    obstacles = scan_to_obstacles(
        (1.0, 0.0, 0.0), ranges=ranges,
        angle_min=-1.0, angle_increment=0.017453,
        min_range=0.1, max_range=3.0, obstacle_radius=0.05,
    )
    text = json.dumps([
        {"type": "circle", "x": ob.x, "y": ob.y, "radius": ob.radius}
        for ob in obstacles
    ])
    parsed = parse_obstacles_json(text)
    assert parsed == obstacles
