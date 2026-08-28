"""safety_sim.world：2D 世界的幾何判定（ground truth 端，給 metrics 用）。"""
import math

from vgr_core.safety import Circle, Pose
from safety_sim.world import World

# 4m x 2m 矩形 geofence，車半徑 10cm。
FENCE = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))


def make_world(obstacles=()):
    return World(geofence=FENCE, obstacles=tuple(obstacles), robot_radius_m=0.10)


def test_center_is_inside_with_positive_clearance():
    w = make_world()
    p = Pose(2.0, 0.0, 0.0)
    assert w.contains(p)
    # 距最近邊界 1.0m，扣掉車半徑 0.1m。
    assert math.isclose(w.min_clearance(p), 0.9, abs_tol=1e-9)


def test_near_wall_clearance_shrinks_and_collides():
    w = make_world()
    assert w.min_clearance(Pose(3.95, 0.0, 0.0)) < 0.0   # 距右牆 5cm < 車半徑
    assert w.collided(Pose(3.95, 0.0, 0.0))
    assert not w.collided(Pose(3.5, 0.0, 0.0))


def test_outside_geofence_is_violation():
    w = make_world()
    p = Pose(4.5, 0.0, 0.0)
    assert not w.contains(p)
    assert w.collided(p)
    assert w.min_clearance(p) < 0.0


def test_circle_obstacle_clearance():
    w = make_world([Circle(2.0, 0.0, 0.2)])
    # 距障礙圓心 0.5m - 障礙半徑 0.2 - 車半徑 0.1 = 0.2m 淨空。
    assert math.isclose(w.min_clearance(Pose(1.5, 0.0, 0.0)), 0.2, abs_tol=1e-9)
    assert w.collided(Pose(1.75, 0.0, 0.0))   # 0.25 - 0.2 - 0.1 < 0


def test_empty_geofence_means_unbounded():
    w = World(geofence=(), obstacles=(), robot_radius_m=0.10)
    assert w.contains(Pose(100.0, 100.0, 0.0))
    assert w.min_clearance(Pose(100.0, 100.0, 0.0)) == math.inf
