"""evaluate_blind_obstacle 的盲走-障礙比較評估測試。"""
import math

from gazebo_sim.evaluate_blind_obstacle import evaluate_blind_obstacle

BOX = {"x": 2.0, "y": 0.0, "size_x": 0.40, "size_y": 0.60}
GOAL = (3.2, 0.0)
D_SAFE = 0.28


def _true(t, x, y):
    return {"topic": "/sim/true_pose", "t": t, "true_pose": {"x": x, "y": y}}


def _status(t, drift, est_x=None, est_y=None):
    debug = {"pose_drift_m": drift}
    if est_x is not None:
        debug["estimated_x"] = est_x
    if est_y is not None:
        debug["estimated_y"] = est_y
    return {"topic": "/safety_gate/status", "t": t, "mode": "blind", "debug": debug}


def test_error_inside_inflated_radius_safe():
    true_points = [{"t": 1.0, "x": 0.5}, {"t": 2.0, "x": 1.0}, {"t": 3.0, "x": 1.5}]
    drift = [0.0, 0.3, 0.5]
    rows = [_true(tp["t"], tp["x"], 0.0) for tp in true_points]
    rows += [
        _status(tp["t"], d, est_x=tp["x"] - 0.1 * d, est_y=0.0)
        for tp, d in zip(true_points, drift)
    ]
    result = evaluate_blind_obstacle(rows, box=BOX, goal=GOAL, d_safe=D_SAFE)

    assert result["max_est_error_m"] <= max(result["radius_series"])
    assert result["error_covered_by_inflation"] is True
    # 誤差 = |0.1 * drift|，最大 0.05。
    assert math.isclose(result["max_est_error_m"], 0.05)


def test_error_exceeds_fixed_radius_penetration():
    true_points = [{"t": 1.0, "x": 0.5}, {"t": 2.0, "x": 1.0}, {"t": 3.0, "x": 1.5}]
    rows = [_true(tp["t"], tp["x"], 0.0) for tp in true_points]
    rows += [
        _status(tp["t"], 0.2, est_x=tp["x"] - 0.35, est_y=0.0)
        for tp in true_points
    ]
    result = evaluate_blind_obstacle(rows, box=BOX, goal=GOAL, d_safe=D_SAFE)

    assert result["max_est_error_m"] > 0.28
    assert result["penetration_fixed_radius_m"] > 0
    # 誤差 = 0.35，穿透 = 0.35 − 0.28。
    assert math.isclose(result["penetration_fixed_radius_m"], 0.07)


def test_true_clearance_uses_box():
    rows = [_true(1.0, 1.8, 0.0)]
    result = evaluate_blind_obstacle(rows, box=BOX, goal=GOAL, d_safe=D_SAFE)

    # (1.8, 0) 在箱體左緣上，淨空 = 0 − 0.23。
    assert math.isclose(result["min_true_clearance_m"], -0.23)
    assert result["collided"] is True


def test_nonfinite_startup_estimate_is_excluded():
    rows = [
        _true(1.0, 0.5, 0.0),
        _status(1.0, 0.0, est_x=math.nan, est_y=math.nan),
        _true(2.0, 1.0, 0.0),
        _status(2.0, 0.2, est_x=0.9, est_y=0.0),
    ]

    result = evaluate_blind_obstacle(rows, box=BOX, goal=GOAL, d_safe=D_SAFE)

    assert math.isclose(result["max_est_error_m"], 0.1)
    assert len(result["radius_series"]) == 1
    assert math.isclose(result["radius_series"][0], 0.48)
