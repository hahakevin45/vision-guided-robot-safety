"""`evaluate_local_detour` 的接受度測試。

驗證純函式對繞行成功、正面頂箱碰撞、以及淨空使用箱體幾何（含車半徑）
三個行為的計算。
"""
import math
import os
import subprocess
from pathlib import Path

import pytest

from gazebo_sim.evaluate_local_detour import evaluate_detour_trace

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPO_ROOT / "gazebo_sim" / "scripts" / "run_local_detour_compare.sh"

BOX = {"x": 2.0, "y": 0.0, "size_x": 0.40, "size_y": 0.60}
GOAL = (3.5, 0.0)


def _rows(points):
    return [
        {"topic": "/sim/true_pose", "t": t, "true_pose": {"x": x, "y": y, "theta": 0.0}}
        for t, x, y in points
    ]


def test_detour_around_box_succeeds():
    # 車心繞過箱右側（箱 x∈[1.8,2.2]），最後到達 goal=(3.5,0)。
    rows = _rows([
        (0, 0.7, 0.0),
        (1, 1.5, 0.0),
        (2, 1.8, 0.6),
        (3, 2.4, 0.6),
        (4, 2.9, 0.3),
        (5, 3.5, 0.0),
    ])

    report = evaluate_detour_trace(rows, box=BOX, goal=GOAL)

    assert report["reached_goal"] is True
    assert report["collided"] is False
    assert report["max_abs_y"] >= 0.5
    assert report["min_clearance_m"] > 0.0
    assert report["final_goal_dist_m"] <= 0.15
    assert report["arrive_t_s"] == 5


def test_head_on_stall_fails():
    # 直行頂箱（車心停在箱前表面 x=1.8），未繞行、未到達 goal。
    rows = _rows([
        (0, 0.7, 0.0),
        (1, 1.3, 0.0),
        (2, 1.8, 0.0),
        (3, 1.8, 0.0),
        (4, 1.8, 0.0),
        (5, 1.8, 0.0),
    ])

    report = evaluate_detour_trace(rows, box=BOX, goal=GOAL)

    assert report["reached_goal"] is False
    assert report["collided"] is True
    assert report["arrive_t_s"] is None


def test_clearance_uses_box_geometry():
    # 車心 (1.8,0) 正好在箱前表面：箱距離 = 0，扣車半徑後 = −0.23。
    rows = _rows([(0, 1.8, 0.0)])

    report = evaluate_detour_trace(rows, box=BOX, goal=GOAL)

    assert report["min_clearance_m"] == pytest.approx(-0.23)


def test_empty_trace_defaults():
    report = evaluate_detour_trace([], box=BOX, goal=GOAL)

    assert report["reached_goal"] is False
    assert report["collided"] is False
    assert report["min_clearance_m"] == math.inf
    assert report["max_abs_y"] == 0.0
    assert report["final_goal_dist_m"] == math.inf
    assert report["arrive_t_s"] is None


def test_detour_runner_dry_run():
    # DRY_RUN：合規 arm 立即回 0 並印出 DRY_RUN 行，不啟動任何節點。
    out_dir = "/tmp/local_detour_compare_dry"
    env = dict(os.environ, DRY_RUN="YES")
    res = subprocess.run(
        ["bash", str(RUNNER), "--arm", "sapf", "--out", out_dir],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
    )
    assert res.returncode == 0
    assert "DRY_RUN" in res.stdout
    assert f"arm=sapf" in res.stdout
    assert f"out={out_dir}" in res.stdout

    # 不合規 arm：arg parse 拒絕，回 2。
    bad = subprocess.run(
        ["bash", str(RUNNER), "--arm", "bogus", "--out", "/tmp/local_detour_compare_bad"],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
    )
    assert bad.returncode == 2


def test_runner_uses_source_nav2_params_not_stale_install_copy():
    script = RUNNER.read_text(encoding="utf-8")
    expected = (
        'params_file:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/'
        'config/nav2_params.yaml"'
    )
    assert expected in script


def test_runner_has_one_visual_measurement_owner():
    script = RUNNER.read_text(encoding="utf-8")
    assert "python3 -m gazebo_sim.nodes.scan_to_obstacles" not in script


def test_runner_uses_blind_navfn_with_full_dwb_stack() -> None:
    script = RUNNER.read_text(encoding="utf-8")
    assert "navigation.launch.py" in script
    assert "controller_plugin:=dwb" in script
    assert "controller_only.launch.py" not in script
    assert "follow_path_client" not in script
    assert "straight_plan_publisher" in script
