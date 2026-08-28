"""繞行比較評估：從 trace 計算繞行成功/碰撞/時間/路徑。

輸入是 Gazebo 模擬的 `/sim/true_pose` 行（list[dict] 或 JSONL 路徑），
搭配單一方箱（box）與目標點（goal），輸出逐點繞行指標。

本模組為**純函式**，不做 IO、不讀參數、不 import 測試。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from vgr_core.geometry.arena_geometry import Box2D, box_distance_to_point

ROBOT_RADIUS = 0.23
GOAL_TOLERANCE_M = 0.15

_POSE_TOPIC = "/sim/true_pose"


def _load_rows(rows) -> list[dict]:
    """接受 list[dict] 或 JSONL 路徑，一律回傳 list[dict]。"""
    if isinstance(rows, (str, Path)):
        path = Path(rows)
        return [
            json.loads(line)
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    return rows


def _pose_points(rows: list[dict]) -> list[tuple[float, float, float]]:
    """抽取出 `/sim/true_pose` 行的 (t, x, y)。"""
    points = []
    for row in rows:
        if row.get("topic") == _POSE_TOPIC and "true_pose" in row:
            pose = row["true_pose"]
            points.append((float(row.get("t", 0.0)), float(pose["x"]), float(pose["y"])))
    return points


def evaluate_detour_trace(rows, *, box: dict, goal: tuple[float, float]) -> dict:
    """計算單一方箱繞行 trace 的指標。

    rows: list[dict] 或 JSONL 路徑。
    /sim/true_pose row: {"t", "true_pose": {"x", "y", "theta"}}
    box: {"x", "y", "size_x", "size_y"}；goal: (gx, gy)。

    回傳：
    - reached_goal: 有 true 點距 goal ≤ 0.15
    - collided: min(箱距離 − 0.23) < 0
    - min_clearance_m: min(箱距離 − 0.23)
    - max_abs_y: max(|true_y|)
    - final_goal_dist_m: 最後點距 goal
    - arrive_t_s: 到達點 t 或 None

    空 trace：reached=False, collided=False, min_clearance=inf,
    max_abs_y=0, final_goal_dist=inf, arrive=None。
    """
    points = _pose_points(_load_rows(rows))
    box2d = Box2D(box["x"], box["y"], box["size_x"], box["size_y"])
    gx, gy = goal

    if not points:
        return {
            "reached_goal": False,
            "collided": False,
            "min_clearance_m": math.inf,
            "max_abs_y": 0.0,
            "final_goal_dist_m": math.inf,
            "arrive_t_s": None,
        }

    def _dist(x: float, y: float) -> float:
        return math.hypot(x - gx, y - gy)

    clearances = [box_distance_to_point(box2d, x, y) - ROBOT_RADIUS for _, x, y in points]
    min_clearance = min(clearances)

    arrive_t = None
    for t, x, y in points:
        if _dist(x, y) <= GOAL_TOLERANCE_M:
            arrive_t = t
            break

    _, fx, fy = points[-1]
    return {
        "reached_goal": any(_dist(x, y) <= GOAL_TOLERANCE_M for _, x, y in points),
        "collided": min_clearance < 0.0,
        "min_clearance_m": min_clearance,
        "max_abs_y": max(abs(y) for _, _, y in points),
        "final_goal_dist_m": _dist(fx, fy),
        "arrive_t_s": arrive_t,
    }
