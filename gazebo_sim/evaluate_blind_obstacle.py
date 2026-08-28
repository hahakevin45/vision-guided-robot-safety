"""盲走-障礙比較評估。

輸入 recoder 的 JSONL trace（或 list[dict]），輸出 true 淨空（到箱）、
估計誤差、圈覆蓋等指標。純函式、無 I/O 依賴（路徑載入除外）。
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from vgr_core.geometry.arena_geometry import Box2D, box_distance_to_point

ROBOT_RADIUS = 0.23

TRUE_TOPIC = "/sim/true_pose"
STATUS_TOPIC = "/safety_gate/status"


def _load_rows(rows: list[dict[str, Any]] | str | Path) -> list[dict[str, Any]]:
    """接受 list[dict] 或 JSONL 路徑，回傳 row list。"""
    if isinstance(rows, (str, Path)):
        path = Path(rows)
        return [
            json.loads(line)
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    return list(rows)


def _nearest_status(status_rows: list[dict[str, Any]], t: float) -> dict[str, Any] | None:
    """優先找 round(t,2) 完全匹配；否則取時間最近的一筆。"""
    exact = next((r for r in status_rows if round(float(r["t"]), 2) == round(t, 2)), None)
    if exact is not None:
        return exact
    if not status_rows:
        return None
    return min(status_rows, key=lambda r: abs(float(r["t"]) - t))


def evaluate_blind_obstacle(
    rows: list[dict[str, Any]] | str | Path,
    *,
    box: dict[str, float],
    goal: tuple[float, float],
    d_safe: float,
) -> dict[str, Any]:
    """評估盲走障礙淨空與估計誤差。

    rows: list[dict] 或 JSONL 路徑。
      /sim/true_pose row: {"t", "true_pose": {"x", "y"}}
      /safety_gate/status row: {"t", "mode", "debug": {"pose_drift_m", "estimated_x", "estimated_y"}}
    box={"x","y","size_x","size_y"}；goal=(gx,gy)。

    回傳：
      min_true_clearance_m: min(箱距離 − ROBOT_RADIUS)
      collided: min_clearance < 0
      max_est_error_m: max(hypot(true − estimated))
      radius_series: [d_safe + drift] 每筆
      penetration_fixed_radius_m: max(0, max_err − d_safe)
      error_covered_by_inflation: all(err ≤ radius + 1e-9)
      reached_goal: 有 true 點距 goal ≤ 0.15
      arrive_t_s: 該點 t 或 None
    """
    rows_list = _load_rows(rows)

    true_rows = [r for r in rows_list if r.get("topic") == TRUE_TOPIC]
    status_rows = [r for r in rows_list if r.get("topic") == STATUS_TOPIC]
    true_rows.sort(key=lambda r: float(r["t"]))
    status_rows.sort(key=lambda r: float(r["t"]))

    box2d = Box2D(
        x=float(box["x"]), y=float(box["y"]),
        size_x=float(box["size_x"]), size_y=float(box["size_y"]),
    )

    clearances: list[float] = []
    radius_series: list[float] = []
    errors: list[float] = []
    arrive_t: float | None = None

    gx, gy = goal

    for tr in true_rows:
        t = float(tr["t"])
        pose = tr["true_pose"]
        tx, ty = float(pose["x"]), float(pose["y"])

        clearances.append(box_distance_to_point(box2d, tx, ty) - ROBOT_RADIUS)

        status = _nearest_status(status_rows, t)
        debug = (status or {}).get("debug") or {}
        if "estimated_x" in debug and "estimated_y" in debug:
            drift = float(debug.get("pose_drift_m", 0.0))
            ex, ey = float(debug["estimated_x"]), float(debug["estimated_y"])
            if all(math.isfinite(value) for value in (drift, ex, ey)):
                radius_series.append(d_safe + max(0.0, drift))
                errors.append(math.hypot(tx - ex, ty - ey))

        if arrive_t is None and math.hypot(tx - gx, ty - gy) <= 0.15:
            arrive_t = t

    min_clearance = min(clearances) if clearances else float("inf")
    max_err = max(errors) if errors else 0.0

    return {
        "min_true_clearance_m": min_clearance,
        "collided": min_clearance < 0,
        "max_est_error_m": max_err,
        "radius_series": radius_series,
        "penetration_fixed_radius_m": max(0.0, max_err - d_safe),
        "error_covered_by_inflation": all(
            e <= r + 1e-9 for e, r in zip(errors, radius_series)
        ),
        "reached_goal": arrive_t is not None,
        "arrive_t_s": arrive_t,
    }
