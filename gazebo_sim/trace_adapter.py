"""Gazebo JSONL trace 轉換器。

以 `/cmd_vel_safe` event 作為 `TraceSample` 主軸，其他 topic 取時間最近的
event；`clearance` 不信任 recorder，改用傳入的 `world.min_clearance()` 重算。
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from safety_sim.runner import Trace, TraceSample
from vgr_core.safety import Pose, Twist
from safety_sim.world import World


def _pose(data: dict[str, Any]) -> Pose:
    return Pose(float(data["x"]), float(data["y"]), float(data["theta"]))


def _twist(data: dict[str, Any]) -> Twist:
    return Twist(float(data["v"]), float(data["omega"]))


def _nearest(rows: list[dict[str, Any]], t: float) -> dict[str, Any] | None:
    if not rows:
        return None
    return min(rows, key=lambda row: abs(float(row["t"]) - t))


def load_trace(jsonl_path: str | Path, world: World) -> Trace:
    """讀取 recorder JSONL，回傳 safety_sim.runner.Trace。"""
    path = Path(jsonl_path)
    rows = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    by_topic: dict[str, list[dict[str, Any]]] = {}
    metadata: dict[str, Any] = {}
    for row in rows:
        topic = row.get("topic")
        if topic == "metadata":
            metadata.update(row)
            continue
        by_topic.setdefault(str(topic), []).append(row)
    for topic_rows in by_topic.values():
        topic_rows.sort(key=lambda row: float(row["t"]))

    samples: list[TraceSample] = []
    for safe in by_topic.get("/cmd_vel_safe", ()):
        t = float(safe["t"])
        true_row = _nearest(by_topic.get("/sim/true_pose", []), t)
        if true_row is None:
            continue
        true_pose = _pose(true_row["true_pose"])
        aruco_row = _nearest(by_topic.get("/aruco/pose", []), t)
        if aruco_row is None:
            est_pose = None
            pose_age_s = math.inf
        else:
            est_pose = _pose(aruco_row["pose"])
            pose_age_s = max(0.0, t - float(aruco_row["stamp_s"]))
        nav_row = _nearest(by_topic.get("/cmd_vel_nav", []), t)
        status_row = _nearest(by_topic.get("/safety_gate/status", []), t)
        desired = _twist(nav_row["twist"]) if nav_row is not None else Twist.stop()
        cmd = _twist(safe["twist"])
        actual_twist = _twist(true_row["actual_twist"]) if "actual_twist" in true_row else cmd
        mode = str(status_row.get("mode", "UNKNOWN")) if status_row is not None else "UNKNOWN"
        debug = dict(status_row.get("debug", {})) if status_row is not None else {}
        samples.append(TraceSample(
            t=t,
            true_pose=true_pose,
            est_pose=est_pose,
            pose_age_s=pose_age_s,
            link_age_s=0.0,
            desired=desired,
            cmd=cmd,
            mode=mode,
            actual_twist=actual_twist,
            clearance=world.min_clearance(true_pose),
            debug=debug,
        ))

    return Trace(
        scenario_name=str(metadata.get("scenario_name", path.stem)),
        filter_name=str(metadata.get("filter_name", "gazebo")),
        world=world,
        samples=samples,
    )
