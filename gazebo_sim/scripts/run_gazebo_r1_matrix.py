#!/usr/bin/env python3
"""Run the reproducible 90-run Gazebo R1 drift matrix.

The synthetic fault lowers left-wheel friction. Each run teleports to the
start, drives straight, and records simulated true pose versus dead-reckoned
odometry. Ground truth is used only by the evaluator, never by a safety filter.

Usage:
  python3 gazebo_sim/scripts/run_gazebo_r1_matrix.py --out /tmp/r1_gazebo --mu 0.05 --seed 1
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
WORLD_SRC = REPO_ROOT / "gazebo_sim/worlds/vgr_arena.world"
MODEL_SRC = REPO_ROOT / "gazebo_sim/models/vgr_diff_drive"

START = (0.5, 0.0)
SETTLE_S = 0.3
STOP_HOLD_S = 0.5
POST_STOP_S = 0.4


def _read_pose(topic: str, timeout_s: int = 10) -> tuple[float, float, float]:
    """讀 ignition.msgs.Odometry 的 position + yaw（遞迴找 position）。"""
    out = subprocess.run(
        ["ign", "topic", "-e", "--json-output", "-t", topic, "-n", "1"],
        capture_output=True, text=True, timeout=timeout_s + 5,
    )
    if out.returncode != 0:
        raise RuntimeError(f"read {topic} failed: {out.stderr[:200]}")
    for line in out.stdout.splitlines():
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            continue

        def find_pose(v):
            if isinstance(v, dict):
                p = v.get("position")
                if isinstance(p, dict) and "x" in p:
                    q = v.get("orientation") or {}
                    yaw = 0.0
                    try:
                        qw, qx, qy, qz = (float(q[k]) for k in ("w", "x", "y", "z"))
                        yaw = math.atan2(2 * (qw * qz + qx * qy),
                                         1 - 2 * (qy * qy + qz * qz))
                    except Exception:
                        pass
                    return float(p["x"]), float(p["y"]), yaw
                for child in v.values():
                    r = find_pose(child)
                    if r is not None:
                        return r
            elif isinstance(v, list):
                for child in v:
                    r = find_pose(child)
                    if r is not None:
                        return r
            return None

        found = find_pose(data)
        if found is not None:
            return found
    raise RuntimeError(f"pose not found in {topic} output")


def _set_pose(world: str, model: str, x: float, y: float, z: float) -> None:
    req = (f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}')
    subprocess.run(
        ["ign", "service", "-s", f"/world/{world}/set_pose", "--timeout", "5000",
         "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
         "--req", req],
        capture_output=True, text=True, timeout=15,
    )


def _publish_twist(v: float, w: float = 0.0) -> None:
    subprocess.run(
        ["ign", "topic", "-t", "/cmd_vel_safe", "-m", "ignition.msgs.Twist",
         "-p", f"linear: {{x: {v}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {w}}}"],
        capture_output=True, text=True, timeout=15,
    )


def _wait_topic(topic: str, server_pid: int, timeout_s: int = 40) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if server_pid.poll() is not None:
            raise RuntimeError("gazebo server exited early")
        probe = subprocess.run(["ign", "topic", "-l"], capture_output=True,
                               text=True, timeout=10)
        if topic in probe.stdout.splitlines():
            return
        time.sleep(0.5)
    raise RuntimeError(f"topic {topic} timeout")


def build_world(mu: float, out_dir: Path) -> Path:
    """產生含低摩擦左輪的 world（vgr_arena + 車 @ START）。"""
    import sys

    sys.path.insert(0, str(REPO_ROOT))
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws/src/vgr_core"))
    from gazebo_sim.generators.generate_robot_sdf import build_robot_sdf
    from vgr_core.motion import DiffDriveParams

    models = out_dir / "models/vgr_diff_drive"
    models.mkdir(parents=True, exist_ok=True)
    (models / "model.config").write_text((MODEL_SRC / "model.config").read_text(
        encoding="utf-8"), encoding="utf-8")
    (models / "model.sdf").write_text(
        build_robot_sdf(DiffDriveParams(), left_wheel_mu=mu), encoding="utf-8")

    tree = ET.parse(WORLD_SRC)
    world = tree.getroot().find("world")
    inc = ET.SubElement(world, "include")
    ET.SubElement(inc, "uri").text = "model://vgr_diff_drive"
    ET.SubElement(inc, "pose").text = f"{START[0]} {START[1]} 0 0 0 0"
    ET.indent(tree.getroot(), space="  ")
    world_path = out_dir / "r1.world"
    tree.write(world_path, encoding="unicode", xml_declaration=False)
    return world_path


def run_matrix(*, out_dir: Path, mu: float, seed: int, speedup: bool = False) -> Path:
    from safety_sim.experiments.physical_contract import build_r1_schedule
    from safety_sim.experiments.r1_drift import analyze_r1_directory

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    world_path = build_world(mu, out_dir)

    env = dict(os.environ)
    env["HOME"] = str(out_dir / "home")
    env["IGN_GAZEBO_RESOURCE_PATH"] = (
        f"{out_dir / 'models'}:{REPO_ROOT / 'gazebo_sim/models'}")
    (out_dir / "home").mkdir(exist_ok=True)

    server = subprocess.Popen(
        ["ign", "gazebo", "-s", "-r", str(world_path)],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    try:
        _wait_topic("/sim/true_pose", server)
        _wait_topic("/odom", server)
        runs, dropped = build_r1_schedule(seed)
        if speedup:
            zero = [r for r in runs if r.target_blind_distance_m == 0.0][:1]
            moving = [r for r in runs if r.target_blind_distance_m > 0.0][:2]
            runs = zero + moving
        rows = []
        for run in runs:
            _set_pose("vgr_arena", "vgr_diff_drive", START[0], START[1], 0.065)
            time.sleep(SETTLE_S)
            t0 = _read_pose("/sim/true_pose")
            o0 = _read_pose("/odom")
            if run.commanded_speed_mps > 0.0:
                _publish_twist(run.commanded_speed_mps)
                travel_s = run.target_blind_distance_m / run.commanded_speed_mps
                if speedup:
                    travel_s = min(travel_s, 1.0)
                time.sleep(travel_s + STOP_HOLD_S)
                _publish_twist(0.0)
                time.sleep(POST_STOP_S)
            else:
                time.sleep(1.0)  # zero cell：原地量測 anchor 誤差
            t1 = _read_pose("/sim/true_pose")
            o1 = _read_pose("/odom")
            _publish_twist(0.0)
            # /odom 是 robot-relative 積分（原點 0）；盲走信念 =
            # 最後 anchor（START）+ odom 位移。fused 用此映射，
            # 不能直接用 odom 絕對座標（會帶座標原點偏移）。
            odom_dx = o1[0] - o0[0]
            odom_dy = o1[1] - o0[1]
            rows.append({
                "run_id": run.run_id,
                "speed_mps": run.commanded_speed_mps,
                "blind_m": odom_dx,
                "physical_x": t1[0], "physical_y": t1[1],
                "fused_x": START[0] + odom_dx,
                "fused_y": START[1] + odom_dy,
                "intended_x": START[0] + run.target_blind_distance_m,
                "intended_y": START[1],
                "baseline_length_m": run.target_blind_distance_m,
                "baseline_residual_m": 0.002,
                "payload_kg": 1.0,
                "floor_material": "gazebo_floor_mu1.0",
            })
            if speedup:
                print(f"{run.run_id}: done", flush=True)
        csv_path = out_dir / "measurements.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        meta = {"mu_left": mu, "seed": seed, "n_runs": len(rows),
                "dropped_cells": dropped,
                "note": "Gazebo R1: left-wheel friction injection; "
                        "physical=true_pose, fused=/odom"}
        (out_dir / "meta.json").write_text(json.dumps(meta, indent=2),
                                           encoding="utf-8")
        analyze_r1_directory(out_dir, instrument_resolution_m=0.002)
        return csv_path
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="run_gazebo_r1_matrix")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--mu", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--speedup", action="store_true",
                        help="縮短 sleep（除錯用；不用於正式數據）")
    args = parser.parse_args(argv)
    csv_path = run_matrix(out_dir=args.out, mu=args.mu, seed=args.seed,
                          speedup=args.speedup)
    print(f"matrix done: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
