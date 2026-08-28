#!/usr/bin/env python3
"""畫實驗 trace：軌跡 + 場地 + 障礙 + 起點/goal。

用法：
  python3 gazebo_sim/scripts/plot_experiment_trace.py \
      outputs/safety_experiments/GS3_box/sapf_new.jsonl --out /tmp/sapf_box.png
  python3 gazebo_sim/scripts/plot_experiment_trace.py \
      outputs/safety_experiments/R3_gazebo_matrix/sapf_new_run_0.jsonl \
      --scenario R3 --out /tmp/r3.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[2]
GEOMETRY = json.loads(
    (REPO / "outputs/safety_experiments/geometry.json").read_text(encoding="utf-8"))


def load_trace(path: Path):
    pts, cmds, modes = [], [], []
    for line in Path(path).open(encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        row = json.loads(line)
        topic = row.get("topic")
        if topic == "/sim/true_pose":
            p = row["true_pose"]
            pts.append((row["t"], p["x"], p["y"], p.get("theta", 0.0)))
        elif topic == "/cmd_vel_safe":
            cmds.append((row["t"], row["twist"]["v"], row["twist"]["omega"]))
        elif topic == "/safety_gate/status":
            modes.append((row["t"], row.get("mode")))
    return pts, cmds, modes


def draw_geometry(ax, scenario: str):
    arena = GEOMETRY["arena"]["geofence"]
    xs = [p[0] for p in arena] + [arena[0][0]]
    ys = [p[1] for p in arena] + [arena[0][1]]
    ax.plot(xs, ys, "k-", lw=1.5, label="geofence")

    if scenario == "R3":
        (x0, y0), (x1, y1) = GEOMETRY["scenarios"]["R3"]["virtual_line"]
        ax.plot([x0, x1], [y0, y1], "r--", lw=2, label="virtual line (x=2.0)")
    else:
        ob = GEOMETRY["obstacles"]["SAPF_OBSTACLE"]
        ax.add_patch(plt.Rectangle(
            (ob["x"] - ob["size_x"] / 2, ob["y"] - ob["size_y"] / 2),
            ob["size_x"], ob["size_y"],
            color="tab:green", alpha=0.6, label="obstacle box 0.4x0.4"))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="plot_experiment_trace")
    parser.add_argument("trace", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scenario", choices=("S8_GS3", "R3"), default="S8_GS3")
    args = parser.parse_args(argv)

    pts, cmds, modes = load_trace(args.trace)
    if not pts:
        raise SystemExit("no /sim/true_pose samples in trace")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.5))
    draw_geometry(ax1, args.scenario)
    sc = GEOMETRY["scenarios"][args.scenario]
    ax1.plot(sc["start"][0], sc["start"][1], "s", color="black", label="start")
    ax1.plot(sc["goal"][0], sc["goal"][1], "*", color="tab:red", ms=15,
             label="goal")
    ts = [p[0] for p in pts]
    xs = [p[1] for p in pts]
    ys = [p[2] for p in pts]
    ax1.plot(xs, ys, "-", color="tab:blue", lw=1.5, label="robot path")
    ax1.plot(xs[-1], ys[-1], "o", color="tab:red", ms=6)
    ax1.set_aspect("equal")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title(f"{args.trace.name} — final dist to goal "
                  f"{math.hypot(xs[-1] - sc['goal'][0], ys[-1] - sc['goal'][1]):.3f} m")
    ax1.legend(fontsize=8, loc="best")
    ax1.grid(alpha=0.3)

    ax2.plot(ts, xs, label="x", lw=1.2)
    ax2.plot(ts, ys, label="y", lw=1.2)
    if cmds:
        cts = [c[0] for c in cmds]
        ax2.plot(cts, [abs(c[1]) for c in cmds], label="|v| safe", lw=0.8,
                 color="tab:green", alpha=0.7)
    if modes:
        mts = [m[0] for m in modes]
        ax2.fill_between(mts, -1, 1, where=[m[1] == "STOP" for m in modes],
                         color="red", alpha=0.15, label="STOP")
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("x/y [m]")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=140)
    print(f"saved {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
