"""單次執行的視覺化：俯視軌跡 + 命令/實際速度時間線 + 位姿新鮮度與淨空。"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .runner import Trace  # noqa: E402
from .scenario import Scenario  # noqa: E402

_MODE_COLORS = {"PASS": "#2a9d2a", "MODIFIED": "#e0a000", "STOP": "#d62728"}


def write_compare_markdown(results: dict, path: str | Path) -> None:
    """filters × scenarios 比較表：先 verdict 總表，再逐情境指標明細。"""
    from .compare import CellResult  # noqa: F401（型別說明用）
    from .scenarios import get_scenario

    scenario_names = sorted({k[0] for k in results})
    filter_names = sorted({k[1] for k in results})

    lines: list[str] = ["# 安全層比較表", ""]

    lines.append("| scenario | " + " | ".join(filter_names) + " |")
    lines.append("|---" * (len(filter_names) + 1) + "|")
    for s in scenario_names:
        cells = []
        for f in filter_names:
            r = results[(s, f)]
            cells.append("PASS" if r.passed else f"FAIL ({'; '.join(r.reasons)})")
        lines.append(f"| {s} | " + " | ".join(cells) + " |")
    lines.append("")

    metric_rows = (
        ("collided", lambda rep: str(rep.collided)),
        ("min_clearance [m]", lambda rep: f"{rep.min_clearance:.3f}"),
        ("max_speed [m/s]", lambda rep: f"{rep.max_speed_mps:.3f}"),
        ("time_to_stop_after_fault [s]", lambda rep: f"{rep.time_to_stop_after_fault_s:.2f}"),
        ("intervention_ratio", lambda rep: f"{rep.intervention_ratio:.3f}"),
        ("cmd_distortion", lambda rep: f"{rep.cmd_distortion:.4f}"),
    )
    for s in scenario_names:
        lines.append(f"## {s}：{get_scenario(s).description}")
        lines.append("")
        lines.append("| metric | " + " | ".join(filter_names) + " |")
        lines.append("|---" * (len(filter_names) + 1) + "|")
        for label, fmt in metric_rows:
            row = [fmt(results[(s, f)].report) for f in filter_names]
            lines.append(f"| {label} | " + " | ".join(row) + " |")
        lines.append("")

    Path(path).write_text("\n".join(lines), encoding="utf-8")


def plot_trace(trace: Trace, scenario: Scenario, path: str | Path) -> None:
    fig = plt.figure(figsize=(12, 8))
    grid = fig.add_gridspec(3, 2, width_ratios=[1.2, 1.0])

    # --- 俯視軌跡 ---
    ax = fig.add_subplot(grid[:, 0])
    fence = trace.world.geofence
    if fence:
        xs = [p[0] for p in fence] + [fence[0][0]]
        ys = [p[1] for p in fence] + [fence[0][1]]
        ax.plot(xs, ys, "k-", linewidth=2, label="geofence")
    from vgr_core.geometry.arena_geometry import Box2D

    for ob in trace.world.obstacles:
        if isinstance(ob, Box2D):
            ax.add_patch(plt.Rectangle(
                (ob.x - ob.size_x / 2.0, ob.y - ob.size_y / 2.0),
                ob.size_x, ob.size_y, color="gray", alpha=0.6))
        else:
            ax.add_patch(plt.Circle((ob.x, ob.y), ob.radius,
                                    color="gray", alpha=0.6))
    for mode, color in _MODE_COLORS.items():
        pts = [(s.true_pose.x, s.true_pose.y) for s in trace.samples if s.mode == mode]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], ".",
                    color=color, markersize=3, label=f"mode={mode}")
    start = trace.samples[0].true_pose
    ax.plot(start.x, start.y, "b^", markersize=10, label="start")
    hits = [(s.true_pose.x, s.true_pose.y) for s in trace.samples if s.clearance < 0.0]
    if hits:
        ax.plot(hits[0][0], hits[0][1], "rx", markersize=14, markeredgewidth=3,
                label="collision")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{trace.scenario_name} / {trace.filter_name}")
    ax.legend(loc="upper left", fontsize=8)

    ts = [s.t for s in trace.samples]

    # --- 速度時間線 ---
    ax = fig.add_subplot(grid[0, 1])
    ax.plot(ts, [s.desired.v for s in trace.samples], "--", color="gray", label="nav desired v")
    ax.plot(ts, [s.cmd.v for s in trace.samples], color="#e0a000", label="filtered v")
    ax.plot(ts, [s.actual_twist.v for s in trace.samples], color="#1f77b4", label="actual v")
    ax.axhline(scenario.max_v_mps, color="r", linestyle=":", linewidth=1, label="safe limit")
    ax.set_ylabel("v [m/s]")
    ax.legend(fontsize=7)

    # --- 位姿新鮮度 ---
    ax = fig.add_subplot(grid[1, 1])
    ax.plot(ts, [min(s.pose_age_s, 10.0) for s in trace.samples], color="#9467bd")
    ax.set_ylabel("pose age [s]")

    # --- 淨空 ---
    ax = fig.add_subplot(grid[2, 1])
    ax.plot(ts, [s.clearance for s in trace.samples], color="#2ca02c")
    ax.axhline(0.0, color="r", linestyle=":", linewidth=1)
    ax.set_ylabel("clearance [m]")
    ax.set_xlabel("t [s]")

    if scenario.fault_t0 is not None:
        for panel in fig.axes[1:]:
            panel.axvline(scenario.fault_t0, color="r", alpha=0.4, linewidth=1)

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
