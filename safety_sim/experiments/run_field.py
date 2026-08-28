"""大場地 × 定位誤差模型 × 濾波器對抗掃描（wrapper harness）。

重用核心零件（World / DiffDriveVehicle / CommandLink / nav / filters / metrics），
自帶一個 tick 迴圈（複製自 safety_sim.runner 的 ~30 行）以便：
  1. 換上 FieldLocalizer（多回傳 pose_drift_m）。
  2. 把 pose_drift_m 填進 Observation（核心 runner 永遠填 0）。
其餘完全走核心，不改任何 filter 或模擬器核心檔。

用法：
  python3 -m safety_sim.experiments.run_field \\
      --out outputs/sim_field_comparison --seeds 50
  # 或 --smoke 只跑 3 seeds 快速自檢
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

from vgr_core.motion import DiffDriveParams

from .. import metrics
from ..filters import available_filters, make_filter
from ..link import CommandLink
from ..runner import Trace, TraceSample
from ..types import Observation, SafetyDecision, StaticInfo
from ..vehicle import DiffDriveVehicle
from .field_localizer import FieldLocalizer
from .field_scenarios import EpisodeConfig, all_episodes, make_arena

FILTERS = [
    "passthrough",  # baseline（無濾波）
    "safe_apf",
    "cbf",
    "iccbf",
    "geofence_vo",
    "nh_vo",
    "gf_dwa",
    "backup_mps",
]


@dataclass
class EpisodeResult:
    collided: bool
    min_clearance: float
    reached_goal: bool
    success: bool  # 抵達且全程未碰撞
    time_to_goal_s: float | None
    intervention_ratio: float
    max_speed_mps: float


def run_episode(cfg: EpisodeConfig, filter_name: str, seed: int) -> EpisodeResult:
    world = make_arena(goal=cfg.goal)
    vehicle = DiffDriveVehicle(DiffDriveParams(), pose=cfg.start_pose, **cfg.vehicle_kwargs)
    localizer = FieldLocalizer(
        update_hz=cfg.update_hz,
        noise_xy_std=cfg.noise_xy_std,
        systematic_bias_m=cfg.systematic_bias_m,
        drift_rate_per_m=cfg.drift_rate_per_m,
        seed=seed,
    )
    link = CommandLink(timeout_s=0.5)
    nav = cfg.make_nav()
    faults = cfg.make_faults(seed)
    filt = make_filter(filter_name)

    filt.reset(StaticInfo(
        params=vehicle.params,
        robot_radius_m=world.robot_radius_m,
        geofence=world.geofence,
        max_v_mps=cfg.max_v_mps,
        max_omega_rad_s=cfg.max_omega_rad_s,
    ))

    control_hz, plant_hz = 20.0, 100.0
    control_dt = 1.0 / control_hz
    plant_dt = 1.0 / plant_hz
    substeps = max(1, round(control_dt / plant_dt))
    ticks = round(cfg.duration_s * control_hz)

    samples: list[TraceSample] = []
    time_to_goal: float | None = None
    for i in range(ticks):
        t = i * control_dt
        est_pose, pose_age, pose_drift = localizer.observe(
            vehicle.pose, t, dropout=faults.active("aruco_dropout", t))
        obs = Observation(
            pose=est_pose,
            pose_age_s=pose_age,
            wheel_feedback=vehicle.wheel_counts_per_s,
            obstacles=world.obstacles,
            link_age_s=link.age_s(t),
            pose_drift_m=(0.0 if math.isinf(pose_drift) else pose_drift),
        )
        desired = nav.command(obs, t)
        decision: SafetyDecision = filt.filter(desired, obs, t, control_dt)
        link.send(decision.cmd, t, dropped=faults.active("link_drop", t))
        link.poll(vehicle, t)
        for _ in range(substeps):
            vehicle.step(plant_dt)

        clearance = world.min_clearance(vehicle.pose)
        samples.append(TraceSample(
            t=t, true_pose=vehicle.pose, est_pose=est_pose, pose_age_s=pose_age,
            link_age_s=obs.link_age_s, desired=desired, cmd=decision.cmd,
            mode=decision.mode, actual_twist=vehicle.twist_actual,
            clearance=clearance, debug=decision.debug,
        ))
        dist_goal = math.hypot(vehicle.pose.x - cfg.goal[0], vehicle.pose.y - cfg.goal[1])
        if time_to_goal is None and dist_goal <= cfg.success_radius_m:
            time_to_goal = t

    trace = Trace(scenario_name=cfg.name, filter_name=filter_name, world=world, samples=samples)
    report = metrics.summarize(trace, fault_t0=cfg.fault_t0(seed))
    reached = time_to_goal is not None
    return EpisodeResult(
        collided=report.collided,
        min_clearance=report.min_clearance,
        reached_goal=reached,
        success=reached and not report.collided,
        time_to_goal_s=time_to_goal,
        intervention_ratio=report.intervention_ratio,
        max_speed_mps=report.max_speed_mps,
    )


def _pct(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    k = (len(s) - 1) * (p / 100.0)
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return s[int(k)]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def aggregate(results: list[EpisodeResult]) -> dict:
    n = len(results)
    clearances = [r.min_clearance for r in results]
    reach_times = [r.time_to_goal_s for r in results if r.time_to_goal_s is not None]
    return {
        "n": n,
        "collision_rate": sum(r.collided for r in results) / n,
        "min_clearance_median_m": statistics.median(clearances),
        "min_clearance_p5_m": _pct(clearances, 5.0),
        "min_clearance_worst_m": min(clearances),
        "goal_reach_rate": sum(r.reached_goal for r in results) / n,
        "success_rate": sum(r.success for r in results) / n,
        "mean_time_to_goal_s": (statistics.mean(reach_times) if reach_times else None),
        "mean_intervention_ratio": statistics.mean(r.intervention_ratio for r in results),
    }


def run_all(seeds: int, filters: list[str], episodes: dict[str, EpisodeConfig]) -> dict:
    out: dict = {}
    for scen_name, cfg in episodes.items():
        out[scen_name] = {"description": cfg.description, "filters": {}}
        for fname in filters:
            results = [run_episode(cfg, fname, seed) for seed in range(seeds)]
            out[scen_name]["filters"][fname] = aggregate(results)
    return out


def _fmt(v, spec="{:.3f}") -> str:
    if v is None:
        return "n/a"
    if isinstance(v, float) and math.isnan(v):
        return "n/a"
    return spec.format(v) if isinstance(v, float) else str(v)


def write_summary(data: dict, meta: dict, path: Path) -> None:
    lines: list[str] = []
    lines.append("# 大場地 × 定位誤差 × 安全濾波器對抗掃描")
    lines.append("")
    lines.append(f"- 產生時間指令：`{meta['cmd']}`")
    lines.append(f"- 每 (情境 × 濾波器) 隨機種子數：**{meta['seeds']}**")
    lines.append(f"- 濾波器：{', '.join(meta['filters'])}（`passthrough` = 無濾波 baseline）")
    lines.append(f"- 場地：梯形牆角多邊形 {meta['arena']}（World 原生多邊形 geofence）")
    lines.append("- 定位誤差模型：位姿噪聲 σ=2cm、系統偏差 4cm（固定方向/seed）、"
                 "盲段 1–4.2s 隨機、盲段中估計位姿凍結（原生語意）+ pose_drift 上界 24%/m（wrapper 注入）")
    lines.append("")
    cols = [
        ("collision_rate", "碰撞率", "{:.2%}"),
        ("min_clearance_median_m", "淨空中位(m)", "{:.3f}"),
        ("min_clearance_p5_m", "淨空P5(m)", "{:.3f}"),
        ("min_clearance_worst_m", "淨空最差(m)", "{:.3f}"),
        ("goal_reach_rate", "到點率", "{:.2%}"),
        ("success_rate", "成功率(到點且未撞)", "{:.2%}"),
        ("mean_time_to_goal_s", "平均用時(s)", "{:.2f}"),
        ("mean_intervention_ratio", "干預比", "{:.3f}"),
    ]
    for scen_name, scen in data.items():
        lines.append(f"## 情境：{scen_name}")
        lines.append("")
        lines.append(f"_{scen['description']}_")
        lines.append("")
        header = "| 濾波器 | " + " | ".join(c[1] for c in cols) + " |"
        sep = "| --- | " + " | ".join("---" for _ in cols) + " |"
        lines.append(header)
        lines.append(sep)
        for fname, agg in scen["filters"].items():
            cells = []
            for key, _label, spec in cols:
                v = agg.get(key)
                if isinstance(v, float) and not math.isnan(v):
                    cells.append(spec.format(v))
                elif v is None or (isinstance(v, float) and math.isnan(v)):
                    cells.append("n/a")
                else:
                    cells.append(str(v))
            label = fname + (" (baseline)" if fname == "passthrough" else "")
            lines.append(f"| {label} | " + " | ".join(cells) + " |")
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="safety_sim.experiments.run_field")
    parser.add_argument("--out", type=Path, default=Path("outputs/sim_field_comparison"))
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument("--smoke", action="store_true", help="只跑 3 seeds 快速自檢")
    parser.add_argument("--filters", default=None, help="逗號分隔，預設全部 8 個")
    args = parser.parse_args(argv)

    seeds = 3 if args.smoke else args.seeds
    filters = args.filters.split(",") if args.filters else FILTERS
    # 驗證 filter 名。
    valid = set(available_filters())
    for f in filters:
        if f not in valid:
            raise SystemExit(f"unknown filter {f!r}; available: {sorted(valid)}")

    episodes = all_episodes()
    from .field_scenarios import ARENA
    data = run_all(seeds, filters, episodes)

    args.out.mkdir(parents=True, exist_ok=True)
    cmd = f"python3 -m safety_sim.experiments.run_field --out {args.out} --seeds {seeds}"
    meta = {"cmd": cmd, "seeds": seeds, "filters": filters, "arena": str(ARENA)}
    payload = {"meta": meta, "scenarios": data}
    (args.out / "results.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    write_summary(data, meta, args.out / "summary.md")
    print(f"wrote {args.out/'results.json'} and {args.out/'summary.md'} "
          f"({len(filters)} filters × {len(episodes)} scenarios × {seeds} seeds)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
