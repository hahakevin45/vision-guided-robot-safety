"""E1/E2 安全濾波器實驗：盲段衝牆 × 倒車盲區。

E1：盲段衝牆交叉實驗（信念-真實分岔語意）
  盲段中信念位姿繼續按 odom 積分前進（偏差=盲走里程×24%、方向 per-seed
  隨機）；真實位姿=實際運動。濾波器吃信念距離，碰撞用真實位姿判。
  校準 CBF 參數使無盲段停距與 safe_apf 差 <2cm →
  跑 2×2×50 seeds = {safe_apf, cbf_calibrated}×{無盲段, 盲段}。
  預期：無盲段兩者同；有盲段 cbf 真實停距散開、safe_apf 提早停且分佈緊。

E2：倒車盲區重現
  {safe_apf, cbf 預設, cbf_calibrated}×seeds，碰撞率、停距。

用法：
  python3 -m safety_sim.experiments.run_e1e2 --out outputs/sim_e1e2
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
from ..filters import make_filter
from ..link import CommandLink
from ..runner import Trace, TraceSample
from ..types import Observation, SafetyDecision, StaticInfo
from ..vehicle import DiffDriveVehicle
from .e1e2_scenarios import (BLIND_APPROACH, REVERSE_INTO_WALL,
                              E1E2EpisodeConfig, make_rect_arena)
from .field_localizer import FieldLocalizer


@dataclass
class E1E2EpisodeResult:
    collided: bool
    final_clearance: float
    true_stop_dist_m: float
    belief_stop_dist_m: float
    belief_vs_true_diff_m: float
    min_clearance: float
    max_speed_mps: float


def _distance_to_wall(pose_x: float, target_wall_x: float) -> float:
    return abs(pose_x - target_wall_x)


def run_episode(cfg: E1E2EpisodeConfig, filter_name: str, seed: int, *,
                blind_enabled: bool = True,
                filter_kwargs: dict | None = None) -> E1E2EpisodeResult:
    world = make_rect_arena()
    vehicle = DiffDriveVehicle(DiffDriveParams(), pose=cfg.start_pose,
                               **cfg.vehicle_kwargs)
    localizer = FieldLocalizer(
        update_hz=cfg.update_hz,
        noise_xy_std=cfg.noise_xy_std,
        systematic_bias_m=cfg.systematic_bias_m,
        drift_rate_per_m=cfg.drift_rate_per_m,
        seed=seed,
        blind_max_s=cfg.blind_max_s,
        blind_max_dist_m=cfg.blind_max_dist_m,
    )
    link = CommandLink(timeout_s=0.5)
    nav = cfg.make_nav()
    filt = make_filter(filter_name, **(filter_kwargs or {}))

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

    blind_active = False
    tw = cfg.target_wall_x

    samples: list[TraceSample] = []
    for i in range(ticks):
        t = i * control_dt
        true_pose = vehicle.pose

        if blind_enabled and cfg.blind_at_distance_m is not None and not blind_active:
            if _distance_to_wall(true_pose.x, tw) <= cfg.blind_at_distance_m:
                blind_active = True

        est_pose, pose_age, pose_drift = localizer.observe(
            true_pose, t, dropout=blind_active)
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
        link.send(decision.cmd, t, dropped=False)
        link.poll(vehicle, t)
        for _ in range(substeps):
            vehicle.step(plant_dt)

        if blind_enabled and cfg.blind_at_distance_m is not None and not blind_active:
            if _distance_to_wall(vehicle.pose.x, tw) <= cfg.blind_at_distance_m:
                blind_active = True

        clearance = world.min_clearance(vehicle.pose)
        samples.append(TraceSample(
            t=t, true_pose=vehicle.pose, est_pose=est_pose, pose_age_s=pose_age,
            link_age_s=obs.link_age_s, desired=desired, cmd=decision.cmd,
            mode=decision.mode, actual_twist=vehicle.twist_actual,
            clearance=clearance, debug=decision.debug,
        ))

    final_true = vehicle.pose
    final_est = est_pose
    true_dist = _distance_to_wall(final_true.x, tw)
    belief_dist = _distance_to_wall(final_est.x, tw) if final_est else float("nan")

    trace = Trace(scenario_name=cfg.name, filter_name=filter_name,
                  world=world, samples=samples)
    report = metrics.summarize(trace)

    return E1E2EpisodeResult(
        collided=report.collided,
        final_clearance=world.min_clearance(final_true),
        true_stop_dist_m=true_dist,
        belief_stop_dist_m=belief_dist,
        belief_vs_true_diff_m=(belief_dist - true_dist) if final_est else float("nan"),
        min_clearance=report.min_clearance,
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


def aggregate(results: list[E1E2EpisodeResult]) -> dict:
    n = len(results)
    true_dists = [r.true_stop_dist_m for r in results]
    belief_diffs = [r.belief_vs_true_diff_m for r in results
                    if not math.isnan(r.belief_vs_true_diff_m)]
    return {
        "n": n,
        "collision_rate": sum(r.collided for r in results) / n,
        "true_stop_dist_median_m": statistics.median(true_dists),
        "true_stop_dist_p5_m": _pct(true_dists, 5.0),
        "true_stop_dist_p95_m": _pct(true_dists, 95.0),
        "true_stop_dist_min_m": min(true_dists),
        "true_stop_dist_max_m": max(true_dists),
        "belief_vs_true_diff_median_m": statistics.median(belief_diffs) if belief_diffs else None,
        "belief_vs_true_diff_mean_m": statistics.mean(belief_diffs) if belief_diffs else None,
        "min_clearance_median_m": statistics.median([r.min_clearance for r in results]),
    }


def calibrate_cbf(cfg: E1E2EpisodeConfig, seed: int = 0) -> dict:
    """掃 CBF 參數，找無盲段停距與 safe_apf 預設差 <2cm 的組合。"""
    safe_ref = run_episode(cfg, "safe_apf", seed, blind_enabled=False)
    ref_dist = safe_ref.true_stop_dist_m

    buffer_vals = [0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.12, 0.14, 0.16]
    alpha_vals = [0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0]

    best = None
    best_diff = float("inf")
    sweep: list[dict] = []

    for bm in buffer_vals:
        for al in alpha_vals:
            r = run_episode(cfg, "cbf", seed, blind_enabled=False,
                            filter_kwargs={"buffer_m": bm, "alpha": al})
            diff = abs(r.true_stop_dist_m - ref_dist)
            entry = {"buffer_m": bm, "alpha": al,
                     "true_stop_dist_m": r.true_stop_dist_m,
                     "diff_vs_safe_apf_m": diff}
            sweep.append(entry)
            if diff < best_diff:
                best_diff = diff
                best = entry

    return {
        "safe_apf_stop_dist_m": ref_dist,
        "safe_apf_final_clearance_m": safe_ref.final_clearance,
        "calibrated": best,
        "best_diff_m": best_diff,
        "sweep": sweep,
    }


def run_e1(cfg: E1E2EpisodeConfig, cbf_calibrated: dict, seeds: int) -> dict:
    """E1: 2×2×seeds = {safe_apf, cbf_calibrated}×{無盲段, 盲段}。"""
    filter_configs: list[dict] = [
        {"key": "safe_apf", "name": "safe_apf", "kwargs": None},
        {"key": "cbf_calibrated", "name": "cbf",
         "kwargs": {"buffer_m": cbf_calibrated["buffer_m"],
                    "alpha": cbf_calibrated["alpha"]}},
    ]
    blind_labels = {"no_blind": False, "blind": True}
    out: dict = {}
    for fcfg in filter_configs:
        fkey = fcfg["key"]
        out[fkey] = {}
        for blabel, blind_enabled in blind_labels.items():
            results = [run_episode(cfg, fcfg["name"], seed,
                                   blind_enabled=blind_enabled,
                                   filter_kwargs=fcfg["kwargs"])
                       for seed in range(seeds)]
            out[fkey][blabel] = aggregate(results)
    return out


def run_e2(cfg: E1E2EpisodeConfig, cbf_calibrated: dict, seeds: int) -> dict:
    """E2: {safe_apf, cbf, cbf_calibrated}×seeds。"""
    filter_configs: list[dict] = [
        {"key": "safe_apf", "name": "safe_apf", "kwargs": None},
        {"key": "cbf_default", "name": "cbf", "kwargs": None},
        {"key": "cbf_calibrated", "name": "cbf",
         "kwargs": {"buffer_m": cbf_calibrated["buffer_m"],
                    "alpha": cbf_calibrated["alpha"]}},
    ]
    out: dict = {}
    for fcfg in filter_configs:
        fkey = fcfg["key"]
        results = [run_episode(cfg, fcfg["name"], seed,
                               blind_enabled=False,
                               filter_kwargs=fcfg["kwargs"])
                   for seed in range(seeds)]
        out[fkey] = aggregate(results)
    return out


def write_e1_summary(data: dict, cfg: E1E2EpisodeConfig,
                     cbf_calibrated: dict, meta: dict, path: Path) -> None:
    lines: list[str] = []
    lines.append("# E1 盲段衝牆交叉實驗（信念-真實分岔語意）")
    lines.append("")
    lines.append(f"- 產生指令：`{meta['cmd']}`")
    lines.append(f"- 每 (濾波器 × 盲段) 種子數：**{meta['seeds']}**")
    lines.append(f"- 情境：{cfg.description}")
    lines.append(f"- 盲段語意：信念按 odom 積分前進（偏差=盲走里程×24%、方向 per-seed 隨機、")
    lines.append(f"  疊 σ2cm 噪聲）；真實=實際運動。濾波器吃信念距離，碰撞用真實位姿判。")
    lines.append(f"- 盲走預算（場地政策）：blind_max_s={cfg.blind_max_s}s, "
                 f"blind_max_dist={cfg.blind_max_dist_m}m")
    lines.append(f"- pose_drift_m（實車契約）：0.10 + 0.30×盲走里程")
    lines.append(f"- 校準 cbf_calibrated：buffer_m={cbf_calibrated['buffer_m']}, "
                 f"alpha={cbf_calibrated['alpha']}")
    lines.append(f"- safe_apf 預設無盲段停距：{meta['safe_apf_ref_m']:.4f} m")
    lines.append(f"- cbf_calibrated 無盲段停距差：{meta['cal_diff_m']:.4f} m")
    lines.append("")

    cols = [
        ("collision_rate", "碰撞率", "{:.2%}"),
        ("true_stop_dist_median_m", "真實停距中位(m)", "{:.4f}"),
        ("true_stop_dist_p5_m", "真實停距P5(m)", "{:.4f}"),
        ("true_stop_dist_p95_m", "真實停距P95(m)", "{:.4f}"),
        ("belief_vs_true_diff_median_m", "信念vs真實差中位(m)", "{:.4f}"),
        ("min_clearance_median_m", "淨空中位(m)", "{:.4f}"),
    ]
    header = "| 濾波器 | 盲段 | " + " | ".join(c[1] for c in cols) + " |"
    sep = "| --- | --- | " + " | ".join("---" for _ in cols) + " |"
    lines.append(header)
    lines.append(sep)
    for fkey in data:
        for blabel in ["no_blind", "blind"]:
            agg = data[fkey][blabel]
            cells = []
            for key, _label, spec in cols:
                v = agg.get(key)
                if v is None or (isinstance(v, float) and math.isnan(v)):
                    cells.append("n/a")
                else:
                    cells.append(spec.format(v))
            bdisp = "無" if blabel == "no_blind" else "有"
            lines.append(f"| {fkey} | {bdisp} | " + " | ".join(cells) + " |")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_e2_summary(data: dict, cfg: E1E2EpisodeConfig,
                     cbf_calibrated: dict, meta: dict, path: Path) -> None:
    lines: list[str] = []
    lines.append("# E2 倒車盲區重現")
    lines.append("")
    lines.append(f"- 產生指令：`{meta['cmd']}`")
    lines.append(f"- 每濾波器種子數：**{meta['seeds']}**")
    lines.append(f"- 情境：{cfg.description}")
    lines.append(f"- 車尾對牆 0.8m，常速倒車 v=-0.05")
    lines.append(f"- cbf_calibrated（來自 E1 校準）：buffer_m={cbf_calibrated['buffer_m']}, "
                 f"alpha={cbf_calibrated['alpha']}")
    lines.append("")

    cols = [
        ("collision_rate", "碰撞率", "{:.2%}"),
        ("true_stop_dist_median_m", "真實停距中位(m)", "{:.4f}"),
        ("true_stop_dist_p5_m", "真實停距P5(m)", "{:.4f}"),
        ("true_stop_dist_p95_m", "真實停距P95(m)", "{:.4f}"),
        ("min_clearance_median_m", "淨空中位(m)", "{:.4f}"),
    ]
    header = "| 濾波器 | " + " | ".join(c[1] for c in cols) + " |"
    sep = "| --- | " + " | ".join("---" for _ in cols) + " |"
    lines.append(header)
    lines.append(sep)
    for fkey, agg in data.items():
        cells = []
        for key, _label, spec in cols:
            v = agg.get(key)
            if v is None or (isinstance(v, float) and math.isnan(v)):
                cells.append("n/a")
            else:
                cells.append(spec.format(v))
        lines.append(f"| {fkey} | " + " | ".join(cells) + " |")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="safety_sim.experiments.run_e1e2")
    parser.add_argument("--out", type=Path, default=Path("outputs/sim_e1e2"))
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument("--smoke", action="store_true", help="只跑 3 seeds 快速自檢")
    args = parser.parse_args(argv)

    seeds = 3 if args.smoke else args.seeds
    args.out.mkdir(parents=True, exist_ok=True)
    cmd = f"python3 -m safety_sim.experiments.run_e1e2 --out {args.out} --seeds {seeds}"

    # 校準
    print("calibrating CBF vs safe_apf on blind_approach (no blind)...")
    cal = calibrate_cbf(BLIND_APPROACH)
    cbf_cal = cal["calibrated"]
    print(f"  safe_apf stop dist: {cal['safe_apf_stop_dist_m']:.4f} m")
    print(f"  cbf_calibrated: buffer_m={cbf_cal['buffer_m']}, alpha={cbf_cal['alpha']}, "
          f"diff={cbf_cal['diff_vs_safe_apf_m']:.4f} m")

    # E1
    print(f"running E1 ({2 * 2} × {seeds} seeds)...")
    e1_data = run_e1(BLIND_APPROACH, cbf_cal, seeds)
    e1_json = {
        "meta": {"cmd": cmd, "seeds": seeds,
                 "cbf_calibrated": cbf_cal,
                 "safe_apf_ref_m": cal["safe_apf_stop_dist_m"],
                 "cal_diff_m": cal["best_diff_m"],
                 "calibration_sweep": cal["sweep"]},
        "scenario": "blind_approach",
        "results": e1_data,
    }
    (args.out / "e1_results.json").write_text(
        json.dumps(e1_json, ensure_ascii=False, indent=2), encoding="utf-8")
    write_e1_summary(e1_data, BLIND_APPROACH, cbf_cal,
                     {"cmd": cmd, "seeds": seeds,
                      "safe_apf_ref_m": cal["safe_apf_stop_dist_m"],
                      "cal_diff_m": cal["best_diff_m"]},
                     args.out / "e1_summary.md")
    print(f"  wrote e1_results.json + e1_summary.md")

    # E2
    e2_seeds = 3 if args.smoke else seeds
    print(f"running E2 ({3} filters × {e2_seeds} seeds)...")
    e2_data = run_e2(REVERSE_INTO_WALL, cbf_cal, e2_seeds)
    e2_json = {
        "meta": {"cmd": cmd, "seeds": e2_seeds,
                 "cbf_calibrated": cbf_cal},
        "scenario": "reverse_into_wall",
        "results": e2_data,
    }
    (args.out / "e2_results.json").write_text(
        json.dumps(e2_json, ensure_ascii=False, indent=2), encoding="utf-8")
    write_e2_summary(e2_data, REVERSE_INTO_WALL, cbf_cal,
                     {"cmd": cmd, "seeds": e2_seeds},
                     args.out / "e2_summary.md")
    print(f"  wrote e2_results.json + e2_summary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
