"""GS Gazebo trace 評估器。

讀取 `trace_recorder` 產生的 JSONL，轉成 safety_sim 的 Trace 後沿用既有
metrics/scenario 判定；Gazebo 實體 collision 外緣比 safety_sim 的
robot_radius=0.23 m 仍留額外餘裕，所以 Gazebo 驗收把 clearance < 0.05 m
視為撞牆。
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
from pathlib import Path
from typing import Any

from safety_sim import metrics
from safety_sim.scenario import Scenario
from safety_sim.scenarios import get_scenario

from gazebo_sim.trace_adapter import load_trace


GAZEBO_COLLISION_CLEARANCE_M = 0.05
GS_TO_SAFETY_SCENARIO = {
    "GS1": "S1",
    "GS2": "S2",
    "GS3": "S8",
}

# GS3 單障礙繞行門檻：goal 距離、橫向偏離、安全命令樣本。
GS3_GOAL = (3.2, 0.0)
GS3_MAX_FINAL_DIST_M = 0.15
GS3_MIN_LATERAL_DEV_M = 0.30
GS3_MIN_CLEARANCE_M = 0.05


def _finite_or_none(value: float) -> float | None:
    return value if math.isfinite(value) else None


def _metrics_dict(report: metrics.MetricsReport) -> dict[str, Any]:
    data = dataclasses.asdict(report)
    data["time_to_stop_after_fault_s"] = _finite_or_none(data["time_to_stop_after_fault_s"])
    return data


def _scenario_for_gazebo(gs_name: str) -> Scenario:
    try:
        scenario = get_scenario(GS_TO_SAFETY_SCENARIO[gs_name])
    except KeyError:
        raise ValueError(f"unknown GS scenario {gs_name!r}; expected GS1 or GS2") from None
    if gs_name == "GS2":
        # ROS topic/service 傳輸加上 20 Hz 離散化會讓 Gazebo 的 STOP 觀測時間
        # 比純 Python runner 鬆散；G2 驗收放寬為 3.0s，但不改 safety_sim 本體。
        scenario = dataclasses.replace(
            scenario,
            expectation=dataclasses.replace(
                scenario.expectation,
                stop_within_s_after_fault=3.0,
            ),
        )
    return scenario


def _infer_fault_t0_from_pose_age(trace) -> float | None:
    """從 pose_age 持續上升的最後一段回推 dropout 起點。

    取「最後」而非「第一」段：aruco 節點 stamp 與 recorder clock 之間可能
    存在固定偏移（Gazebo 大量 topic 流量下的時鐘同步 artifact），會產生
    一段 age 從 0 爬升到固定 lag 的偽段；dropout 才是唯一 age 持續無界
    上升的段（2026-08-09 GS2 實測）。單一故障情境下最後一段即故障段。
    """
    candidate: float | None = None
    prev = None
    for sample in trace.samples:
        if not math.isfinite(sample.pose_age_s):
            prev = sample
            continue
        if prev is not None and sample.pose_age_s >= 0.20 and sample.pose_age_s > prev.pose_age_s + 0.02:
            # dropout 段 age = t - freeze_time 精確成立，直接反投影凍結時刻
            candidate = sample.t - sample.pose_age_s
        prev = sample
    return candidate


def _gazebo_collided(trace) -> bool:
    return any(sample.clearance < GAZEBO_COLLISION_CLEARANCE_M for sample in trace.samples)


def evaluate_trace(
    trace_path: str | Path,
    scenario_name: str,
    *,
    fault_t0: float | None = None,
) -> dict[str, Any]:
    """回傳可 JSON 序列化的 GS 評估報告。"""
    scenario = _scenario_for_gazebo(scenario_name)
    trace = load_trace(trace_path, scenario.make_world())
    if not trace.samples:
        # 空 trace 視為失敗（recorder 沒收到任何 cmd_vel_safe）。
        empty_summary = metrics.MetricsReport(
            collided=False,
            min_clearance=math.inf,
            max_speed_mps=0.0,
            time_to_stop_after_fault_s=math.inf,
            intervention_ratio=0.0,
            cmd_distortion=0.0,
        )
        report = {
            "scenario": scenario_name,
            "safety_sim_scenario": scenario.name,
            "filter": trace.filter_name,
            "passed": False,
            "reasons": ["no trace samples"],
            "metrics": _metrics_dict(empty_summary),
        }
        if scenario_name == "GS3":
            report["gs3"] = {
                "goal": list(GS3_GOAL),
                "final_goal_distance_m": math.inf,
                "max_lateral_deviation_m": 0.0,
                "max_abs_gamma_rad": 0.0,
                "safe_command_samples": 0,
                "pass": False,
                "reasons": ["no trace samples"],
            }
        return report
    if fault_t0 is None and scenario_name == "GS2":
        fault_t0 = _infer_fault_t0_from_pose_age(trace)

    summary = metrics.summarize(trace, fault_t0=fault_t0)
    gazebo_collision = _gazebo_collided(trace)
    summary = dataclasses.replace(summary, collided=gazebo_collision)

    eval_scenario = dataclasses.replace(scenario, fault_t0=fault_t0)
    passed, reasons = eval_scenario.evaluate(trace)

    if scenario.expectation.require_no_collision and gazebo_collision:
        passed = False
        min_clearance = metrics.min_clearance(trace)
        reasons = [
            reason for reason in reasons
            if reason != "collided (or left geofence)"
        ]
        reasons.insert(
            0,
            f"Gazebo clearance {min_clearance:.3f} m < "
            f"{GAZEBO_COLLISION_CLEARANCE_M:.3f} m collision threshold",
        )

    gs3: dict[str, Any] | None = None
    if scenario_name == "GS3":
        gs3 = _evaluate_gs3(trace)
        if not gs3["pass"]:
            passed = False
            reasons = [reason for reason in reasons if reason != "-"]
            reasons.extend(gs3["reasons"])

    report = {
        "scenario": scenario_name,
        "safety_sim_scenario": scenario.name,
        "filter": trace.filter_name,
        "passed": passed,
        "reasons": reasons,
        "metrics": _metrics_dict(summary),
    }
    if gs3 is not None:
        report["gs3"] = gs3
    return report


def _evaluate_gs3(trace) -> dict[str, Any]:
    """GS3 專屬門檻：繞行必須發生、goal 必須到達、淨空必須守住。

    所有數據皆來自 ground-truth trace 與 `/safety_gate/status` debug；
    filter 的 runtime 決策不讀這些值。
    """
    data: dict[str, Any] = {
        "goal": list(GS3_GOAL),
        "final_goal_distance_m": math.inf,
        "max_lateral_deviation_m": 0.0,
        "max_abs_gamma_rad": 0.0,
        "safe_command_samples": 0,
        "pass": False,
        "reasons": [],
    }
    if not trace.samples:
        data["reasons"].append("no trace samples")
        return data
    final = trace.samples[-1].true_pose
    data["final_goal_distance_m"] = math.hypot(
        final.x - GS3_GOAL[0], final.y - GS3_GOAL[1]
    )
    data["max_lateral_deviation_m"] = max(abs(s.true_pose.y) for s in trace.samples)
    data["max_abs_gamma_rad"] = max(
        (float(s.debug.get("max_abs_gamma_rad", 0.0)) for s in trace.samples),
        default=0.0,
    )
    data["safe_command_samples"] = sum(1 for s in trace.samples if s.mode == "MODIFIED")
    reasons: list[str] = []
    if data["final_goal_distance_m"] > GS3_MAX_FINAL_DIST_M:
        reasons.append(
            f"did not reach goal: final distance {data['final_goal_distance_m']:.3f} m "
            f"> {GS3_MAX_FINAL_DIST_M} m"
        )
    if data["max_lateral_deviation_m"] < GS3_MIN_LATERAL_DEV_M:
        reasons.append(
            f"no detour: max |y| {data['max_lateral_deviation_m']:.3f} m "
            f"< {GS3_MIN_LATERAL_DEV_M} m"
        )
    min_clear = metrics.min_clearance(trace)
    if min_clear < GS3_MIN_CLEARANCE_M:
        reasons.append(f"clearance {min_clear:.3f} m < {GS3_MIN_CLEARANCE_M} m")
    if data["safe_command_samples"] <= 0:
        reasons.append("no MODIFIED safe command samples")
    data["pass"] = not reasons
    data["reasons"] = reasons
    return data


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a GS Gazebo JSONL trace.")
    parser.add_argument("trace_jsonl", help="trace_recorder JSONL path")
    parser.add_argument("scenario", choices=sorted(GS_TO_SAFETY_SCENARIO))
    parser.add_argument("--fault-t0", type=float, default=None,
                        help="sim time when GS2 dropout service was called")
    parser.add_argument("--output", type=Path, default=None,
                        help="optional JSON report output path")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    report = evaluate_trace(args.trace_jsonl, args.scenario, fault_t0=args.fault_t0)
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
