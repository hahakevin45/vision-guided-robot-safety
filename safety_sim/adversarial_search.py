"""Adversarial parameter search for the halfspace-projection CBF filter."""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Iterable

from . import metrics
from .faults import FaultSchedule, FaultWindow
from .filters import make_filter
from .nav import ScriptedNav
from .runner import Trace, run_scenario
from .scenario import Scenario
from .scenarios.basic import _make_arena
from .types import Pose, Twist


@dataclass(frozen=True)
class SearchParams:
    scenario_family: str
    max_v_mps: float
    noise_xy_std: float
    update_hz: float
    motor_time_constant_s: float
    corner_angle_deg: float | None = None
    blackout_duration_s: float | None = None


NOMINAL_NOISE_XY_STD = 0.04
NOMINAL_UPDATE_HZ = 15.0
NOMINAL_MOTOR_TAU_S = 0.08

HYPOTHESIS_LABELS = {
    "noise_boundary_limit_cycle": "(a) 噪聲在邊界引發 limit cycle",
    "low_update_rate_observation_staleness": "(b) 低更新率/觀測延遲造成保守性不足",
    "actuator_delay_braking_underestimate": "(c) 執行器延遲導致煞距低估",
    "reverse_lookahead_geometry_error": "(d) 倒車命令下 lookahead 幾何錯誤",
}


def generate_param_grid(
    *,
    max_v_values: tuple[float, ...] = (0.15, 0.25, 0.35, 0.5),
    noise_values: tuple[float, ...] = (0.04, 0.08, 0.15),
    update_hz_values: tuple[float, ...] = (15.0, 8.0, 4.0),
    motor_tau_values: tuple[float, ...] = (0.08, 0.15, 0.3),
    corner_angle_values: tuple[float, ...] = (30.0, 45.0, 60.0),
    blackout_duration_values: tuple[float, ...] = (0.6, 1.0, 1.5),
    scenario_families: tuple[str, ...] = ("S1", "S1R", "S4", "S6"),
) -> Iterable[SearchParams]:
    for family in scenario_families:
        for max_v in max_v_values:
            for noise in noise_values:
                for hz in update_hz_values:
                    for tau in motor_tau_values:
                        if family == "S6":
                            for angle in corner_angle_values:
                                yield SearchParams(family, max_v, noise, hz, tau, corner_angle_deg=angle)
                        elif family == "S4":
                            for blackout in blackout_duration_values:
                                yield SearchParams(family, max_v, noise, hz, tau, blackout_duration_s=blackout)
                        else:
                            yield SearchParams(family, max_v, noise, hz, tau)


def make_scenario(params: SearchParams) -> Scenario:
    common = {
        "max_v_mps": params.max_v_mps,
        "localizer_kwargs": {
            "update_hz": params.update_hz,
            "noise_xy_std": params.noise_xy_std,
            "noise_theta_std": 0.02,
        },
        "vehicle_kwargs": {"motor_time_constant_s": params.motor_time_constant_s},
        "duration_s": 20.0,
    }
    if params.scenario_family == "S1":
        return Scenario(
            name="D_S1_wall_rush",
            description="adversarial S1 wall rush",
            make_world=_make_arena,
            make_nav=lambda: ScriptedNav(((0.0, Twist(2.0, 0.0)),)),
            start_pose=Pose(0.5, 0.0, 0.0),
            **common,
        )
    if params.scenario_family == "S1R":
        return Scenario(
            name="D_S1_reverse_wall_rush",
            description="adversarial reverse command toward rear wall",
            make_world=_make_arena,
            make_nav=lambda: ScriptedNav(((0.0, Twist(-2.0, 0.0)),)),
            start_pose=Pose(0.5, 0.0, 0.0),
            **common,
        )
    if params.scenario_family == "S4":
        blackout = params.blackout_duration_s if params.blackout_duration_s is not None else 1.0
        starts = (4.0, 8.0, 12.0)
        return Scenario(
            name=f"D_S4_blackout_{blackout:.1f}",
            description="adversarial intermittent pose blackout",
            make_world=_make_arena,
            make_nav=lambda: ScriptedNav(((0.0, Twist(params.max_v_mps, 0.0)),)),
            faults=FaultSchedule(tuple(FaultWindow(s, s + blackout, "aruco_dropout") for s in starts)),
            fault_t0=starts[0],
            start_pose=Pose(0.5, 0.0, 0.0),
            **common,
        )
    if params.scenario_family == "S6":
        angle = math.radians(params.corner_angle_deg if params.corner_angle_deg is not None else 45.0)
        return Scenario(
            name=f"D_S6_corner_{params.corner_angle_deg:.0f}",
            description="adversarial angled corner approach",
            make_world=_make_arena,
            make_nav=lambda: ScriptedNav(((0.0, Twist(2.0, 0.0)),)),
            start_pose=Pose(0.5, -0.5, angle),
            **common,
        )
    raise ValueError(f"unknown scenario family: {params.scenario_family}")


def _distance_traveled(trace: Trace) -> float:
    if not trace.samples:
        return 0.0
    start = trace.samples[0].true_pose
    end = trace.samples[-1].true_pose
    return math.hypot(end.x - start.x, end.y - start.y)


def _clearance_crossings(trace: Trace, threshold: float = 0.02) -> int:
    previous: bool | None = None
    crossings = 0
    for sample in trace.samples:
        current = sample.clearance < threshold
        if previous is not None and current != previous:
            crossings += 1
        previous = current
    return crossings


def _stop_ratio(trace: Trace) -> float:
    if not trace.samples:
        return 0.0
    return sum(1 for sample in trace.samples if sample.mode == "STOP") / len(trace.samples)


def evaluate_trace(trace: Trace, params: SearchParams) -> dict:
    summary = metrics.summarize(trace)
    stuck = _distance_traveled(trace) < 0.3 and not summary.collided
    return {
        "collided": summary.collided,
        "min_clearance": summary.min_clearance,
        "stuck": stuck,
        "max_speed_mps": summary.max_speed_mps,
        "max_speed_exceeded": summary.max_speed_mps > params.max_v_mps * 1.05,
        "distance_traveled_m": _distance_traveled(trace),
        "clearance_crossings": _clearance_crossings(trace),
        "stop_ratio": _stop_ratio(trace),
    }


def classify_failure(result_metrics: dict) -> list[str]:
    failures = []
    if result_metrics["collided"]:
        failures.append("collided")
    if result_metrics["min_clearance"] < 0.02:
        failures.append("low_clearance")
    if result_metrics["stuck"]:
        failures.append("stuck")
    return failures


# --filter 穿透點：預設掃 cbf；掃候選方法時整套流程（含消融重跑）
# 都用同一個 filter，兩份輸出可直接比「安全包絡大小」。
ACTIVE_FILTER = "cbf"


def run_case(params: SearchParams) -> dict:
    trace = run_scenario(make_scenario(params), make_filter(ACTIVE_FILTER))
    return evaluate_trace(trace, params)


def _ablation_candidates(params: SearchParams) -> list[tuple[str, str, float | str, SearchParams]]:
    candidates: list[tuple[str, str, float | str, SearchParams]] = []
    if params.noise_xy_std != NOMINAL_NOISE_XY_STD:
        candidates.append(
            (
                "noise_boundary_limit_cycle",
                "noise_xy_std",
                NOMINAL_NOISE_XY_STD,
                replace(params, noise_xy_std=NOMINAL_NOISE_XY_STD),
            )
        )
    if params.update_hz != NOMINAL_UPDATE_HZ:
        candidates.append(
            (
                "low_update_rate_observation_staleness",
                "update_hz",
                NOMINAL_UPDATE_HZ,
                replace(params, update_hz=NOMINAL_UPDATE_HZ),
            )
        )
    if params.motor_time_constant_s != NOMINAL_MOTOR_TAU_S:
        candidates.append(
            (
                "actuator_delay_braking_underestimate",
                "motor_time_constant_s",
                NOMINAL_MOTOR_TAU_S,
                replace(params, motor_time_constant_s=NOMINAL_MOTOR_TAU_S),
            )
        )
    if params.scenario_family == "S1R":
        candidates.append(
            (
                "reverse_lookahead_geometry_error",
                "scenario_family",
                "S1",
                replace(params, scenario_family="S1"),
            )
        )
    return candidates


def classify_hypotheses(
    params: SearchParams,
    result_metrics: dict,
    *,
    runner: Callable[[SearchParams], dict] = run_case,
) -> tuple[list[str], dict[str, dict]]:
    if not classify_failure(result_metrics):
        return [], {}
    out = []
    evidence = {}
    for hypothesis, axis, nominal_value, ablated_params in _ablation_candidates(params):
        ablated_metrics = runner(ablated_params)
        ablated_failures = classify_failure(ablated_metrics)
        flipped = not ablated_failures
        evidence[hypothesis] = {
            "axis": axis,
            "nominal_value": nominal_value,
            "ablated_parameters": _params_dict(ablated_params),
            "ablated_failure_types": ablated_failures,
            "ablated_metrics": ablated_metrics,
            "flipped": flipped,
        }
        if flipped:
            out.append(hypothesis)
    return out, evidence


def _params_dict(params: SearchParams) -> dict:
    out = {
        "max_v_mps": params.max_v_mps,
        "noise_xy_std": params.noise_xy_std,
        "update_hz": params.update_hz,
        "motor_time_constant_s": params.motor_time_constant_s,
    }
    if params.corner_angle_deg is not None:
        out["corner_angle_deg"] = params.corner_angle_deg
    if params.blackout_duration_s is not None:
        out["blackout_duration_s"] = params.blackout_duration_s
    return out


def _severity(result: dict) -> tuple[float, float]:
    metrics_ = result["metrics"]
    return (metrics_["min_clearance"], -metrics_.get("distance_traveled_m", 0.0))


def _retain_top_per_family(failures: list[dict], top: int) -> list[dict]:
    grouped: dict[str, list[dict]] = {}
    for failure in failures:
        grouped.setdefault(failure["scenario_family"], []).append(failure)
    retained = []
    for family in sorted(grouped):
        retained.extend(sorted(grouped[family], key=_severity)[:top])
    return sorted(retained, key=lambda failure: (failure["scenario_family"], _severity(failure)))


def _family_totals(grid: Iterable[SearchParams]) -> dict[str, int]:
    totals: dict[str, int] = {}
    for params in grid:
        totals[params.scenario_family] = totals.get(params.scenario_family, 0) + 1
    return totals


def collect_failures(grid: Iterable[SearchParams]) -> list[dict]:
    failures = []
    for params in grid:
        result_metrics = run_case(params)
        failure_types = classify_failure(result_metrics)
        if not failure_types:
            continue
        hypotheses, ablation_evidence = classify_hypotheses(params, result_metrics)
        failures.append(
            {
                "scenario_family": params.scenario_family,
                "parameters": _params_dict(params),
                "metrics": result_metrics,
                "failure_types": failure_types,
                "hypotheses": hypotheses,
                "ablation_evidence": ablation_evidence,
            }
        )
    return sorted(failures, key=lambda failure: (failure["scenario_family"], _severity(failure)))


def run_search(grid: Iterable[SearchParams], *, top: int) -> list[dict]:
    return _retain_top_per_family(collect_failures(grid), top)


def _mildest_by_group(failures: list[dict], key: str) -> dict[str, dict]:
    grouped: dict[str, list[dict]] = {}
    for failure in failures:
        for item in failure[key]:
            grouped.setdefault(item, []).append(failure)
    return {name: sorted(items, key=_severity, reverse=True)[0] for name, items in grouped.items()}


def _nominal_parameter_failure(failure: dict) -> bool:
    params = failure["parameters"]
    return (
        params.get("noise_xy_std") == NOMINAL_NOISE_XY_STD
        and params.get("update_hz") == NOMINAL_UPDATE_HZ
        and params.get("motor_time_constant_s") == NOMINAL_MOTOR_TAU_S
    )


def _format_ablation_evidence(failure: dict, hypothesis: str) -> str:
    evidence = failure.get("ablation_evidence", {}).get(hypothesis)
    if not evidence:
        return "no ablation evidence"
    return (
        f"撥回 `{evidence['axis']}` 到 `{evidence['nominal_value']}` -> "
        f"{'PASS' if evidence['flipped'] else '仍 FAIL'} "
        f"(failures={evidence['ablated_failure_types']})"
    )


def _counts_by_family(failures: list[dict]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for failure in failures:
        counts[failure["scenario_family"]] = counts.get(failure["scenario_family"], 0) + 1
    return counts


def render_markdown(
    failures: list[dict],
    *,
    total_runs: int,
    top: int,
    all_failures: list[dict] | None = None,
) -> str:
    summary_failures = all_failures if all_failures is not None else failures
    family_counts: dict[str, int] = {}
    family_totals: dict[str, int | str] = {}
    for failure in summary_failures:
        family = failure["scenario_family"]
        family_counts[family] = family_counts.get(family, 0) + 1
        family_totals[family] = failure.get("family_total_runs", "unknown")
    lines = [
        "# CBF Adversarial Search Summary",
        "",
        f"- Total parameter runs: {total_runs}",
        f"- Failures retained: {len(failures)} / top {top} per scenario family",
        "",
        "## Failures By Scenario Family",
    ]
    for family in sorted(family_counts):
        family_failures = [failure for failure in summary_failures if failure["scenario_family"] == family]
        mildest = sorted(family_failures, key=_severity, reverse=True)[0]
        lines.extend(
            [
                "",
                f"### {family}",
                f"- failures observed: {family_counts[family]} / total {family_totals[family]}",
                "- mildest observed failure:",
                _format_failure(mildest),
            ]
        )

    lines.extend(["", "## Nominal Parameter Failures"])
    nominal = [failure for failure in summary_failures if _nominal_parameter_failure(failure)]
    if nominal:
        nominal_s1r_speeds = sorted(
            {
                failure["parameters"]["max_v_mps"]
                for failure in nominal
                if failure["scenario_family"] == "S1R" and failure["metrics"].get("collided")
            }
        )
        if nominal_s1r_speeds:
            lines.extend(
                [
                    "",
                    "- S1R nominal reverse wall rush collides across max_v_mps="
                    f"{', '.join(str(speed) for speed in nominal_s1r_speeds)}",
                ]
            )
        for failure in sorted(
            nominal,
            key=lambda item: (
                item["scenario_family"],
                item["parameters"].get("max_v_mps", 0.0),
                _severity(item),
            ),
        ):
            lines.extend(["", _format_failure(failure)])
    else:
        lines.extend(["", "- none retained"])

    lines.extend(["", "## Hypothesis Check"])
    by_hypothesis = _mildest_by_group(summary_failures, "hypotheses")
    for key, label in HYPOTHESIS_LABELS.items():
        if key in by_hypothesis:
            lines.extend(
                [
                    "",
                    f"- Found {label}: yes",
                    f"  - {_format_failure_inline(by_hypothesis[key])}",
                    f"  - evidence: {_format_ablation_evidence(by_hypothesis[key], key)}",
                ]
            )
        else:
            lines.extend(["", f"- Found {label}: no"])
    lines.append("")
    return "\n".join(lines)


def _format_failure(failure: dict) -> str:
    return (
        f"- scenario: {failure['scenario_family']}\n"
        f"- params: `{json.dumps(failure['parameters'], sort_keys=True)}`\n"
        f"- metrics: `{json.dumps(failure['metrics'], sort_keys=True)}`\n"
        f"- hypotheses: {', '.join(failure['hypotheses']) or 'none'}"
    )


def _format_failure_inline(failure: dict) -> str:
    m = failure["metrics"]
    return (
        f"{failure['scenario_family']} params="
        f"`{json.dumps(failure['parameters'], sort_keys=True)}`, "
        f"min_clearance={m['min_clearance']:.3f}, stuck={m['stuck']}, collided={m['collided']}"
    )


def write_outputs(
    failures: list[dict],
    output: Path,
    *,
    top: int,
    total_runs: int,
    all_failures: list[dict] | None = None,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    summary_failures = all_failures if all_failures is not None else failures
    payload = {
        "top_per_family": top,
        "total_runs": total_runs,
        "failure_counts_by_family": _counts_by_family(summary_failures),
        "nominal_parameter_failures": [
            failure for failure in summary_failures if _nominal_parameter_failure(failure)
        ],
        "all_failures": summary_failures,
        "failures": failures,
    }
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8")
    output.with_suffix(".md").write_text(
        render_markdown(failures, total_runs=total_runs, top=top, all_failures=all_failures), encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--top", type=int, default=30)
    parser.add_argument("--filter", default="cbf", help="safety_sim filter 名（預設 cbf）")
    args = parser.parse_args()
    global ACTIVE_FILTER
    ACTIVE_FILTER = args.filter

    grid = list(generate_param_grid())
    family_totals = _family_totals(grid)
    all_failures = collect_failures(grid)
    for failure in all_failures:
        failure["family_total_runs"] = family_totals[failure["scenario_family"]]
    failures = _retain_top_per_family(all_failures, args.top)
    write_outputs(failures, args.output, top=args.top, total_runs=len(grid), all_failures=all_failures)
    print(f"{len(failures)} failures written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
