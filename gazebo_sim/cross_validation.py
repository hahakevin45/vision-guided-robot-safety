"""兩關交叉驗證報告產生器。

讀取 Gazebo 評估 JSON，對映到 safety_sim 情境後現跑純 Python 關，
並輸出 verdict 與核心指標的並排 Markdown 報告。
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from safety_sim.compare import run_matrix
from safety_sim.metrics import MetricsReport

from .evaluate_gs_trace import GS_TO_SAFETY_SCENARIO


METRIC_ROWS = (
    ("min_clearance", "min_clearance"),
    ("max_speed", "max_speed_mps"),
    ("time_to_stop_after_fault", "time_to_stop_after_fault_s"),
    ("intervention_ratio", "intervention_ratio"),
)


@dataclass(frozen=True)
class GazeboEval:
    path: Path
    gs_scenario: str
    safety_scenario: str
    filter_name: str
    passed: bool
    reasons: tuple[str, ...]
    metrics: dict[str, Any]


def _load_gazebo_eval(path: Path) -> GazeboEval:
    data = json.loads(path.read_text(encoding="utf-8"))
    gs_scenario = str(data["scenario"])
    safety_scenario = str(
        data.get("safety_sim_scenario") or GS_TO_SAFETY_SCENARIO[gs_scenario]
    )
    return GazeboEval(
        path=path,
        gs_scenario=gs_scenario,
        safety_scenario=safety_scenario,
        filter_name=str(data["filter"]),
        passed=bool(data["passed"]),
        reasons=tuple(str(reason) for reason in data.get("reasons", ())),
        metrics=dict(data.get("metrics", {})),
    )


def _find_eval_paths(gazebo_dir: Path) -> list[Path]:
    paths = sorted(gazebo_dir.glob("*.eval.json"))
    paths.extend(path for path in sorted(gazebo_dir.glob("*_eval.json")) if path not in paths)
    if not paths:
        raise FileNotFoundError(f"no Gazebo eval JSON found under {gazebo_dir}")
    return paths


def _scenario_sort_key(name: str) -> tuple[int, str]:
    if len(name) > 1 and name[0].isalpha() and name[1:].isdigit():
        return (int(name[1:]), name)
    return (9999, name)


def _format_verdict(passed: bool) -> str:
    return "PASS" if passed else "FAIL"


def _metric_from_report(report: MetricsReport, key: str) -> float | None:
    value = getattr(report, key)
    if isinstance(value, float) and math.isinf(value):
        return None
    return value


def _format_metric(value: Any) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, (int, float)):
        if isinstance(value, float) and math.isinf(value):
            return "N/A"
        return f"{value:.3f}"
    return str(value)


def _known_differences_reference() -> str:
    return "README.md — Safety Layer Evaluation"


def _write_markdown(
    gazebo_evals: list[GazeboEval],
    python_results: dict[tuple[str, str], Any],
    output: Path,
) -> None:
    rows = sorted(
        gazebo_evals,
        key=lambda item: (_scenario_sort_key(item.safety_scenario), item.filter_name, item.path.name),
    )

    lines: list[str] = ["# 兩關交叉驗證報告", ""]
    lines.append("## Verdict 對照表")
    lines.append("")
    lines.append("| scenario | filter | python 關 | Gazebo 關 | 一致 |")
    lines.append("|---|---|---|---|---|")

    mismatches: list[tuple[GazeboEval, Any]] = []
    for item in rows:
        py = python_results[(item.safety_scenario, item.filter_name)]
        agrees = py.passed == item.passed
        if not agrees:
            mismatches.append((item, py))
        lines.append(
            f"| {item.safety_scenario} / {item.gs_scenario} | {item.filter_name} | "
            f"{_format_verdict(py.passed)} | {_format_verdict(item.passed)} | "
            f"{'✓' if agrees else '✗'} |"
        )
    lines.append("")

    lines.append("## 指標並排表")
    lines.append("")
    lines.append("| scenario | filter | metric | python 關 | Gazebo 關 |")
    lines.append("|---|---|---|---|---|")
    for item in rows:
        py = python_results[(item.safety_scenario, item.filter_name)]
        for label, key in METRIC_ROWS:
            lines.append(
                f"| {item.safety_scenario} / {item.gs_scenario} | {item.filter_name} | "
                f"{label} | {_format_metric(_metric_from_report(py.report, key))} | "
                f"{_format_metric(item.metrics.get(key))} |"
            )
    lines.append("")

    lines.append("## 差異解讀")
    lines.append("")
    if mismatches:
        for item, py in mismatches:
            lines.append(
                f"- {item.safety_scenario} / {item.gs_scenario} / {item.filter_name} "
                f"verdict 不一致：python 關 {_format_verdict(py.passed)}，"
                f"Gazebo 關 {_format_verdict(item.passed)}。"
            )
            if py.reasons:
                lines.append(f"  Python reasons: {'; '.join(py.reasons)}")
            if item.reasons:
                lines.append(f"  Gazebo reasons: {'; '.join(item.reasons)}")
    else:
        lines.append("- 未發現 verdict 不一致。")
    lines.append("")
    lines.append(
        "- 已知模型差異：撞牆判定門檻 0.05m；GS2 停車時限放寬 3s；"
        "Gazebo 非決定性，因此此報告以 threshold smoke 與 verdict 對照為主。"
    )
    lines.append(f"- 評估 JSON 未包含完整模型差異解釋時，引用 {_known_differences_reference()}。")
    lines.append("")

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8")


def write_cross_validation_report(gazebo_dir: str | Path, output: str | Path) -> None:
    """讀取 Gazebo 評估 JSON，現跑 safety_sim 並寫出兩關交叉驗證報告。"""
    gazebo_dir = Path(gazebo_dir)
    output = Path(output)
    gazebo_evals = [_load_gazebo_eval(path) for path in _find_eval_paths(gazebo_dir)]

    filter_names = sorted({item.filter_name for item in gazebo_evals})
    scenario_names = sorted(
        {item.safety_scenario for item in gazebo_evals},
        key=_scenario_sort_key,
    )
    python_results = run_matrix(filter_names, scenario_names)
    _write_markdown(gazebo_evals, python_results, output)


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate Gazebo/safety_sim cross-validation report.")
    parser.add_argument("--gazebo-dir", type=Path, required=True, help="Gazebo eval JSON directory")
    parser.add_argument("--output", type=Path, required=True, help="Markdown report output path")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI 入口，回傳 process exit code。"""
    args = _parse_args(argv)
    write_cross_validation_report(args.gazebo_dir, args.output)
    print(f"report   {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
