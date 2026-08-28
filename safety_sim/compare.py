"""filters × scenarios 全矩陣執行，供比較表與 CLI 使用。"""
from __future__ import annotations

from dataclasses import dataclass

from . import metrics
from .filters import make_filter
from .runner import run_scenario
from .scenarios import get_scenario


@dataclass(frozen=True)
class CellResult:
    scenario_name: str
    filter_name: str
    passed: bool
    reasons: tuple[str, ...]
    report: metrics.MetricsReport


def run_matrix(filter_names: list[str],
               scenario_names: list[str]) -> dict[tuple[str, str], CellResult]:
    results: dict[tuple[str, str], CellResult] = {}
    for scenario_name in scenario_names:
        scenario = get_scenario(scenario_name)
        for filter_name in filter_names:
            trace = run_scenario(scenario, make_filter(filter_name))
            passed, reasons = scenario.evaluate(trace)
            results[(scenario_name, filter_name)] = CellResult(
                scenario_name=scenario_name,
                filter_name=filter_name,
                passed=passed,
                reasons=tuple(reasons),
                report=metrics.summarize(trace, fault_t0=scenario.fault_t0),
            )
    return results
