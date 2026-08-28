"""命令列入口。

    python3 -m safety_sim list
    python3 -m safety_sim run --scenario S2 --filter clamp_watchdog --plot out.png

run 的退出碼：情境通過 0、不通過 1，可直接接 CI。
"""
from __future__ import annotations

import argparse
from dataclasses import asdict

from . import metrics
from .filters import available_filters, make_filter
from .runner import run_scenario
from .scenarios import all_scenario_names, get_scenario


def _cmd_list() -> int:
    print("scenarios:")
    for name in all_scenario_names():
        print(f"  {name}: {get_scenario(name).description}")
    print("filters:")
    for name in available_filters():
        print(f"  {name}")
    return 0


def _cmd_run(scenario_name: str, filter_name: str, plot_path: str | None) -> int:
    scenario = get_scenario(scenario_name)
    trace = run_scenario(scenario, make_filter(filter_name))
    report = metrics.summarize(trace, fault_t0=scenario.fault_t0)

    print(f"scenario {scenario.name} ({scenario.description})")
    print(f"filter   {filter_name}")
    for key, value in asdict(report).items():
        print(f"  {key}: {value}")

    passed, reasons = scenario.evaluate(trace)
    print(f"verdict  {'PASS' if passed else 'FAIL'}")
    for reason in reasons:
        print(f"  - {reason}")

    if plot_path:
        from .report import plot_trace   # matplotlib 只在需要時載入
        plot_trace(trace, scenario, plot_path)
        print(f"plot     {plot_path}")
    return 0 if passed else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="safety_sim")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="列出情境與 filter")

    run_p = sub.add_parser("run", help="跑單一 (scenario, filter) 並輸出指標")
    run_p.add_argument("--scenario", required=True)
    run_p.add_argument("--filter", required=True)
    run_p.add_argument("--plot", default=None, help="輸出軌跡圖 PNG 路徑")

    cmp_p = sub.add_parser("compare", help="跑 filters × scenarios 矩陣並輸出 markdown 比較表")
    cmp_p.add_argument("--filters", default=None, help="逗號分隔，預設全部")
    cmp_p.add_argument("--scenarios", default=None, help="逗號分隔，預設全部")
    cmp_p.add_argument("--output", required=True, help="markdown 輸出路徑")

    args = parser.parse_args(argv)
    if args.command == "list":
        return _cmd_list()
    if args.command == "compare":
        return _cmd_compare(args.filters, args.scenarios, args.output)
    return _cmd_run(args.scenario, args.filter, args.plot)


def _cmd_compare(filters_arg: str | None, scenarios_arg: str | None,
                 output: str) -> int:
    from .compare import run_matrix
    from .report import write_compare_markdown

    filter_names = (filters_arg.split(",") if filters_arg
                    else available_filters())
    scenario_names = (scenarios_arg.split(",") if scenarios_arg
                      else all_scenario_names())
    results = run_matrix(filter_names, scenario_names)

    for s in scenario_names:
        row = ", ".join(
            f"{f}={'PASS' if results[(s, f)].passed else 'FAIL'}"
            for f in filter_names)
        print(f"{s}: {row}")
    write_compare_markdown(results, output)
    print(f"report   {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
