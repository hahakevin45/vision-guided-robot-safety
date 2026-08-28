"""safety_sim.report / cli：單次執行的軌跡圖與命令列入口。"""
from pathlib import Path

from safety_sim.cli import main
from safety_sim.filters import make_filter
from safety_sim.report import plot_trace
from safety_sim.runner import run_scenario
from safety_sim.scenarios import get_scenario


def test_plot_trace_writes_png(tmp_path: Path):
    scenario = get_scenario("S2")
    trace = run_scenario(scenario, make_filter("clamp_watchdog"))
    out = tmp_path / "s2_clamp.png"
    plot_trace(trace, scenario, out)
    assert out.exists()
    assert out.stat().st_size > 10_000   # 真的有畫東西，不是空檔


def test_cli_run_prints_metrics_and_verdict(tmp_path: Path, capsys):
    out = tmp_path / "s1.png"
    rc = main(["run", "--scenario", "S1", "--filter", "passthrough",
               "--plot", str(out)])
    captured = capsys.readouterr().out
    assert rc == 1                        # 情境不通過 → 非零退出碼
    assert "collided" in captured
    assert "FAIL" in captured
    assert out.exists()


def test_cli_run_pass_returns_zero(capsys):
    rc = main(["run", "--scenario", "S2", "--filter", "clamp_watchdog"])
    captured = capsys.readouterr().out
    assert rc == 0
    assert "PASS" in captured


def test_cli_list(capsys):
    rc = main(["list"])
    captured = capsys.readouterr().out
    assert rc == 0
    assert "S1" in captured and "clamp_watchdog" in captured
