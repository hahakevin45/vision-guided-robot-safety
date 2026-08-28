"""compare：filters × scenarios 全矩陣 → markdown 比較表。"""
from pathlib import Path

from safety_sim.cli import main
from safety_sim.report import write_compare_markdown
from safety_sim.compare import run_matrix


def test_run_matrix_covers_all_cells():
    results = run_matrix(["passthrough", "cbf"], ["S1", "S7"])
    assert set(results.keys()) == {("S1", "passthrough"), ("S1", "cbf"),
                                   ("S7", "passthrough"), ("S7", "cbf")}
    s1_pass = results[("S1", "passthrough")]
    assert s1_pass.passed is False
    assert s1_pass.report.collided is True
    assert results[("S1", "cbf")].passed is True


def test_write_compare_markdown(tmp_path: Path):
    results = run_matrix(["passthrough", "clamp_watchdog", "cbf"], ["S1", "S2"])
    out = tmp_path / "compare.md"
    write_compare_markdown(results, out)
    text = out.read_text()
    assert "| S1 |" in text
    assert "PASS" in text and "FAIL" in text
    # 活性指標要出現在報告裡，不能只報安全面。
    assert "intervention" in text
    assert "min_clearance" in text


def test_cli_compare(tmp_path: Path, capsys):
    out = tmp_path / "compare.md"
    rc = main(["compare", "--filters", "passthrough,cbf",
               "--scenarios", "S1,S7", "--output", str(out)])
    assert rc == 0
    assert out.exists()
    assert "S1" in capsys.readouterr().out


def test_cli_compare_defaults_to_all(tmp_path: Path):
    out = tmp_path / "compare_all.md"
    rc = main(["compare", "--output", str(out)])
    assert rc == 0
    text = out.read_text()
    for name in ("S1", "S7", "cbf", "clamp_watchdog", "passthrough"):
        assert name in text
