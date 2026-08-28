"""gazebo_sim.cross_validation：兩關交叉驗證 markdown 報告。"""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from safety_sim.metrics import MetricsReport


def _write_eval(path: Path, *, scenario: str, safety_scenario: str,
                filter_name: str, passed: bool, min_clearance: float) -> None:
    path.write_text(
        json.dumps(
            {
                "scenario": scenario,
                "safety_sim_scenario": safety_scenario,
                "filter": filter_name,
                "passed": passed,
                "reasons": [] if passed else ["fake Gazebo failure"],
                "metrics": {
                    "collided": not passed,
                    "min_clearance": min_clearance,
                    "max_speed_mps": 0.15,
                    "time_to_stop_after_fault_s": None,
                    "intervention_ratio": 0.25,
                    "cmd_distortion": 0.5,
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )


def test_cross_validation_report_marks_matches_and_mismatches(
    tmp_path: Path, monkeypatch
) -> None:
    from gazebo_sim import cross_validation

    gazebo_dir = tmp_path / "gazebo"
    gazebo_dir.mkdir()
    _write_eval(
        gazebo_dir / "GS1_cbf.eval.json",
        scenario="GS1",
        safety_scenario="S1",
        filter_name="cbf",
        passed=True,
        min_clearance=0.18,
    )
    _write_eval(
        gazebo_dir / "GS2_passthrough.eval.json",
        scenario="GS2",
        safety_scenario="S2",
        filter_name="passthrough",
        passed=False,
        min_clearance=-0.02,
    )

    def fake_run_matrix(filter_names: list[str], scenario_names: list[str]):
        assert filter_names == ["cbf", "passthrough"]
        assert scenario_names == ["S1", "S2"]
        return {
            ("S1", "cbf"): SimpleNamespace(
                passed=True,
                reasons=(),
                report=MetricsReport(False, 0.20, 0.14, float("inf"), 0.20, 0.4),
            ),
            ("S2", "passthrough"): SimpleNamespace(
                passed=True,
                reasons=(),
                report=MetricsReport(False, 0.40, 0.15, 0.80, 0.10, 0.3),
            ),
        }

    monkeypatch.setattr(cross_validation, "run_matrix", fake_run_matrix)

    output = tmp_path / "cross_validation.md"
    cross_validation.write_cross_validation_report(gazebo_dir, output)

    text = output.read_text(encoding="utf-8")
    assert "# 兩關交叉驗證報告" in text
    assert "| scenario | filter | python 關 | Gazebo 關 | 一致 |" in text
    assert "| S1 / GS1 | cbf | PASS | PASS | ✓ |" in text
    assert "| S2 / GS2 | passthrough | PASS | FAIL | ✗ |" in text
    assert "| min_clearance | 0.200 | 0.180 |" in text
    assert "## 差異解讀" in text
    assert "S2 / GS2 / passthrough" in text
    assert "verdict 不一致：python 關 PASS，Gazebo 關 FAIL" in text
    assert "撞牆判定門檻 0.05m" in text
    assert "GS2 停車時限放寬 3s" in text
    assert "Gazebo 非決定性" in text
    assert "README.md" in text


def test_cross_validation_cli_writes_output(tmp_path: Path, monkeypatch) -> None:
    from gazebo_sim import cross_validation

    gazebo_dir = tmp_path / "gazebo"
    gazebo_dir.mkdir()
    _write_eval(
        gazebo_dir / "GS1_cbf_20260703_135259.eval.json",
        scenario="GS1",
        safety_scenario="S1",
        filter_name="cbf",
        passed=True,
        min_clearance=0.18,
    )

    monkeypatch.setattr(
        cross_validation,
        "run_matrix",
        lambda filter_names, scenario_names: {
            ("S1", "cbf"): SimpleNamespace(
                passed=True,
                reasons=(),
                report=MetricsReport(False, 0.20, 0.14, float("inf"), 0.20, 0.4),
            )
        },
    )

    output = tmp_path / "report.md"
    rc = cross_validation.main(["--gazebo-dir", str(gazebo_dir), "--output", str(output)])

    assert rc == 0
    assert output.exists()
    assert "S1 / GS1" in output.read_text(encoding="utf-8")
