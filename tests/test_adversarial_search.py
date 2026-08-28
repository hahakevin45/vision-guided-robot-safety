import json

from safety_sim.adversarial_search import (
    SearchParams,
    classify_hypotheses,
    classify_failure,
    generate_param_grid,
    run_search,
    write_outputs,
)


def test_failure_criteria_classify_independent_flags():
    metrics = {
        "collided": True,
        "min_clearance": 0.10,
        "stuck": False,
        "max_speed_exceeded": False,
    }
    assert classify_failure(metrics) == ["collided"]

    metrics = {
        "collided": False,
        "min_clearance": 0.019,
        "stuck": True,
        "max_speed_exceeded": True,
    }
    assert classify_failure(metrics) == ["low_clearance", "stuck"]


def test_small_grid_smoke_finds_results_under_twenty_combinations():
    grid = list(
        generate_param_grid(
            max_v_values=(0.15,),
            noise_values=(0.04,),
            update_hz_values=(15.0,),
            motor_tau_values=(0.08,),
            corner_angle_values=(30.0,),
            blackout_duration_values=(0.6,),
            scenario_families=("S1", "S4", "S6"),
        )
    )
    assert len(grid) == 3

    results = run_search(grid, top=10)
    assert len(results) <= 10
    assert all(result["failure_types"] for result in results)


def test_hypothesis_classification_uses_single_axis_ablation_flip():
    base = SearchParams("S1R", 0.25, 0.15, 4.0, 0.3)
    calls = []

    def fake_runner(params):
        calls.append(params)
        if params.scenario_family == "S1":
            return {
                "collided": False,
                "min_clearance": 0.05,
                "stuck": False,
                "clearance_crossings": 0,
            }
        return {
            "collided": True,
            "min_clearance": -0.05,
            "stuck": False,
            "clearance_crossings": 3,
        }

    hypotheses, evidence = classify_hypotheses(
        base,
        {"collided": True, "min_clearance": -0.05, "stuck": False, "clearance_crossings": 3},
        runner=fake_runner,
    )

    assert hypotheses == ["reverse_lookahead_geometry_error"]
    assert evidence["reverse_lookahead_geometry_error"]["flipped"] is True
    assert evidence["actuator_delay_braking_underestimate"]["flipped"] is False
    assert any(call.motor_time_constant_s == 0.08 for call in calls)
    assert any(call.noise_xy_std == 0.04 for call in calls)
    assert any(call.update_hz == 15.0 for call in calls)
    assert any(call.scenario_family == "S1" for call in calls)


def test_run_search_retains_top_results_per_family(monkeypatch):
    grid = [
        SearchParams("S1", 0.15, 0.04, 15.0, 0.08),
        SearchParams("S1", 0.25, 0.04, 15.0, 0.08),
        SearchParams("S6", 0.15, 0.04, 15.0, 0.08, corner_angle_deg=30.0),
    ]

    def fake_run_case(params):
        return {
            "collided": True,
            "min_clearance": -params.max_v_mps,
            "stuck": False,
            "max_speed_mps": params.max_v_mps,
            "max_speed_exceeded": False,
            "distance_traveled_m": params.max_v_mps,
            "clearance_crossings": 0,
            "stop_ratio": 0.0,
        }

    monkeypatch.setattr("safety_sim.adversarial_search.run_case", fake_run_case)

    results = run_search(grid, top=1)

    assert [result["scenario_family"] for result in results] == ["S1", "S6"]


def test_json_schema_for_written_failures(tmp_path):
    result = {
        "scenario_family": "S1",
        "parameters": {
            "max_v_mps": 0.15,
            "noise_xy_std": 0.04,
            "update_hz": 15.0,
            "motor_time_constant_s": 0.08,
        },
        "metrics": {
            "collided": False,
            "min_clearance": 0.01,
            "stuck": False,
            "max_speed_mps": 0.12,
            "max_speed_exceeded": False,
        },
        "failure_types": ["low_clearance"],
        "hypotheses": ["actuator_delay_braking_underestimate"],
        "ablation_evidence": {
            "actuator_delay_braking_underestimate": {
                "axis": "motor_time_constant_s",
                "nominal_value": 0.08,
                "ablated_parameters": {
                    "max_v_mps": 0.15,
                    "noise_xy_std": 0.04,
                    "update_hz": 15.0,
                    "motor_time_constant_s": 0.08,
                },
                "ablated_failure_types": [],
                "ablated_metrics": {
                    "collided": False,
                    "min_clearance": 0.05,
                    "stuck": False,
                },
                "flipped": True,
            }
        },
    }
    output = tmp_path / "failures.json"
    write_outputs([result], output, top=1, total_runs=1)

    data = json.loads(output.read_text(encoding="utf-8"))
    assert data["top_per_family"] == 1
    assert data["total_runs"] == 1
    assert data["failures"][0]["scenario_family"] == "S1"
    assert set(data["failures"][0]) == {
        "scenario_family",
        "parameters",
        "metrics",
        "failure_types",
        "hypotheses",
        "ablation_evidence",
    }
    assert output.with_suffix(".md").exists()
