"""R1 盲走漂移分析器測試（spec 8.4-8.6、15）。

- 定位誤差（physical−fused）進 b/k；路徑誤差（physical−intended）不進。
- b = ceil_0.01(max 0m radial + instrument resolution)。
- k_raw = max(max(0,e−b)/s)；k = ceil_0.05(k_raw)。
- per-speed k 與 shared k：shared 必須覆蓋全部樣本，禁止平均掉較大誤差。
- R3 continuous_visual 附錄絕不進 b/k。
- 合法 outlier 必須拉高 envelope。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import pytest

from safety_sim.experiments.r1_drift import (
    EndpointMeasurement,
    EndpointError,
    analyze_r1_directory,
    compute_endpoint_error,
    fit_observed_envelope,
    summarize_by_speed,
)


@dataclass(frozen=True)
class _Ep:
    physical: tuple[float, float]
    fused: tuple[float, float]
    intended: tuple[float, float]
    blind_m: float
    speed_mps: float
    yaw_error_rad: float | None = None


def _err(**kw) -> EndpointError:
    return compute_endpoint_error(EndpointMeasurement(**kw))


# --- 8.4 誤差分解 ---

def test_localization_error_is_physical_minus_fused():
    e = _err(physical=(1.0, 0.0), fused=(0.9, 0.1), intended=(1.0, 0.0),
             blind_m=1.0, speed_mps=0.15)
    assert math.isclose(e.localization_error_m, math.hypot(0.1, -0.1), abs_tol=1e-12)
    assert math.isclose(e.path_control_error_m, 0.0, abs_tol=1e-12)


def test_path_control_error_is_physical_minus_intended():
    e = _err(physical=(1.1, 0.2), fused=(1.05, 0.0), intended=(1.0, 0.0),
             blind_m=1.0, speed_mps=0.15)
    assert math.isclose(e.path_control_error_m, math.hypot(0.1, 0.2), abs_tol=1e-12)


def test_along_cross_radial_components():
    # 沿 +x 前進；physical 在 (1.0, 0.06)、fused 在 (0.95, 0.0)。
    # 誤差向量 (0.05, 0.06)：沿軌 0.05、橫向 0.06、徑向 hypot。
    e = _err(physical=(1.0, 0.06), fused=(0.95, 0.0), intended=(1.0, 0.0),
             blind_m=1.0, speed_mps=0.15, track_dir=(1.0, 0.0))
    assert math.isclose(e.along_track_m, 0.05, abs_tol=1e-12)
    assert math.isclose(e.cross_track_m, 0.06, abs_tol=1e-12)
    assert math.isclose(e.localization_error_m, math.hypot(0.05, 0.06), abs_tol=1e-12)


def test_requires_finite_endpoints():
    with pytest.raises(ValueError):
        _err(physical=(math.nan, 0.0), fused=(0.0, 0.0), intended=(0.0, 0.0),
             blind_m=1.0, speed_mps=0.15)


def test_requires_speed_and_baseline_fields():
    with pytest.raises(ValueError):
        _err(physical=(1.0, 0.0), fused=(0.9, 0.0), intended=(1.0, 0.0),
             blind_m=1.0, speed_mps=None)


# --- 8.5 envelope ---

def test_b_is_zero_max_plus_resolution_rounded_up_to_cm():
    zero = [
        EndpointMeasurement(physical=(0.003, 0.004), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0),
        EndpointMeasurement(physical=(0.0, -0.002), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0),
    ]
    env = fit_observed_envelope(zero, instrument_resolution_m=0.001)
    # max zero radial = hypot(0.003,0.004)=0.005；+0.001=0.006 → ceil 0.01 = 0.01
    assert env.b_m == pytest.approx(0.01)
    # 沒有 moving samples → k = 0
    assert env.k_per_m == 0.0


def test_k_uses_max_ratio_and_rounds_up_to_0_05():
    zero = [EndpointMeasurement(physical=(0.004, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0)]
    moving = [
        EndpointMeasurement(physical=(0.11, 0.0), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=1.0, speed_mps=0.15),
        EndpointMeasurement(physical=(0.20, 0.0), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=2.0, speed_mps=0.15),
    ]
    env = fit_observed_envelope(zero + moving, instrument_resolution_m=0.0)
    # b = 0.01；ratios: (0.11-0.01)/1=0.10, (0.20-0.01)/2=0.095 → k_raw=0.10 → 0.10
    assert env.b_m == pytest.approx(0.01)
    assert env.k_per_m == pytest.approx(0.10)


def test_k_rounds_up_from_just_above_grid():
    zero = [EndpointMeasurement(physical=(0.0, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0)]
    moving = [EndpointMeasurement(physical=(0.051, 0.0), fused=(0.0, 0.0),
                                  intended=(0.0, 0.0), blind_m=1.0, speed_mps=0.15)]
    env = fit_observed_envelope(zero + moving, instrument_resolution_m=0.0)
    # k_raw = 0.051 → ceil to 0.05 grid = 0.10
    assert env.k_per_m == pytest.approx(0.10)


def test_envelope_covers_all_samples():
    zero = [EndpointMeasurement(physical=(0.004, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0)]
    moving = [
        EndpointMeasurement(physical=(0.15, 0.03), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=1.5, speed_mps=0.22),
        EndpointMeasurement(physical=(0.30, -0.05), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=3.0, speed_mps=0.15),
    ]
    env = fit_observed_envelope(zero + moving, instrument_resolution_m=0.0)
    for m in moving:
        e = compute_endpoint_error(m).localization_error_m
        assert e <= env.b_m + env.k_per_m * m.blind_m + 1e-9


def test_legal_outlier_raises_envelope():
    zero = [EndpointMeasurement(physical=(0.0, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0)]
    moving = [
        EndpointMeasurement(physical=(0.05, 0.0), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=1.0, speed_mps=0.15),
        EndpointMeasurement(physical=(0.41, 0.0), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=2.0, speed_mps=0.15),
    ]
    env = fit_observed_envelope(zero + moving, instrument_resolution_m=0.0)
    # 第二點 ratio 0.205 → ceil 至 0.05 網格 = 0.25（比只有第一點時大）
    assert env.k_per_m == pytest.approx(0.25)


def test_shared_k_is_largest_per_speed_k():
    zero = [EndpointMeasurement(physical=(0.0, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0)]
    slow = [EndpointMeasurement(physical=(0.06, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=1.0, speed_mps=0.05)]
    fast = [EndpointMeasurement(physical=(0.26, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=1.0, speed_mps=0.22)]
    env = fit_observed_envelope(zero + slow + fast, instrument_resolution_m=0.0)
    assert env.k_per_m >= env.per_speed_k["0.05"]
    assert env.k_per_m >= env.per_speed_k["0.22"]


def test_shared_k_not_average_of_per_speed():
    zero = [EndpointMeasurement(physical=(0.0, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0)]
    low = [EndpointMeasurement(physical=(0.05, 0.0), fused=(0.0, 0.0),
                               intended=(0.0, 0.0), blind_m=1.0, speed_mps=0.15)]
    high = [EndpointMeasurement(physical=(0.35, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=1.0, speed_mps=0.22)]
    env = fit_observed_envelope(zero + low + high, instrument_resolution_m=0.0)
    avg = (env.per_speed_k["0.15"] + env.per_speed_k["0.22"]) / 2
    assert env.k_per_m > avg


def test_path_control_error_never_sizes_envelope():
    # physical−intended 很大，但 physical−fused 為零：k 必須為 0。
    zero = [EndpointMeasurement(physical=(0.0, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0)]
    moving = [EndpointMeasurement(physical=(0.40, 0.0), fused=(0.40, 0.0),
                                  intended=(0.0, 0.0), blind_m=2.0, speed_mps=0.15)]
    env = fit_observed_envelope(zero + moving, instrument_resolution_m=0.0)
    assert env.k_per_m == pytest.approx(0.0)


def test_r3_appendix_samples_never_enter_b_or_k():
    zero = [EndpointMeasurement(physical=(0.0, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.0)]
    moving = [EndpointMeasurement(physical=(0.10, 0.0), fused=(0.0, 0.0),
                                  intended=(0.0, 0.0), blind_m=1.0, speed_mps=0.15)]
    appendix = [
        EndpointMeasurement(physical=(0.90, 0.0), fused=(0.0, 0.0),
                            intended=(0.0, 0.0), blind_m=0.0, speed_mps=0.15),
    ]
    env = fit_observed_envelope(zero + moving, instrument_resolution_m=0.0,
                                appendix_continuous_visual=appendix)
    ref = fit_observed_envelope(zero + moving, instrument_resolution_m=0.0)
    assert env.b_m == ref.b_m
    assert env.k_per_m == ref.k_per_m


def test_analyze_r1_directory_outputs_risk_artifact(tmp_path: Path):
    # 寫一個最小 measurements.csv 風格目錄，驗證 analyze 產出 risk.json + 圖。
    import csv
    rows = [
        {"run_id": "R1_000_x", "speed_mps": "0.0", "blind_m": "0.0",
         "physical_x": "0.004", "physical_y": "0.0",
         "fused_x": "0.0", "fused_y": "0.0",
         "intended_x": "0.0", "intended_y": "0.0",
         "baseline_length_m": "1.0", "baseline_residual_m": "0.002",
         "payload_kg": "1.0", "floor_material": "concrete"},
        {"run_id": "R1_001_x", "speed_mps": "0.15", "blind_m": "1.0",
         "physical_x": "1.10", "physical_y": "0.02",
         "fused_x": "1.00", "fused_y": "0.0",
         "intended_x": "1.0", "intended_y": "0.0",
         "baseline_length_m": "1.0", "baseline_residual_m": "0.002",
         "payload_kg": "1.0", "floor_material": "concrete"},
    ]
    header = ",".join(rows[0].keys())
    body = "\n".join(",".join(str(v) for v in row.values()) for row in rows)
    (tmp_path / "measurements.csv").write_text(f"{header}\n{body}\n",
                                               encoding="utf-8")
    out = analyze_r1_directory(tmp_path, instrument_resolution_m=0.0)
    assert (out / "risk.json").exists()
    risk = json.loads((out / "risk.json").read_text(encoding="utf-8"))
    assert "b_m" in risk and "k_per_m" in risk
    assert (out / "envelope_all_speeds.png").exists()
    assert (out / "envelope_by_speed.png").exists()


def test_summarize_by_speed_groups_exactly():
    m1 = EndpointMeasurement(physical=(1.0, 0.0), fused=(0.9, 0.0),
                             intended=(1.0, 0.0), blind_m=1.0, speed_mps=0.15)
    m2 = EndpointMeasurement(physical=(2.0, 0.0), fused=(1.8, 0.0),
                             intended=(2.0, 0.0), blind_m=1.0, speed_mps=0.22)
    s = summarize_by_speed([m1, m2])
    assert set(s) == {"0.05", "0.15", "0.22"}
    assert s["0.15"]["count"] == 1
    assert s["0.22"]["count"] == 1
    assert s["0.05"]["count"] == 0


def test_r1_cli_schedule_command(tmp_path: Path):
    """CLI schedule 子命令可經 main(argv) 呼叫（smoke 發現的 argv bug）。"""
    from safety_sim.experiments import r1_drift

    out = tmp_path / "sched.json"
    rc = r1_drift.main(["schedule", "--out", str(out), "--seed", "1"])
    assert rc == 0
    assert out.exists()
    assert len(json.loads(out.read_text(encoding="utf-8"))["runs"]) == 90


def test_near_zero_blind_float_counts_as_zero_cell():
    """Gazebo odom 積分會給 0m cell 微小非零值（~1e-19）：必須視為 zero。"""
    zero = [EndpointMeasurement(physical=(0.004, 0.0), fused=(0.0, 0.0),
                                intended=(0.0, 0.0),
                                blind_m=-5.983e-20, speed_mps=0.0)]
    moving = [EndpointMeasurement(physical=(0.11, 0.0), fused=(0.0, 0.0),
                                  intended=(0.0, 0.0),
                                  blind_m=1.0, speed_mps=0.15)]
    env = fit_observed_envelope(zero + moving, instrument_resolution_m=0.0)
    assert env.n_zero == 1
    assert env.n_moving == 1
