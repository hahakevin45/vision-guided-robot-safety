"""R1 盲走漂移分析器（spec 8.4-8.6、15）。

- 定位誤差 = physical − fused；路徑誤差 = physical − intended（僅診斷）。
- b = ceil_0.01(max 0m radial + instrument resolution)。
- k_raw = max_i max(0, e_i − b) / s_i（全部 moving cells、所有速度）。
- k = ceil_0.05(k_raw)。
- per-speed k 與 shared k 並列；shared 取較大者，禁止平均掉較大誤差。
- R3 continuous_visual 附錄永不進 b/k。
- 產出 risk.json 與兩張圖（spec 8.6）。
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from ..experiments.physical_contract import PLATFORM_CEILING_MPS


def _finite(value: float, name: str) -> float:
    if value is None or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return float(value)


@dataclass(frozen=True)
class EndpointMeasurement:
    """一趟 R1 的量測（spec 6.4/8.4）。真值座標皆為驅動軸心中點。"""

    physical: tuple[float, float]       # 捲尺/雷射真值終點
    fused: tuple[float, float]          # /pose_fused 終點
    intended: tuple[float, float]       # 理想基線終點
    blind_m: float                      # 實際 odom 盲走里程
    speed_mps: float                    # commanded speed（0 表示 0m cell）
    track_dir: tuple[float, float] = (1.0, 0.0)  # 理想行進方向單位向量
    yaw_error_rad: float | None = None
    # 量測 metadata（spec 6.4/6.5）
    run_id: str | None = None
    baseline_length_m: float | None = None
    baseline_residual_m: float | None = None
    payload_kg: float | None = None
    floor_material: str | None = None


@dataclass(frozen=True)
class EndpointError:
    localization_error_m: float   # physical − fused（進 b/k）
    path_control_error_m: float   # physical − intended（僅診斷）
    along_track_m: float
    cross_track_m: float
    yaw_error_rad: float | None


def compute_endpoint_error(m: EndpointMeasurement) -> EndpointError:
    px, py = _finite(m.physical[0], "physical.x"), _finite(m.physical[1], "physical.y")
    fx, fy = _finite(m.fused[0], "fused.x"), _finite(m.fused[1], "fused.y")
    ix, iy = _finite(m.intended[0], "intended.x"), _finite(m.intended[1], "intended.y")
    if m.speed_mps is None or m.blind_m is None:
        raise ValueError("speed_mps and blind_m are required")
    speed = _finite(m.speed_mps, "speed_mps")
    _finite(m.blind_m, "blind_m")

    dx, dy = px - fx, py - fy
    loc = math.hypot(dx, dy)
    path = math.hypot(px - ix, py - iy)

    tx, ty = m.track_dir
    norm = math.hypot(tx, ty)
    if norm <= 0.0:
        raise ValueError("track_dir must be non-zero")
    tx, ty = tx / norm, ty / norm
    along = dx * tx + dy * ty
    cross = math.hypot(dx, dy - along * 0)  # 保留正負語意需拆解
    # cross 用外積符號：cross = (dx,dy) 在垂直方向的分量（含正負）
    cross_signed = dx * (-ty) + dy * tx
    return EndpointError(
        localization_error_m=loc,
        path_control_error_m=path,
        along_track_m=along,
        cross_track_m=abs(cross_signed),
        yaw_error_rad=m.yaw_error_rad,
    )


def _ceil_grid(value: float, grid: float) -> float:
    if value <= 0.0:
        return 0.0
    return math.ceil(value / grid) * grid


@dataclass(frozen=True)
class EnvelopeResult:
    b_m: float
    k_per_m: float
    k_raw: float
    n_zero: int
    n_moving: int
    per_speed_k: dict[str, float]
    certified_max_speed_mps: float
    ceiling_mps: float


def summarize_by_speed(samples: list[EndpointMeasurement]) -> dict[str, dict]:
    """依 commanded speed 分群（spec 8.5-4）；key 為 '%.2f'。"""
    groups: dict[str, list[EndpointMeasurement]] = {}
    for m in samples:
        key = f"{float(m.speed_mps):.2f}"
        groups.setdefault(key, []).append(m)
    out: dict[str, dict] = {}
    for speed in (0.05, 0.15, 0.22):
        key = f"{speed:.2f}"
        group = groups.get(key, [])
        out[key] = {"count": len(group)}
        if group:
            e = [compute_endpoint_error(m).localization_error_m for m in group]
            out[key]["median_m"] = float(sorted(e)[len(e) // 2])
            out[key]["max_m"] = float(max(e))
    return out


def fit_observed_envelope(
    samples: list[EndpointMeasurement],
    *,
    instrument_resolution_m: float,
    appendix_continuous_visual: list[EndpointMeasurement] | None = None,
) -> EnvelopeResult:
    """spec 8.5。`appendix_continuous_visual`（R3 附錄）永不進入 b/k。"""
    res = _finite(instrument_resolution_m, "instrument_resolution_m")
    zero = [m for m in samples if abs(float(m.blind_m)) <= 1e-9]
    moving = [m for m in samples if float(m.blind_m) > 1e-9]
    if not zero:
        raise ValueError("R1 requires zero-distance calibration samples")

    zero_radial = [compute_endpoint_error(m).localization_error_m for m in zero]
    b = _ceil_grid(max(zero_radial) + res, 0.01)

    ratios: list[float] = []
    for m in moving:
        e = compute_endpoint_error(m).localization_error_m
        ratios.append(max(0.0, e - b) / float(m.blind_m))
    k_raw = max(ratios) if ratios else 0.0
    k = _ceil_grid(k_raw, 0.05)

    per_speed: dict[str, float] = {}
    for speed in (0.05, 0.15, 0.22):
        group = [m for m in moving if abs(float(m.speed_mps) - speed) < 1e-9]
        gr: list[float] = []
        for m in group:
            e = compute_endpoint_error(m).localization_error_m
            gr.append(max(0.0, e - b) / float(m.blind_m))
        per_speed[f"{speed:.2f}"] = _ceil_grid(max(gr), 0.05) if gr else 0.0

    return EnvelopeResult(
        b_m=b, k_per_m=k, k_raw=k_raw,
        n_zero=len(zero), n_moving=len(moving),
        per_speed_k=per_speed,
        certified_max_speed_mps=0.22,
        ceiling_mps=PLATFORM_CEILING_MPS,
    )


_CSV_COLUMNS = (
    "run_id", "speed_mps", "blind_m",
    "physical_x", "physical_y", "fused_x", "fused_y",
    "intended_x", "intended_y",
    "baseline_length_m", "baseline_residual_m",
    "payload_kg", "floor_material",
)


def _parse_csv(path: Path) -> list[EndpointMeasurement]:
    with path.open(encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            raise ValueError(f"empty CSV: {path}")
        missing = [c for c in _CSV_COLUMNS if c not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV missing columns: {missing}")
        out: list[EndpointMeasurement] = []
        for row in reader:
            if not any(v.strip() for v in row.values()):
                continue
            out.append(EndpointMeasurement(
                physical=(float(row["physical_x"]), float(row["physical_y"])),
                fused=(float(row["fused_x"]), float(row["fused_y"])),
                intended=(float(row["intended_x"]), float(row["intended_y"])),
                blind_m=float(row["blind_m"]),
                speed_mps=float(row["speed_mps"]),
                run_id=row.get("run_id") or None,
                baseline_length_m=float(row["baseline_length_m"]) if row.get("baseline_length_m") else None,
                baseline_residual_m=float(row["baseline_residual_m"]) if row.get("baseline_residual_m") else None,
                payload_kg=float(row["payload_kg"]) if row.get("payload_kg") else None,
                floor_material=row.get("floor_material") or None,
            ))
    return out


def _plot_envelope(samples: list[EndpointMeasurement], env: EnvelopeResult,
                   path: Path, *, by_speed: bool) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if by_speed:
        fig, ax = plt.subplots(figsize=(7, 4))
        colors = {0.05: "tab:green", 0.15: "tab:blue", 0.22: "tab:red"}
        for m in samples:
            e = compute_endpoint_error(m).localization_error_m
            ax.scatter(m.blind_m, e, color=colors.get(float(m.speed_mps), "gray"),
                       s=22, label=f"{m.speed_mps:.2f} m/s" if m.speed_mps > 0 else "0 m")
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), fontsize=8)
        title = "radial localization error by commanded speed"
    else:
        fig, ax = plt.subplots(figsize=(7, 4))
        for m in samples:
            e = compute_endpoint_error(m).localization_error_m
            ax.scatter(m.blind_m, e, s=22, color="tab:blue")
        title = "radial localization error vs blind distance"
    s = sorted(m.blind_m for m in samples)
    if s:
        xs = [0.0, max(s)]
        ax.plot(xs, [env.b_m + env.k_per_m * x for x in xs],
                color="black", ls="--", lw=1.5,
                label=f"b + k·s (b={env.b_m:.2f}, k={env.k_per_m:.2f})")
        ax.legend(fontsize=8)
    ax.set_xlabel("blind distance [m]")
    ax.set_ylabel("radial localization error [m]")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)


def analyze_r1_directory(data_dir: Path, *, instrument_resolution_m: float,
                         appendix_csv: Path | None = None) -> Path:
    """讀 measurements.csv → envelope → 寫 risk.json + 兩張圖（spec 8.6/15）。"""
    samples = _parse_csv(data_dir / "measurements.csv")
    appendix: list[EndpointMeasurement] = []
    if appendix_csv is not None and appendix_csv.exists():
        appendix = _parse_csv(appendix_csv)
    env = fit_observed_envelope(
        samples, instrument_resolution_m=instrument_resolution_m,
        appendix_continuous_visual=appendix,
    )
    per_speed_summary = summarize_by_speed(samples)

    risk = {
        "b_m": env.b_m,
        "k_per_m": env.k_per_m,
        "k_raw": env.k_raw,
        "n_zero": env.n_zero,
        "n_moving": env.n_moving,
        "per_speed_k": env.per_speed_k,
        "per_speed_summary": per_speed_summary,
        "certified_max_speed_mps": env.certified_max_speed_mps,
        "platform_ceiling_mps": env.ceiling_mps,
        "certified_note": (
            "observed conservative envelope, not a population-level 99.9% bound; "
            "certified only up to 0.22 m/s on this drivetrain ceiling"
        ),
    }
    (data_dir / "risk.json").write_text(
        json.dumps(risk, ensure_ascii=False, indent=2), encoding="utf-8")
    _plot_envelope(samples, env, data_dir / "envelope_all_speeds.png", by_speed=False)
    _plot_envelope(samples, env, data_dir / "envelope_by_speed.png", by_speed=True)
    return data_dir


def main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    from .physical_contract import build_r1_schedule, load_schedule, write_schedule

    parser = argparse.ArgumentParser(prog="safety_sim.experiments.r1_drift")
    sub = parser.add_subparsers(dest="command", required=True)

    p_sched = sub.add_parser("schedule", help="產生 R1 90-run 排程")
    p_sched.add_argument("--out", type=Path, required=True)
    p_sched.add_argument("--seed", type=int, default=1)
    p_sched.add_argument("--runout-max-m", type=float, default=3.0)

    p_val = sub.add_parser("validate", help="驗證 schedule/manifest")
    p_val.add_argument("path", type=Path)

    p_ana = sub.add_parser("analyze", help="分析 R1 量測目錄")
    p_ana.add_argument("data_dir", type=Path)
    p_ana.add_argument("--resolution-m", type=float, default=0.002)
    p_ana.add_argument("--appendix-csv", type=Path, default=None)

    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    if args.command == "schedule":
        runs, dropped = build_r1_schedule(args.seed, runout_max_m=args.runout_max_m)
        write_schedule(args.out, seed=args.seed, runs=runs,
                       meta={"dropped_cells": dropped})
        print(f"wrote {len(runs)} R1 runs to {args.out}")
        if dropped:
            print(f"dropped cells: {dropped}")
    elif args.command == "validate":
        seed, runs = load_schedule(args.path)
        print(f"schedule seed={seed} count={len(runs)}")
    else:
        out = analyze_r1_directory(
            args.data_dir, instrument_resolution_m=args.resolution_m,
            appendix_csv=args.appendix_csv)
        print(f"wrote risk.json + plots to {out}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
