from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


DEFAULT_RESOLUTIONS = [(640, 480), (1280, 720), (1920, 1080)]


def main() -> int:
    parser = argparse.ArgumentParser(description="Profile USB camera resolution throughput.")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--frames", type=int, default=180)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--min-fps", type=float, default=20.0)
    parser.add_argument(
        "--resolution",
        action="append",
        default=None,
        help="Resolution as WIDTHxHEIGHT. Can be repeated.",
    )
    parser.add_argument("--report", type=Path, default=Path("outputs/camera_resolution_profile.json"))
    parser.add_argument("--sample-dir", type=Path, default=Path("outputs/camera_resolution_samples"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    resolutions = parse_resolutions(args.resolution)
    if args.dry_run:
        results = [dry_run_result(width, height, args.frames, args.fps) for width, height in resolutions]
    else:
        args.sample_dir.mkdir(parents=True, exist_ok=True)
        results = [
            profile_resolution(
                camera_index=args.camera_index,
                width=width,
                height=height,
                requested_fps=args.fps,
                frames_requested=args.frames,
                sample_dir=args.sample_dir,
                min_fps=args.min_fps,
            )
            for width, height in resolutions
        ]

    report = {
        "pass": all(item["pass"] for item in results),
        "camera_index": args.camera_index,
        "min_fps": args.min_fps,
        "dry_run": args.dry_run,
        "results": results,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("CAMERA RESOLUTION PROFILE: PASS" if report["pass"] else "CAMERA RESOLUTION PROFILE: FAIL")
    return 0 if report["pass"] else 1


def parse_resolutions(values: list[str] | None) -> list[tuple[int, int]]:
    if not values:
        return DEFAULT_RESOLUTIONS
    parsed = []
    for value in values:
        try:
            width_text, height_text = value.lower().split("x", 1)
            parsed.append((int(width_text), int(height_text)))
        except ValueError as exc:
            raise ValueError(f"bad resolution: {value}") from exc
    return parsed


def dry_run_result(width: int, height: int, frames_requested: int, fps: float) -> dict:
    result = {
        "resolution_requested": [width, height],
        "fps_requested": fps,
        "opened": True,
        "resolution_actual": [width, height],
        "fps_setting_actual": fps,
        "frames_requested": frames_requested,
        "frames_read": frames_requested,
        "fps_effective": fps,
        "avg_read_latency_ms": 1000.0 / fps,
        "p95_read_latency_ms": 1000.0 / fps,
        "mean_brightness_min": 1.0,
        "mean_brightness_max": 1.0,
        "std_min": 10.0,
        "std_max": 10.0,
        "sample_path": None,
        "error": None,
    }
    result["pass"] = evaluate_profile_result(result, min_fps=20.0)
    return result


def profile_resolution(
    *,
    camera_index: int,
    width: int,
    height: int,
    requested_fps: float,
    frames_requested: int,
    sample_dir: Path,
    min_fps: float,
) -> dict:
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, requested_fps)

    opened = cap.isOpened()
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps_setting = float(cap.get(cv2.CAP_PROP_FPS))
    frames = 0
    means: list[float] = []
    stds: list[float] = []
    latencies: list[float] = []
    error = None
    sample_path = sample_dir / f"sample_{width}x{height}.jpg"
    start = time.monotonic()
    try:
        for index in range(frames_requested):
            before = time.monotonic()
            ok, frame = cap.read()
            after = time.monotonic()
            if not ok or frame is None:
                error = f"read failed at frame {index}"
                break
            frames += 1
            latencies.append((after - before) * 1000.0)
            if index % 30 == 0:
                means.append(float(np.mean(frame)))
                stds.append(float(np.std(frame)))
            if index == min(30, frames_requested - 1):
                cv2.imwrite(str(sample_path), frame)
    finally:
        cap.release()

    elapsed = max(time.monotonic() - start, 1e-9)
    sorted_latencies = sorted(latencies)
    p95_index = max(0, min(len(sorted_latencies) - 1, int(len(sorted_latencies) * 0.95) - 1))
    result = {
        "resolution_requested": [width, height],
        "fps_requested": requested_fps,
        "opened": opened,
        "resolution_actual": [actual_width, actual_height],
        "fps_setting_actual": actual_fps_setting,
        "frames_requested": frames_requested,
        "frames_read": frames,
        "fps_effective": frames / elapsed,
        "avg_read_latency_ms": sum(latencies) / len(latencies) if latencies else None,
        "p95_read_latency_ms": sorted_latencies[p95_index] if sorted_latencies else None,
        "mean_brightness_min": min(means) if means else None,
        "mean_brightness_max": max(means) if means else None,
        "std_min": min(stds) if stds else None,
        "std_max": max(stds) if stds else None,
        "sample_path": str(sample_path) if sample_path.exists() else None,
        "error": error,
    }
    result["pass"] = evaluate_profile_result(result, min_fps=min_fps)
    return result


def evaluate_profile_result(result: dict, *, min_fps: float) -> bool:
    return bool(
        result.get("opened")
        and result.get("resolution_actual") == result.get("resolution_requested")
        and result.get("frames_read") == result.get("frames_requested")
        and float(result.get("fps_effective") or 0.0) >= min_fps
        and float(result.get("std_min") or 0.0) > 5.0
        and result.get("error") is None
    )


if __name__ == "__main__":
    raise SystemExit(main())
