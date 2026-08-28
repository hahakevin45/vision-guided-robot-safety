from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture short camera/video slices for remote visual inspection."
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--camera-index", type=int, default=None)
    source.add_argument("--video", type=Path, default=None)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--sample-interval-s", type=float, default=5.0)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--thumb-width", type=int, default=320)
    parser.add_argument("--thumb-height", type=int, default=180)
    args = parser.parse_args()

    camera_index = 0 if args.camera_index is None and args.video is None else args.camera_index
    if args.output_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = Path(f"outputs/camera_remote_capture_{stamp}")

    result = capture_slices(
        camera_index=camera_index,
        video_path=args.video,
        duration_s=args.duration_s,
        sample_interval_s=args.sample_interval_s,
        output_dir=args.output_dir,
        thumb_size=(args.thumb_width, args.thumb_height),
    )

    print(json.dumps(result, indent=2))
    print("CAMERA SLICE CAPTURE: PASS" if result["pass"] else "CAMERA SLICE CAPTURE: FAIL")
    return 0 if result["pass"] else 1


def capture_slices(
    camera_index: int | None,
    video_path: Path | None,
    duration_s: float,
    sample_interval_s: float,
    output_dir: Path,
    thumb_size: tuple[int, int] = (320, 180),
) -> dict[str, object]:
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if sample_interval_s <= 0:
        raise ValueError("sample_interval_s must be positive")

    frames_dir = output_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    source = str(video_path) if video_path is not None else int(camera_index or 0)
    cap = cv2.VideoCapture(source)

    result: dict[str, object] = {
        "pass": False,
        "source": str(source),
        "duration_s": duration_s,
        "sample_interval_s": sample_interval_s,
        "frames_saved": [],
        "frames_saved_count": 0,
        "read_frames": 0,
        "error": None,
    }

    try:
        if not cap.isOpened():
            raise RuntimeError(f"could not open source: {source}")

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
        result.update({"width": width, "height": height, "reported_fps": fps})

        saved: list[dict[str, object]] = []
        if video_path is not None:
            read_frames = _capture_video_file(cap, duration_s, sample_interval_s, frames_dir, saved)
        else:
            read_frames = _capture_live_camera(cap, duration_s, sample_interval_s, frames_dir, saved)

        result["read_frames"] = read_frames
        result["frames_saved"] = saved
        result["frames_saved_count"] = len(saved)
        if saved:
            sheet_path = output_dir / "contact_sheet.jpg"
            _write_contact_sheet(output_dir, saved, sheet_path, thumb_size)
            result["contact_sheet"] = str(sheet_path)
        result["pass"] = len(saved) > 0 and result["error"] is None
    except Exception as exc:  # noqa: BLE001 - CLI must preserve capture failure details.
        result["error"] = str(exc)
    finally:
        cap.release()

    report_path = output_dir / "capture_report.json"
    report_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def _capture_live_camera(
    cap,
    duration_s: float,
    sample_interval_s: float,
    frames_dir: Path,
    saved: list[dict[str, object]],
) -> int:
    start = time.monotonic()
    next_sample = 0.0
    read_frames = 0
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration_s:
            break
        ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"failed to read frame at {elapsed:.3f}s")
        read_frames += 1
        if elapsed >= next_sample:
            _save_frame(frames_dir, saved, frame, elapsed)
            next_sample += sample_interval_s
    return read_frames


def _capture_video_file(
    cap,
    duration_s: float,
    sample_interval_s: float,
    frames_dir: Path,
    saved: list[dict[str, object]],
) -> int:
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 30.0)
    if fps <= 0:
        fps = 30.0
    next_sample = 0.0
    read_frames = 0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        elapsed = read_frames / fps
        if elapsed >= duration_s:
            break
        read_frames += 1
        if elapsed + 1e-9 >= next_sample:
            _save_frame(frames_dir, saved, frame, elapsed)
            next_sample += sample_interval_s
    return read_frames


def _save_frame(
    frames_dir: Path,
    saved: list[dict[str, object]],
    frame,
    elapsed_s: float,
) -> None:
    filename = frames_dir / f"frame_{len(saved):02d}_{elapsed_s:05.1f}s.jpg"
    cv2.imwrite(str(filename), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    saved.append(
        {
            "relative_path": str(filename.relative_to(frames_dir.parent)),
            "path": str(filename),
            "elapsed_s": round(elapsed_s, 3),
        }
    )


def _write_contact_sheet(
    output_dir: Path,
    frames: list[dict[str, object]],
    sheet_path: Path,
    thumb_size: tuple[int, int],
) -> None:
    thumb_w, thumb_h = thumb_size
    thumbs = []
    for item in frames:
        img = cv2.imread(str(output_dir / str(item["relative_path"])))
        if img is None:
            continue
        thumb = cv2.resize(img, (thumb_w, thumb_h), interpolation=cv2.INTER_AREA)
        cv2.putText(
            thumb,
            f"{float(item['elapsed_s']):.1f}s",
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        thumbs.append(thumb)

    if not thumbs:
        raise RuntimeError("no frames available for contact sheet")

    cols = min(3, len(thumbs))
    rows = (len(thumbs) + cols - 1) // cols
    sheet = np.full((rows * thumb_h, cols * thumb_w, 3), 255, dtype=np.uint8)
    for idx, thumb in enumerate(thumbs):
        row, col = divmod(idx, cols)
        y0 = row * thumb_h
        x0 = col * thumb_w
        sheet[y0 : y0 + thumb_h, x0 : x0 + thumb_w] = thumb
    cv2.imwrite(str(sheet_path), sheet, [int(cv2.IMWRITE_JPEG_QUALITY), 90])


if __name__ == "__main__":
    raise SystemExit(main())
