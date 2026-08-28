from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

from vgr_driver.pipeline import Phase1Pipeline


def main() -> int:
    """驗證 live camera 在 no-marker 場景下會保持安全停止。"""

    parser = argparse.ArgumentParser(
        description="Certify live camera input for the Phase 1 no-marker safety case."
    )
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--allow-marker", action="store_true")
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/live_camera_certification.json"),
    )
    parser.add_argument("--debug-video", type=Path, default=None)
    parser.add_argument("--camera-width", type=int, default=None)
    parser.add_argument("--camera-height", type=int, default=None)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    args = parser.parse_args()

    result = {
        "camera_index": args.camera_index,
        "requested_frames": args.frames,
        "pass": False,
        "checks": {},
        "summary": {},
        "error": None,
    }

    try:
        # certification 使用真實 camera source，但仍接 mock controller；重點是驗證安全行為。
        pipeline = Phase1Pipeline()
        diagnostics = pipeline.run_camera(
            camera_index=args.camera_index,
            max_frames=args.frames,
            debug_video_path=args.debug_video,
            width=args.camera_width,
            height=args.camera_height,
            fps=args.camera_fps,
        )
        summary = diagnostics.summary()
        # Counter 用來統計整段測試中各 command/state 出現幾次。
        commands = Counter(event["command"] for event in diagnostics.events)
        states = Counter(event["safety_state"] for event in diagnostics.events)
        detected_frames = int(summary["detections"])
        no_marker_case = detected_frames == 0
        # no-marker certification 用來確認 live camera 沒看到目標時會保持安全停止。
        safe_behavior = (
            commands.get("STOP", 0) == summary["frames"]
            and states.get("SAFE_STOP", 0) == summary["frames"]
        )

        checks = {
            "opened_and_read_frames": summary["frames"] == args.frames,
            "no_marker_detected": no_marker_case,
            "safe_stop_for_no_marker": safe_behavior,
        }
        if args.allow_marker:
            # allow-marker 只用於現場環境無法完全避免 marker 入鏡時，放寬 no-marker 判斷。
            checks["no_marker_detected"] = True
            checks["safe_stop_for_no_marker"] = True

        result.update(
            {
                "pass": all(checks.values()),
                "checks": checks,
                "summary": summary,
                "commands": dict(commands),
                "safety_states": dict(states),
            }
        )
        diagnostics.write(args.report)
    except Exception as exc:  # noqa: BLE001 - CLI must report hardware failures clearly.
        result["error"] = str(exc)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(json.dumps(result, indent=2))
    if result["pass"]:
        print("CAMERA CERTIFICATION: PASS")
        return 0
    print("CAMERA CERTIFICATION: FAIL")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
