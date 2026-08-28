from __future__ import annotations

import argparse
import json
from pathlib import Path

from vgr_core.model import CommandConfig
from vgr_driver.pipeline import Phase1Pipeline


def main() -> int:
    """Phase 1 手動 demo 入口。

    可選擇預錄影片或 live camera，最後印出 diagnostics summary，
    也可以額外輸出 JSON report / debug video。
    """

    parser = argparse.ArgumentParser(description="Run Phase 1 vision-to-mock-MCU demo.")
    source = parser.add_mutually_exclusive_group(required=True)
    # 影片和攝影機是互斥輸入來源；同一次 demo 只跑其中一種。
    source.add_argument("--video", type=Path)
    source.add_argument("--camera-index", type=int)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--debug-video", type=Path, default=None)
    parser.add_argument("--camera-width", type=int, default=None)
    parser.add_argument("--camera-height", type=int, default=None)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--min-confidence", type=float, default=CommandConfig.min_confidence)
    parser.add_argument("--target-lost-timeout-s", type=float, default=CommandConfig.target_lost_timeout_s)
    args = parser.parse_args()

    # CLI 只開放少量常用參數，避免 demo 時需要改程式才能調整安全門檻。
    config = CommandConfig(
        min_confidence=args.min_confidence,
        target_lost_timeout_s=args.target_lost_timeout_s,
    )
    pipeline = Phase1Pipeline(config=config)
    if args.video is not None:
        # 預錄影片路徑可重現，適合 regression test 和產生面試展示素材。
        diagnostics = pipeline.run_video(
            video_path=args.video,
            max_frames=args.max_frames,
            debug_video_path=args.debug_video,
        )
    else:
        # live camera 適合現場展示；max_frames 預設 150，避免 demo 無限執行。
        diagnostics = pipeline.run_camera(
            camera_index=args.camera_index,
            max_frames=args.max_frames or 150,
            debug_video_path=args.debug_video,
            width=args.camera_width,
            height=args.camera_height,
            fps=args.camera_fps,
        )
    summary = diagnostics.summary()
    print(json.dumps(summary, indent=2))
    if args.report is not None:
        # report 保存逐幀事件與統計值，方便事後分析而不是只看終端輸出。
        diagnostics.write(args.report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
