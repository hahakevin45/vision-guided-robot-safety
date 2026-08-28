import json
import subprocess
import sys

import cv2
import numpy as np


def test_camera_slice_capture_writes_frames_sheet_and_report(tmp_path):
    video = tmp_path / "synthetic.mp4"
    writer = cv2.VideoWriter(
        str(video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        5.0,
        (160, 90),
    )
    for index in range(20):
        frame = np.full((90, 160, 3), index * 10, dtype=np.uint8)
        cv2.putText(frame, str(index), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        writer.write(frame)
    writer.release()

    out_dir = tmp_path / "capture"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.capture_camera_slices",
            "--video",
            str(video),
            "--duration-s",
            "3",
            "--sample-interval-s",
            "1",
            "--output-dir",
            str(out_dir),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    report = json.loads((out_dir / "capture_report.json").read_text(encoding="utf-8"))
    assert report["pass"] is True
    assert report["frames_saved_count"] == 3
    assert (out_dir / "contact_sheet.jpg").exists()
    for item in report["frames_saved"]:
        assert (out_dir / item["relative_path"]).exists()
