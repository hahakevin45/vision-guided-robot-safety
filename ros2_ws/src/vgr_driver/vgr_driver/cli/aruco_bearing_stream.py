"""即時偵測 ArUco 標籤，輸出相對車頭的距離、方位與轉向建議。

本工具給 Phase 2 車上測試使用：開啟 USB 相機，套用相機內參估算 marker
相對相機的位置，並把 +x=右、+z=前 的座標轉成車頭方位判斷。

用法:
    python3 -m vgr_driver.cli.aruco_bearing_stream

相機硬體預設 180° 倒放；每一幀偵測前會先呼叫 vgr_driver.vision.camera_orientation.upright()
轉正。若硬體改成正放，加 --no-rotate-180 關掉。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
from cv2 import aruco
import numpy as np

from vgr_driver.vision.camera_orientation import CAMERA_ROTATE_180, upright


DEFAULT_INTRINSICS = Path("ChArUco/camera_intrinsics_640x480.json")
DEFAULT_DICT = "DICT_6X6_250"
DEFAULT_MARKER_LEN = 0.17
DEFAULT_CAMERA = 0


def bearing_and_turn(x: float, z: float, deadband_deg: float = 5.0) -> tuple[float, str, str]:
    """把 marker 的相機座標轉成方位角、左右側與轉向建議。"""
    bearing_deg = math.degrees(math.atan2(x, z))
    if abs(bearing_deg) <= deadband_deg:
        return bearing_deg, "正前方", "FORWARD"
    if bearing_deg < 0.0:
        return bearing_deg, "左", "TURN_LEFT"
    return bearing_deg, "右", "TURN_RIGHT"


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, dict[str, int] | None]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    image_size = data.get("image_size")
    return camera_matrix, dist_coeffs, image_size


def make_detector_params() -> Any:
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()


def detect_markers(
    gray: np.ndarray,
    aruco_dict: Any,
    params: Any,
) -> tuple[Any, np.ndarray | None]:
    if hasattr(aruco, "ArucoDetector"):
        corners, ids, _ = aruco.ArucoDetector(aruco_dict, params).detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    return corners, ids


def estimate_pose(
    marker_corners: np.ndarray,
    marker_len: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """回傳 (rvec, tvec)。新版 OpenCV 沒有 estimatePoseSingleMarkers 時用 solvePnP。"""
    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            [marker_corners],
            marker_len,
            camera_matrix,
            dist_coeffs,
        )
        return rvecs[0][0], tvecs[0][0]

    half = marker_len / 2.0
    obj_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    ok, rvec, tvec = cv2.solvePnP(
        obj_points,
        marker_corners.reshape(-1, 2),
        camera_matrix,
        dist_coeffs,
    )
    if not ok:
        raise RuntimeError("solvePnP failed")
    return rvec.ravel(), tvec.ravel()


def _capture_size(image_size: dict[str, int] | None) -> tuple[int, int]:
    if not image_size:
        return 640, 480
    return int(image_size.get("width", 640)), int(image_size.get("height", 480))


def _print_marker(marker_id: int, tvec: np.ndarray, deadband_deg: float) -> None:
    x, _, z = [float(v) for v in tvec]
    distance_m = float(np.linalg.norm(tvec))
    bearing_deg, side, turn = bearing_and_turn(x, z, deadband_deg=deadband_deg)
    print(
        f"id={marker_id:<3d} distance={distance_m:.3f} m "
        f"bearing={bearing_deg:+.1f} deg side={side} turn={turn}"
    )


def stream_markers(args: argparse.Namespace) -> int:
    # 即時工具：確保每一行馬上輸出，導到 log/pipe 時不會被 block-buffer 卡住。
    try:
        import sys

        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass

    camera_matrix, dist_coeffs, image_size = load_intrinsics(args.intrinsics)
    width, height = _capture_size(image_size)
    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, args.dict))
    params = make_detector_params()

    print(
        f"intrinsics={args.intrinsics} image_size={width}x{height} "
        f"dict={args.dict} marker_len={args.marker_len:.3f}m "
        f"rotate_180={'on' if args.rotate_180 else 'off'}"
    )

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(f"打不開相機 index {args.camera}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("讀取影格失敗")
                return 1

            frame = upright(frame, args.rotate_180)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = detect_markers(gray, aruco_dict, params)
            if ids is None:
                continue

            for marker_corners, marker_id in zip(corners, ids.ravel()):
                _, tvec = estimate_pose(marker_corners, args.marker_len, camera_matrix, dist_coeffs)
                _print_marker(int(marker_id), tvec, args.deadband_deg)
    except KeyboardInterrupt:
        print("\n結束")
        return 0
    finally:
        cap.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream ArUco marker distance, bearing, side, and turn suggestion from a USB camera."
    )
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA, help="USB camera index")
    parser.add_argument("--intrinsics", type=Path, default=DEFAULT_INTRINSICS)
    parser.add_argument("--marker-len", type=float, default=DEFAULT_MARKER_LEN, help="marker 邊長 (m)")
    parser.add_argument("--dict", default=DEFAULT_DICT, help="ArUco dictionary name")
    parser.add_argument("--deadband-deg", type=float, default=5.0, help="FORWARD deadband in degrees")
    parser.add_argument(
        "--rotate-180",
        dest="rotate_180",
        action="store_true",
        default=CAMERA_ROTATE_180,
        help="相機 180° 倒放時偵測前轉正",
    )
    parser.add_argument(
        "--no-rotate-180",
        dest="rotate_180",
        action="store_false",
        help="相機正放時關閉 180° 轉正",
    )
    return parser.parse_args()


def main() -> int:
    return stream_markers(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
