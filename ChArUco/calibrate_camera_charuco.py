"""用 ChArUco 照片校正相機內參 (K + 畸變係數)。

用法:
    python3 calibrate_camera_charuco.py \
        --images "web_photo/*.jpg" \
        --square-len 0.0305 --marker-len 0.023 \
        --out camera_intrinsics.json

輸出 camera_intrinsics.json 內含 camera_matrix (K)、dist_coeffs、
reprojection_error、image_size，供之後 ArUco 定位 (estimatePoseSingleMarkers)
與去畸變使用。
"""
import argparse
import glob
import json

import cv2
from cv2 import aruco
import numpy as np

SQUARES_X, SQUARES_Y = 9, 6
DICT = aruco.DICT_4X4_50


def build_board(square_len, marker_len, aruco_dict):
    if hasattr(aruco, "CharucoBoard"):  # OpenCV 4.7+
        return aruco.CharucoBoard((SQUARES_X, SQUARES_Y), square_len, marker_len, aruco_dict)
    return aruco.CharucoBoard_create(SQUARES_X, SQUARES_Y, square_len, marker_len, aruco_dict)


def make_detector_params():
    if hasattr(aruco, "DetectorParameters_create"):  # 4.5/4.6
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()  # 4.7+


def detect_markers(gray, aruco_dict, params):
    if hasattr(aruco, "ArucoDetector"):  # 4.7+
        detector = aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    return corners, ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default="web_photo/*.jpg", help="glob pattern of calibration photos")
    ap.add_argument("--square-len", type=float, default=0.0305, help="量到的方格實際邊長 (m)")
    ap.add_argument("--marker-len", type=float, default=0.023, help="marker 實際邊長 (m)")
    ap.add_argument("--min-corners", type=int, default=6, help="每張至少偵測到幾個 charuco 角才採用")
    ap.add_argument("--out", default="camera_intrinsics.json")
    args = ap.parse_args()

    aruco_dict = aruco.getPredefinedDictionary(DICT)
    board = build_board(args.square_len, args.marker_len, aruco_dict)
    params = make_detector_params()

    paths = sorted(glob.glob(args.images))
    if not paths:
        raise SystemExit(f"找不到任何照片: {args.images}")

    all_corners, all_ids = [], []
    image_size = None
    used, skipped = [], []

    for path in paths:
        img = cv2.imread(path)
        if img is None:
            skipped.append((path, "unreadable"))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]  # (w, h)

        corners, ids = detect_markers(gray, aruco_dict, params)
        if ids is None or len(ids) == 0:
            skipped.append((path, "no markers"))
            continue

        retval, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if ch_corners is None or retval < args.min_corners:
            skipped.append((path, f"only {retval} charuco corners"))
            continue

        all_corners.append(ch_corners)
        all_ids.append(ch_ids)
        used.append((path, int(retval)))

    print(f"採用 {len(used)}/{len(paths)} 張照片；跳過 {len(skipped)} 張")
    for p, n in used:
        print(f"  [use] {p}  charuco_corners={n}")
    for p, why in skipped:
        print(f"  [skip] {p}  ({why})")

    if len(all_corners) < 5:
        raise SystemExit(f"可用照片太少 ({len(all_corners)})，至少要 5 張(建議 15+，多角度多距離)。")

    rms, K, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, image_size, None, None
    )

    print("\n=== 校正結果 ===")
    print(f"RMS reprojection error: {rms:.4f} px  (< 1.0 佳, 1~2 尚可, >2 建議重拍)")
    print("Camera matrix K =\n", K)
    print("dist coeffs =", dist.ravel())

    result = {
        "image_size": {"width": image_size[0], "height": image_size[1]},
        "camera_matrix": K.tolist(),
        "dist_coeffs": dist.ravel().tolist(),
        "reprojection_error_px": float(rms),
        "square_len_m": args.square_len,
        "marker_len_m": args.marker_len,
        "images_used": len(all_corners),
        "images_total": len(paths),
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(f"\n已寫入 {args.out}")


if __name__ == "__main__":
    main()
