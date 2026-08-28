#!/usr/bin/env python3
"""用已登錄 marker 自動測繪新 marker 的 map 位姿，寫入 room_marker_map.json。

原理：同一幀裡同時看到「已知 marker」（例如 id=1，map 位姿已登錄）與
「新 marker」。對已知者 solvePnP 得 world→optical，對新者以 marker 本地
座標 solvePnP 得 local→optical，兩者相除即 local→world，取中心與 yaw。
多幀取中位數。假設新 marker 貼在鉛直面上（傾斜會引入誤差）。

用法（在 Pi 上、車靜止、兩張 marker 同時入鏡）：
    python3 tools/survey_marker.py --new-id 4 --new-dict DICT_5X5_50 \
        --new-black-size-m 0.13 [--frames 30] [--write]
不加 --write 只印結果；加了才寫入 config/room_marker_map.json。
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer  # noqa: E402
from vgr_driver.vision.camera_orientation import upright  # noqa: E402

sys.path.insert(0, str(REPO_ROOT / "ros2_ws/src/vgr_safety_gate"))
from vgr_safety_gate.aruco_camera_pose import charuco_to_camera_info  # noqa: E402

MARKER_MAP_PATH = REPO_ROOT / "config/room_marker_map.json"
INTRINSICS_PATH = REPO_ROOT / "ChArUco/camera_intrinsics_640x480.json"


def _detect(gray, dict_name, wanted_id):
    from cv2 import aruco

    d = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    if hasattr(aruco, "DetectorParameters"):
        p = aruco.DetectorParameters()
    else:
        p = aruco.DetectorParameters_create()
    p.minMarkerDistanceRate = 0.03
    if hasattr(aruco, "ArucoDetector"):
        corners, ids, _ = aruco.ArucoDetector(d, p).detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, d, parameters=p)
    if ids is None:
        return None
    for corner, mid in zip(corners, ids.ravel()):
        if int(mid) == wanted_id:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
            cv2.cornerSubPix(gray, corner[0], (3, 3), (-1, -1), criteria)
            return corner[0].astype(np.float64)
    return None


def _local_corners(half: float) -> np.ndarray:
    """marker 本地座標（右=+x、上=+z、法線=+y 朝外），角點順序同偵測輸出。"""
    return np.array(
        [
            [-half, 0.0, half],
            [half, 0.0, half],
            [half, 0.0, -half],
            [-half, 0.0, -half],
        ],
        dtype=np.float64,
    )


def survey_once(frame, known_marker, new_dict, new_id, new_half, cam_mtx, dist):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    known_dict = known_marker.get("dictionary", "DICT_6X6_250")
    img_known = _detect(gray, known_dict, int(known_marker["id"]))
    img_new = _detect(gray, new_dict, new_id)
    if img_known is None or img_new is None:
        return None

    world_corners = ArucoWorldLocalizer._marker_corners_world(known_marker)
    ok1, rvec_w, tvec_w = cv2.solvePnP(world_corners, img_known, cam_mtx, dist)
    ok2, rvec_l, tvec_l = cv2.solvePnP(_local_corners(new_half), img_new, cam_mtx, dist)
    if not (ok1 and ok2):
        return None
    r_wo, _ = cv2.Rodrigues(rvec_w)  # world -> optical
    r_lo, _ = cv2.Rodrigues(rvec_l)  # local -> optical
    r_lw = r_wo.T @ r_lo
    t_lw = (r_wo.T @ (tvec_l - tvec_w)).reshape(3)
    right_world = r_lw @ np.array([1.0, 0.0, 0.0])
    yaw = math.atan2(-float(right_world[0]), float(right_world[1]))
    return float(t_lw[0]), float(t_lw[1]), float(t_lw[2]), yaw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--known-id", type=int, default=1)
    parser.add_argument("--new-id", type=int, required=True)
    parser.add_argument("--new-dict", required=True)
    parser.add_argument("--new-black-size-m", type=float, required=True)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    marker_map = json.loads(MARKER_MAP_PATH.read_text(encoding="utf-8"))
    known = next(
        (m for m in marker_map["markers"] if int(m["id"]) == args.known_id), None
    )
    if known is None:
        raise SystemExit(f"known id {args.known_id} not in {MARKER_MAP_PATH}")
    if any(int(m["id"]) == args.new_id for m in marker_map["markers"]):
        print(f"note: id {args.new_id} already in map, will be replaced on --write")

    info = charuco_to_camera_info(json.loads(INTRINSICS_PATH.read_text()))
    cam_mtx = np.array(
        [[info["fx"], 0, info["cx"]], [0, info["fy"], info["cy"]], [0, 0, 1]],
        dtype=np.float64,
    )
    dist = np.array(info["dist_coeffs"], dtype=np.float64)

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, info["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, info["height"])
    samples = []
    tries = 0
    while len(samples) < args.frames and tries < args.frames * 4:
        ok, frame = cap.read()
        tries += 1
        if not ok:
            continue
        result = survey_once(
            upright(frame), known, args.new_dict, args.new_id,
            args.new_black_size_m / 2.0, cam_mtx, dist,
        )
        if result is not None:
            samples.append(result)
    cap.release()

    if len(samples) < max(5, args.frames // 3):
        raise SystemExit(f"only {len(samples)} usable frames; need both markers visible")

    xs, ys, zs, yaws = zip(*samples)
    yaw = math.atan2(
        statistics.median(math.sin(v) for v in yaws),
        statistics.median(math.cos(v) for v in yaws),
    )
    entry = {
        "id": args.new_id,
        "x": round(statistics.median(xs), 4),
        "y": round(statistics.median(ys), 4),
        "z": round(statistics.median(zs), 4),
        "yaw": round(yaw, 4),
        "size_m": args.new_black_size_m,
        "black_size_m": args.new_black_size_m,
        "dictionary": args.new_dict,
    }
    print(json.dumps({"samples": len(samples), "entry": entry,
                      "yaw_deg": round(math.degrees(yaw), 1)}, indent=1))

    if args.write:
        marker_map["markers"] = [
            m for m in marker_map["markers"] if int(m["id"]) != args.new_id
        ]
        marker_map["markers"].append(entry)
        MARKER_MAP_PATH.write_text(
            json.dumps(marker_map, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        print(f"written to {MARKER_MAP_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
