"""載入相機內參，偵測 ArUco 標籤，量出每個標籤相對相機的位置與角度。

這是 Nav 定位的第一步：把校正好的內參 (K + 畸變) 用起來，回答
「相機看到的這張 marker，離我多遠、在哪個方位」。

用法:
    # 對單張照片
    python3 aruco_pose.py --intrinsics camera_intrinsics_640x480.json \\
        --marker-len 0.05 --image some_photo.jpg

    # 對 PW310P 即時串流 (印出每一幀偵測到的 marker 位姿；Ctrl-C 結束)
    python3 aruco_pose.py --intrinsics camera_intrinsics_640x480.json \\
        --marker-len 0.05 --camera 0

--marker-len 要填『你實際印出來、貼在場地上那張定位 marker 的邊長 (公尺)』，
量到多少填多少。座標定義: 相機光心為原點，+x 右、+y 下、+z 前 (公尺)。

**攝影機硬體 180° 倒放**：本車的 PW310P 是上下顛倒安裝的，繞光軸轉了 180°，
所以原始影像的左右、上下都是反的。預設 --rotate-180 會在偵測前把每一幀轉正，
讓輸出的 +x=右、+y=下 與現實一致（左右/上下號一次修好，下游不必再反號）。
若哪天改成正放，加 --no-rotate-180 關掉。
"""
import argparse
import json
import os
import sys

import cv2
from cv2 import aruco
import numpy as np

# 相機 180° 倒放的單一事實來源在 phase1/camera_orientation.py。這支是 standalone
# 工具(常在 cwd=ChArUco 執行)，補上 repo root 才 import 得到；真的找不到就退回
# 同樣預設(倒放)，讓工具在沒有 phase1 的環境仍可跑。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from phase1.camera_orientation import CAMERA_ROTATE_180, upright
except ImportError:
    CAMERA_ROTATE_180 = True

    def upright(frame, rotate_180=CAMERA_ROTATE_180):
        return cv2.rotate(frame, cv2.ROTATE_180) if rotate_180 else frame

# Default localization-marker dictionary; calibration boards may use another family.
DEFAULT_DICT = "DICT_6X6_250"
# Default public marker black-edge length in metres.
DEFAULT_MARKER_LEN = 0.17


def load_intrinsics(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    K = np.array(data["camera_matrix"], dtype=np.float64)
    dist = np.array(data["dist_coeffs"], dtype=np.float64)
    size = data.get("image_size")
    return K, dist, size


def make_detector_params():
    if hasattr(aruco, "DetectorParameters_create"):  # 4.5/4.6
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()  # 4.7+


def detect_markers(gray, aruco_dict, params):
    if hasattr(aruco, "ArucoDetector"):  # 4.7+
        corners, ids, _ = aruco.ArucoDetector(aruco_dict, params).detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    return corners, ids


def estimate_pose(marker_corners, marker_len, K, dist):
    """回傳 (rvec, tvec)。優先用 estimatePoseSingleMarkers，新版沒有就退回 solvePnP。"""
    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers([marker_corners], marker_len, K, dist)
        return rvecs[0][0], tvecs[0][0]
    # 4.7+ 已移除 estimatePoseSingleMarkers → 自己用 solvePnP
    half = marker_len / 2.0
    obj = np.array([[-half, half, 0], [half, half, 0],
                    [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, marker_corners.reshape(-1, 2), K, dist)
    return rvec.ravel(), tvec.ravel()


def yaw_deg_from_rvec(rvec):
    """marker 繞相機 Y 軸的偏擺角 (度)，粗略指示 marker 面朝哪。"""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    return float(np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))))


def report_frame(gray, aruco_dict, params, marker_len, K, dist, label=""):
    corners, ids = detect_markers(gray, aruco_dict, params)
    if ids is None or len(ids) == 0:
        print(f"{label}未偵測到任何 marker")
        return
    for marker_corners, mid in zip(corners, ids.ravel()):
        rvec, tvec = estimate_pose(marker_corners, marker_len, K, dist)
        x, y, z = tvec
        dist_m = float(np.linalg.norm(tvec))
        print(f"{label}id={int(mid):<3d} x={x:+.3f} y={y:+.3f} z={z:+.3f} m  "
              f"距離={dist_m:.3f} m  yaw={yaw_deg_from_rvec(rvec):+.1f}°")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--intrinsics", default="camera_intrinsics_640x480.json")
    ap.add_argument("--marker-len", type=float, default=DEFAULT_MARKER_LEN, help="定位 marker 實際邊長 (m)")
    ap.add_argument("--dict", default=DEFAULT_DICT, help="ArUco 字典 (需與列印的 marker 一致)")
    ap.add_argument("--image", help="對單張照片量測")
    ap.add_argument("--camera", type=int, help="對相機 index 即時量測 (例如 0 = /dev/video0)")
    ap.add_argument("--rotate-180", dest="rotate_180", action="store_true", default=CAMERA_ROTATE_180,
                    help="相機 180° 倒放 → 偵測前把影像轉正 (本車硬體預設開啟)")
    ap.add_argument("--no-rotate-180", dest="rotate_180", action="store_false",
                    help="相機正放時關閉旋轉")
    args = ap.parse_args()

    K, dist, size = load_intrinsics(args.intrinsics)
    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, args.dict))
    params = make_detector_params()
    print(f"內參: {args.intrinsics} (校正解析度 {size})，marker 邊長 {args.marker_len*1000:.0f}mm，"
          f"字典 {args.dict}，180°轉正={'開' if args.rotate_180 else '關'}")

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise SystemExit(f"讀不到照片: {args.image}")
        img = upright(img, args.rotate_180)
        report_frame(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), aruco_dict, params,
                     args.marker_len, K, dist)
        return

    if args.camera is not None:
        if size:
            pass  # 提醒: 相機串流解析度需與內參校正解析度相符
        cap = cv2.VideoCapture(args.camera)
        if size:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, size["width"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size["height"])
        if not cap.isOpened():
            raise SystemExit(f"打不開相機 index {args.camera}")
        actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print(f"相機串流解析度 {actual}（須與內參校正解析度相符）。Ctrl-C 結束。")
        try:
            n = 0
            while True:
                ok, frame = cap.read()
                if not ok:
                    print("讀取影格失敗")
                    break
                frame = upright(frame, args.rotate_180)
                n += 1
                report_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), aruco_dict, params,
                             args.marker_len, K, dist, label=f"[frame {n}] ")
        except KeyboardInterrupt:
            print("\n結束")
        finally:
            cap.release()
        return

    ap.error("需指定 --image 或 --camera")


if __name__ == "__main__":
    main()
