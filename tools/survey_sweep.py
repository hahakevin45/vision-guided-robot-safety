#!/usr/bin/env python3
"""原地旋轉自動測繪：一次掃出整個場地的 marker 地圖。

車放在場地中央，本工具控制原地分段旋轉（兩個半圈來回，避免纏線），
每步多幀偵測可見 marker。**幀與幀之間用編碼器轉角當橋**（實測打滑
~2%）：相機 FOV 只有 ±26°，從場地中央大多數時候一幀只看得到一張
marker，「同框成對」不可行；改為——任何一步看到已登錄 marker 就能
錨定整個旋轉序列的絕對位姿，其他步驟看到的未知 marker 全部可解。

地圖座標系定義：**id 最小的 marker 中心 = 原點 (0,0)，其面向場內的
法線 = +x 軸**；z 由「相機高 0.10m、水平安裝、marker 鉛直」假設推得
（z 誤差只是整體平移，不影響 2D 定位）。

輸出 marker 地圖 JSON＋由 marker 中心凸包內縮的 geofence 提議。

用法（在 Pi 上、12V 開、車在場地中央）：
    python3 tools/survey_sweep.py --out config/field_marker_map.json \
        --black-size-m 0.15 [--dicts DICT_5X5_50,DICT_6X6_250] [--no-motion]
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer  # noqa: E402
from vgr_driver.vision.camera_orientation import upright  # noqa: E402
from vgr_core.model import CommandID  # noqa: E402
from vgr_driver.driver import ControllerBridge  # noqa: E402
from vgr_driver.driver import PosixSerial  # noqa: E402
from vgr_driver.cli.turn_angle import compute_turn_targets  # noqa: E402
from tools.survey_marker import _local_corners  # noqa: E402

sys.path.insert(0, str(REPO_ROOT / "ros2_ws/src/vgr_safety_gate"))
from vgr_safety_gate.aruco_camera_pose import charuco_to_camera_info  # noqa: E402

INTRINSICS_PATH = REPO_ROOT / "ChArUco/camera_intrinsics_640x480.json"
CAMERA_FWD_M = 0.10        # 相機在底盤中心前方（實測掛載）
CAMERA_Z_M = 0.10          # 相機光心離地高
SWEEP_STEP_DEG = 20.0
FRAMES_PER_STEP = 5
MIN_SAMPLES = 3


def detect_all(gray, dict_names: list[str]) -> dict[int, tuple[str, np.ndarray]]:
    """回傳 {id: (dict_name, corners 4x2)}；跨字典撞 id 直接丟棄該 id。"""
    from cv2 import aruco

    found: dict[int, tuple[str, np.ndarray]] = {}
    clashed: set[int] = set()
    for name in dict_names:
        d = aruco.getPredefinedDictionary(getattr(aruco, name))
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
            continue
        for corner, mid in zip(corners, ids.ravel()):
            mid = int(mid)
            if mid in found and found[mid][0] != name:
                clashed.add(mid)
                continue
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
            cv2.cornerSubPix(gray, corner[0], (3, 3), (-1, -1), criteria)
            found[mid] = (name, corner[0].astype(np.float64))
    for mid in clashed:
        found.pop(mid, None)
    return found


# ---------- 位姿幾何（level 相機、chassis 2D） ----------

def _camera_world_rt(chassis: tuple[float, float, float]):
    """chassis (x,y,yaw) → 相機 world→optical 的 (R, t)。"""
    x, y, yaw = chassis
    fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    down = np.array([0.0, 0.0, -1.0])
    r_wo = np.stack([right, down, fwd])
    cam_pos = np.array([x + CAMERA_FWD_M * math.cos(yaw),
                        y + CAMERA_FWD_M * math.sin(yaw),
                        CAMERA_Z_M])
    return r_wo, (-r_wo @ cam_pos)


def chassis_from_marker(entry: dict, img_corners: np.ndarray,
                        cam_mtx, dist) -> tuple[float, float, float] | None:
    """從單一已登錄 marker 解 chassis (x,y,yaw)。"""
    world = ArucoWorldLocalizer._marker_corners_world(entry)
    ok, rvec, tvec = cv2.solvePnP(world, img_corners, cam_mtx, dist)
    if not ok:
        return None
    r_wo, _ = cv2.Rodrigues(rvec)
    cam_pos = (-r_wo.T @ tvec).reshape(3)
    fwd = r_wo.T @ np.array([0.0, 0.0, 1.0])   # optical z 軸的世界方向
    yaw = math.atan2(float(fwd[1]), float(fwd[0]))
    return (float(cam_pos[0]) - CAMERA_FWD_M * math.cos(yaw),
            float(cam_pos[1]) - CAMERA_FWD_M * math.sin(yaw),
            yaw)


def marker_from_chassis(chassis: tuple[float, float, float],
                        img_corners: np.ndarray, half: float,
                        cam_mtx, dist) -> tuple[float, float, float, float] | None:
    """已知 chassis 位姿，解未知 marker 的 (x,y,z,yaw)。"""
    ok, rvec_l, tvec_l = cv2.solvePnP(_local_corners(half), img_corners,
                                      cam_mtx, dist)
    if not ok:
        return None
    r_lo, _ = cv2.Rodrigues(rvec_l)
    r_wo, t_wo = _camera_world_rt(chassis)
    r_lw = r_wo.T @ r_lo
    t_lw = (r_wo.T @ (tvec_l.reshape(3) - t_wo)).reshape(3)
    right_world = r_lw @ np.array([1.0, 0.0, 0.0])
    yaw = math.atan2(-float(right_world[0]), float(right_world[1]))
    return float(t_lw[0]), float(t_lw[1]), float(t_lw[2]), yaw


def _median_pose(samples: list[tuple[float, float, float]]):
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    yaw = math.atan2(statistics.median(math.sin(s[2]) for s in samples),
                     statistics.median(math.cos(s[2]) for s in samples))
    return statistics.median(xs), statistics.median(ys), yaw


def chain_map(
    steps: list[dict],
    ref_id: int,
    black_size_m: float,
    cam_mtx,
    dist,
    passes: int = 3,
) -> dict[int, dict]:
    """編碼器旋轉橋接的鏈式解算。

    steps: [{"cum_rad": 累計轉角, "obs": [ {id:(dict,corners)}, ... ]}, ...]
    """
    half = black_size_m / 2.0
    # 基準張：原點、法線 +x。z 之後由 chassis 解算間接一致（先擺 0，
    # 下面用相機高假設回填）。
    ref_dict = None
    for step in steps:
        for obs in step["obs"]:
            if ref_id in obs:
                ref_dict = obs[ref_id][0]
                break
        if ref_dict:
            break
    if ref_dict is None:
        raise RuntimeError(f"ref id {ref_id} never observed")

    # z_ref：相機高假設（marker 本地 +z 向上）
    z_samples = []
    for step in steps:
        for obs in step["obs"]:
            if ref_id in obs:
                ok, rvec, tvec = cv2.solvePnP(
                    _local_corners(half), obs[ref_id][1], cam_mtx, dist)
                if ok:
                    r, _ = cv2.Rodrigues(rvec)
                    cam_in_marker = (-r.T @ tvec).reshape(3)
                    z_samples.append(CAMERA_Z_M - float(cam_in_marker[2]))
    entries: dict[int, dict] = {ref_id: {
        "id": ref_id, "x": 0.0, "y": 0.0,
        "z": round(statistics.median(z_samples), 4), "yaw": 0.0,
        "size_m": black_size_m, "black_size_m": black_size_m,
        "dictionary": ref_dict,
    }}

    ref_entry = entries[ref_id]
    # 階段一：增量爬行登錄。編碼器橋一次只允許跨 BRIDGE_STEPS 步
    # （2% 打滑 × 60° ≈ 1.2°），每登錄一張 marker 錨定區域就沿圓推進，
    # 避免一次跨大角度累積打滑誤差。
    BRIDGE_STEPS = 3
    changed = True
    while changed:
        changed = False
        anchors = {}
        for idx, step in enumerate(steps):
            samples = []
            for obs in step["obs"]:
                for mid, (dname, corners) in obs.items():
                    if mid in entries:
                        pose = chassis_from_marker(entries[mid], corners,
                                                   cam_mtx, dist)
                        if pose is not None:
                            samples.append(pose)
            if samples:
                anchors[idx] = _median_pose(samples)
        if not anchors:
            raise RuntimeError("no step could be anchored")
        pending: dict[int, list] = {}
        pending_dict: dict[int, str] = {}
        for idx, step in enumerate(steps):
            near = min(anchors, key=lambda a: abs(a - idx))
            if abs(near - idx) > BRIDGE_STEPS:
                continue
            ax, ay, ayaw = anchors[near]
            yaw = ayaw + (step["cum_rad"] - steps[near]["cum_rad"])
            chassis = (ax, ay, math.atan2(math.sin(yaw), math.cos(yaw)))
            for obs in step["obs"]:
                for mid, (dname, corners) in obs.items():
                    if mid in entries:
                        continue
                    est = marker_from_chassis(chassis, corners, half,
                                              cam_mtx, dist)
                    if est is not None:
                        pending.setdefault(mid, []).append(est)
                        pending_dict[mid] = dname
        for mid, samples in pending.items():
            if len(samples) < MIN_SAMPLES:
                continue
            xs, ys, zs, yaws = zip(*samples)
            yaw = math.atan2(statistics.median(math.sin(v) for v in yaws),
                             statistics.median(math.cos(v) for v in yaws))
            entries[mid] = {
                "id": mid, "x": round(statistics.median(xs), 4),
                "y": round(statistics.median(ys), 4),
                "z": round(statistics.median(zs), 4),
                "yaw": round(yaw, 4),
                "size_m": black_size_m, "black_size_m": black_size_m,
                "dictionary": pending_dict[mid],
            }
            changed = True

    # 階段二：bundle adjustment。爬行結果只當初值——每跳橋接的打滑
    # 誤差會沿鏈累積（無迴環閉合），交給全域最小平方一次收斂：
    # 參數 = 非基準 marker 位姿 (x,y,z,yaw) ＋ 每個有觀測步的底盤
    # (x,y,yaw)；殘差 = 角點重投影誤差；基準張固定＝規範原點。
    return bundle_adjust(steps, entries, ref_id, black_size_m, cam_mtx, dist)


def bundle_adjust(steps, entries, ref_id, black_size_m, cam_mtx, dist):
    """Pose-graph 全域最小平方。

    2026-07-15 實掃教訓：線材拖拽讓打滑不均勻（同一編碼器角度兩次經
    過視野差幾十度），「yaw = θ0 + k·cum」的剛性模型直接發散。改成：
    - 每個有觀測步的底盤 (x,y,yaw) 全部自由
    - 相鄰步之間加「編碼器相對轉角」軟約束（σ 隨轉角放大，容忍
      15% 不均勻打滑）→ 單 marker 步驟仍無規範自由度、迴環閉合
    - 位置軟先驗拉向掃描中心（原地旋轉，σ 8cm 容忍拖拽漂移）
    - soft_l1 穩健損失＋參數界限，杜絕發散；解算後 RMS 檢查。
    """
    from scipy.optimize import least_squares

    marker_ids = [m for m in sorted(entries) if m != ref_id]
    obs_steps = [i for i, s in enumerate(steps)
                 if any(m in entries for o in s["obs"] for m in o)]
    # 初值：有錨的步用視覺、沒錨的用最近錨＋編碼器差外插
    anchor_pose = {}
    for idx in obs_steps:
        samples = []
        for obs in steps[idx]["obs"]:
            for mid, (dname, corners) in obs.items():
                if mid in entries:
                    pose = chassis_from_marker(entries[mid], corners,
                                               cam_mtx, dist)
                    if pose is not None:
                        samples.append(pose)
        if samples:
            anchor_pose[idx] = _median_pose(samples)
    if not anchor_pose:
        raise RuntimeError("bundle_adjust: no anchored step")
    px0 = statistics.median(a[0] for a in anchor_pose.values())
    py0 = statistics.median(a[1] for a in anchor_pose.values())
    # θ0、k 初值：錨定步 yaw 對 cum 展開後擬合（k = 平均打滑係數）
    a_idx = sorted(anchor_pose)
    yaws = []
    prev_cum = None
    for i in a_idx:
        raw = anchor_pose[i][2]
        if yaws is not None and yaws:
            expect = yaws[-1] + (steps[i]["cum_rad"] - prev_cum)
            while raw - expect > math.pi:
                raw -= 2 * math.pi
            while raw - expect < -math.pi:
                raw += 2 * math.pi
        yaws.append(raw)
        prev_cum = steps[i]["cum_rad"]
    cums_a = [steps[i]["cum_rad"] for i in a_idx]
    if len(set(round(c, 6) for c in cums_a)) >= 3:
        k0, theta0 = np.polyfit(cums_a, yaws, 1)
    else:
        k0, theta0 = 1.0, yaws[0] - cums_a[0]

    # 位置與 yaw 的漂移都是隨機漫步（萬向輪走位、打滑抖動）：
    # 約束「相鄰步增量」（σ/步）；弱絕對先驗只防規範退化。
    SIGMA_POS_STEP_M = 0.02
    SIGMA_POS_ABS_M = 0.30
    SIGMA_DYAW_STEP = math.radians(1.5)
    SIGMA_DYAW_ABS = math.radians(20.0)

    def pack():
        p = []
        for mid in marker_ids:
            e = entries[mid]
            p += [e["x"], e["y"], e["z"], e["yaw"]]
        p += [float(theta0), float(k0)]
        for idx in obs_steps:
            ax, ay, _ = anchor_pose.get(idx, (px0, py0, 0.0))
            p += [ax, ay, 0.0]
        return np.array(p, dtype=np.float64)

    def unpack(p):
        markers = {ref_id: entries[ref_id]}
        for k, mid in enumerate(marker_ids):
            x, y, z, yaw = p[4 * k: 4 * k + 4]
            markers[mid] = {**entries[mid], "x": x, "y": y, "z": z, "yaw": yaw}
        base = 4 * len(marker_ids)
        th0, kk = p[base], p[base + 1]
        chassis = {}
        for j, idx in enumerate(obs_steps):
            x, y, dyaw = p[base + 2 + 3 * j: base + 2 + 3 * j + 3]
            chassis[idx] = (x, y, th0 + kk * steps[idx]["cum_rad"] + dyaw)
        return markers, chassis

    observations = [
        (idx, mid, corners)
        for idx in obs_steps
        for obs in steps[idx]["obs"]
        for mid, (dname, corners) in obs.items()
        if mid in entries
    ]
    def residuals(p):
        markers, chassis = unpack(p)
        base_p = 4 * len(marker_ids)
        n = len(obs_steps)
        res = np.empty(len(observations) * 8 + 3 * n + 3 * (n - 1))
        for k, (idx, mid, corners) in enumerate(observations):
            world = ArucoWorldLocalizer._marker_corners_world(markers[mid])
            r_wo, t_wo = _camera_world_rt(chassis[idx])
            rvec, _ = cv2.Rodrigues(r_wo)
            img, _ = cv2.projectPoints(world, rvec, t_wo, cam_mtx, dist)
            res[8 * k: 8 * k + 8] = (img.reshape(4, 2) - corners).ravel()
        base = len(observations) * 8
        dyaws = p[base_p + 4::3]
        xs = p[base_p + 2::3]
        ys = p[base_p + 3::3]
        for j, idx in enumerate(obs_steps):
            res[base + 3 * j] = (xs[j] - px0) / SIGMA_POS_ABS_M
            res[base + 3 * j + 1] = (ys[j] - py0) / SIGMA_POS_ABS_M
            res[base + 3 * j + 2] = dyaws[j] / SIGMA_DYAW_ABS
        base += 3 * n
        for j in range(n - 1):
            gap = max(1, obs_steps[j + 1] - obs_steps[j])
            sq = math.sqrt(gap)
            res[base + 3 * j] = (dyaws[j + 1] - dyaws[j]) / (
                SIGMA_DYAW_STEP * sq)
            res[base + 3 * j + 1] = (xs[j + 1] - xs[j]) / (
                SIGMA_POS_STEP_M * sq)
            res[base + 3 * j + 2] = (ys[j + 1] - ys[j]) / (
                SIGMA_POS_STEP_M * sq)
        return res

    p0 = pack()
    lo = np.full_like(p0, -np.inf)
    hi = np.full_like(p0, np.inf)
    for k in range(len(marker_ids)):
        lo[4 * k: 4 * k + 3] = [px0 - 6.0, py0 - 6.0, -0.5]
        hi[4 * k: 4 * k + 3] = [px0 + 6.0, py0 + 6.0, 2.0]
    base = 4 * len(marker_ids)
    lo[base + 1], hi[base + 1] = 0.5, 1.5      # 打滑係數合理界
    for j in range(len(obs_steps)):
        lo[base + 2 + 3 * j: base + 2 + 3 * j + 2] = [px0 - 1.0, py0 - 1.0]
        hi[base + 2 + 3 * j: base + 2 + 3 * j + 2] = [px0 + 1.0, py0 + 1.0]
        lo[base + 2 + 3 * j + 2] = -0.5
        hi[base + 2 + 3 * j + 2] = 0.5
    # 初值可能因矛盾觀測落在界外（如架空演練）：clip 進界限，
    # 讓解算跑完由 RMS 檢查優雅判失敗，而非 ValueError。
    p0 = np.clip(p0, lo, hi)
    # 迭代剔除離群觀測（V4L2 舊幀等會讓個別觀測整幀錯位）：
    # 解一次 → 丟掉 per-obs RMS > max(5×中位數, 20px) 的觀測 → 重解。
    for trim_round in range(3):
        result = least_squares(residuals, p0, bounds=(lo, hi), method="trf",
                               loss="soft_l1", f_scale=3.0, max_nfev=300)
        n_reproj = len(observations) * 8
        per_obs = np.sqrt(
            (result.fun[:n_reproj].reshape(-1, 8) ** 2).mean(axis=1))
        rms = math.sqrt(float(np.mean(result.fun[:n_reproj] ** 2)))
        median_rms = float(np.median(per_obs))
        cut = max(5.0 * median_rms, 20.0)
        keep = per_obs <= cut
        print(f"BA round {trim_round}: {len(observations)} obs, "
              f"RMS {rms:.2f}px, median {median_rms:.2f}px, "
              f"trimming {int((~keep).sum())}")
        if keep.all():
            break
        observations = [o for o, k in zip(observations, keep) if k]
        p0 = np.clip(result.x, lo, hi)
    markers, _ = unpack(result.x)
    if rms > 15.0:
        raise RuntimeError(
            f"bundle adjustment diverged (RMS {rms:.1f}px) — 重掃或檢查線材")
    refined = {}
    for mid, e in markers.items():
        yaw = math.atan2(math.sin(e["yaw"]), math.cos(e["yaw"]))
        refined[mid] = {**e, "x": round(float(e["x"]), 4),
                        "y": round(float(e["y"]), 4),
                        "z": round(float(e["z"]), 4),
                        "yaw": round(float(yaw), 4)}
    return refined


def suggest_geofence(entries: dict[int, dict], inset_m: float = 0.05) -> list[float]:
    """marker 中心凸包向質心內縮 inset 當 geofence 提議（牆上的 marker ≈ 房間輪廓）。"""
    pts = np.array([[e["x"], e["y"]] for e in entries.values()], dtype=np.float32)
    hull = cv2.convexHull(pts).reshape(-1, 2)
    centroid = hull.mean(axis=0)
    fence: list[float] = []
    for p in hull:
        d = p - centroid
        norm = float(np.linalg.norm(d))
        if norm > inset_m:
            p = p - d / norm * inset_m
        fence.extend([round(float(p[0]), 3), round(float(p[1]), 3)])
    return fence


class TurnSession:
    """整個掃描共用一條 serial 連線。

    2026-07-15 實掃教訓：每步轉向都重開 /dev/ttyACM0 會讓 STM32 重置，
    開機的 ~1 秒馬達不受控暴衝（編碼器同時歸零，暴衝量無人記錄）——
    36 步 = 36 次暴衝，掃描位姿全毀。單一連線 + 開機長 settle 根治。
    turn() 回傳編碼器實際達成的 yaw 變化（rad），掃描用實際量當 cum。
    """

    WHEEL_BASE_M = 0.165
    WHEEL_DIAMETER_CM = 6.5
    LEFT_CPR, RIGHT_CPR = 750.0, 749.0

    def __init__(self, device: str, cruise_cps: int = 150,
                 arm_wait_s: float = 30.0) -> None:
        self._cruise = cruise_cps
        self._serial = PosixSerial(device=device, baudrate=115200,
                                   timeout_s=0.5)
        self._serial.__enter__()
        time.sleep(2.5)              # STM32 重置後等它開機完、狀態穩定
        self._serial.flush_input()
        self.bridge = ControllerBridge(self._serial)
        self.bridge.send_command(CommandID.HEARTBEAT)
        self.bridge.send_command(CommandID.STOP)
        if arm_wait_s > 0:
            # 上電窗口：開埠（STM32 重置）時 12V 應為關，此窗口內操作者
            # 把 12V 打開；期間持續發 STOP。
            print(f"== 上電窗口 {arm_wait_s:.0f}s：現在把 12V 打開 ==",
                  flush=True)
            start = time.monotonic()
            while time.monotonic() - start < arm_wait_s:
                self.bridge.send_command(CommandID.STOP)
                time.sleep(0.2)

    def turn(self, step_deg: float, max_seconds: float = 5.0) -> float:
        """閉環轉一步。**編碼器看門狗**（2026-07-15 暴衝排查）：命令
        在跑但計數進展 <20% 預期、持續 0.6s → 視為編碼器回報凍結
        （馬達可能仍在轉＝無回授暴衝），立即 STOP 並中止。"""
        targets = compute_turn_targets(
            step_deg, self.WHEEL_BASE_M, self.WHEEL_DIAMETER_CM,
            self.LEFT_CPR, self.RIGHT_CPR)
        initial = self.bridge.read_encoders()
        init_l = initial.packet.left_count
        init_r = initial.packet.right_count
        left_reached = right_reached = False
        start = time.monotonic()
        enc = initial
        poll_s = 0.05
        window: list[tuple[int, int]] = []   # 看門狗滑動窗（counts）
        WATCH_N = int(0.6 / poll_s)
        polls = []
        stall = False
        try:
            while time.monotonic() - start < max_seconds:
                enc = self.bridge.read_encoders()
                cur = (enc.packet.left_count, enc.packet.right_count)
                polls.append(cur)
                d_l = abs(cur[0] - init_l)
                d_r = abs(cur[1] - init_r)
                left_reached = d_l >= targets["left_target_counts"]
                right_reached = d_r >= targets["right_target_counts"]
                if left_reached and right_reached:
                    break
                window.append(cur)
                if len(window) > WATCH_N:
                    moved = max(abs(cur[0] - window[0][0]),
                                abs(cur[1] - window[0][1]))
                    expect = self._cruise * poll_s * WATCH_N
                    if moved < 0.2 * expect:
                        stall = True
                        break
                    window.pop(0)
                left_cmd = 0 if left_reached else (
                    targets["left_sign"] * self._cruise)
                right_cmd = 0 if right_reached else (
                    targets["right_sign"] * self._cruise)
                self.bridge.send_set_wheel_speed(left_cmd, right_cmd)
                time.sleep(poll_s)
        finally:
            self.bridge.send_command(CommandID.STOP)
        if stall:
            raise RuntimeError(
                f"encoder watchdog: no progress in 0.6s "
                f"(motors may be running unmeasured!) polls={polls[-14:]}")
        if not (left_reached and right_reached):
            raise RuntimeError(
                f"turn {step_deg}° did not reach targets in {max_seconds}s; "
                f"polls={polls[-8:]}")
        circ = math.pi * self.WHEEL_DIAMETER_CM / 100.0
        arc_l = (enc.packet.left_count - init_l) / self.LEFT_CPR * circ
        arc_r = (enc.packet.right_count - init_r) / self.RIGHT_CPR * circ
        return (arc_r - arc_l) / self.WHEEL_BASE_M   # 右轉（deg>0）→ 負

    def close(self) -> None:
        try:
            self.bridge.send_command(CommandID.STOP)
        finally:
            self._serial.__exit__(None, None, None)


def capture_step(cap, dict_names) -> list[dict]:
    obs_list = []
    # V4L2 buffers frames; flush them before sampling after rotation.
    for _ in range(4):
        cap.read()
    for _ in range(FRAMES_PER_STEP * 2):
        ok, frame = cap.read()
        if not ok:
            continue
        found = detect_all(
            cv2.cvtColor(upright(frame), cv2.COLOR_BGR2GRAY), dict_names)
        if found:
            obs_list.append(found)
        if len(obs_list) >= FRAMES_PER_STEP:
            break
    return obs_list


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="config/field_marker_map.json")
    parser.add_argument("--black-size-m", type=float, default=0.15)
    parser.add_argument("--dicts", default="DICT_5X5_50,DICT_6X6_250")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--no-motion", action="store_true")
    parser.add_argument("--save-obs", default="outputs/sweep_observations.json")
    parser.add_argument("--from-obs", default=None,
                        help="跳過掃描，用先前存檔的觀測離線重解")
    parser.add_argument("--arm-wait-s", type=float, default=30.0,
                        help="上電窗口秒數（開埠時 12V 應為關）")
    args = parser.parse_args()
    dict_names = [d.strip() for d in args.dicts.split(",") if d.strip()]

    info = charuco_to_camera_info(json.loads(INTRINSICS_PATH.read_text()))
    cam_mtx = np.array(
        [[info["fx"], 0, info["cx"]], [0, info["fy"], info["cy"]], [0, 0, 1]],
        dtype=np.float64,
    )
    dist = np.array(info["dist_coeffs"], dtype=np.float64)

    if args.from_obs:
        raw = json.loads(Path(args.from_obs).read_text())
        steps = [
            {"cum_rad": s["cum_rad"],
             "obs": [{int(mid): (v[0], np.array(v[1], dtype=np.float64))
                      for mid, v in o.items()} for o in s["obs"]]}
            for s in raw["steps"]
        ]
    else:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, info["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, info["height"])

        # 兩個半圈來回：+20°×9 → −20°×18 → +20°×9，淨旋轉歸零、纏線最多半圈
        plan = ([+SWEEP_STEP_DEG] * 9 + [-SWEEP_STEP_DEG] * 18
                + [+SWEEP_STEP_DEG] * 9)
        if args.no_motion:
            plan = []

        steps = []
        cum = 0.0

        def _save_obs():
            if not args.save_obs:
                return
            save_path = REPO_ROOT / args.save_obs
            save_path.parent.mkdir(parents=True, exist_ok=True)
            save_path.write_text(json.dumps({"steps": [
                {"cum_rad": s["cum_rad"],
                 "obs": [{str(mid): [v[0], np.asarray(v[1]).tolist()]
                          for mid, v in o.items()} for o in s["obs"]]}
                for s in steps
            ]}))

        session = (TurnSession(args.device, arm_wait_s=args.arm_wait_s)
                   if plan else None)
        try:
            obs = capture_step(cap, dict_names)
            steps.append({"cum_rad": 0.0, "obs": obs})
            _save_obs()
            seen0 = set().union(*[o.keys() for o in obs]) if obs else set()
            print(f"step 0 (0°): markers {sorted(seen0)}")
            for i, step_deg in enumerate(plan, start=1):
                try:
                    cum += session.turn(step_deg)   # 編碼器實際達成量
                except RuntimeError as exc:
                    # 轉向卡住（漂移撞牆等）：優雅收尾，用既有資料解算
                    print(f"step {i}: turn aborted ({exc}); "
                          f"solving with {len(steps)} steps")
                    break
                time.sleep(0.4)
                obs = capture_step(cap, dict_names)
                steps.append({"cum_rad": cum, "obs": obs})
                _save_obs()   # 每步即存：中途失敗資料不丟
                step_ids = (set().union(*[o.keys() for o in obs])
                            if obs else set())
                print(f"step {i} ({math.degrees(cum):+.0f}°): "
                      f"{sorted(step_ids)}")
        finally:
            if session is not None:
                session.close()
        cap.release()
        if args.save_obs:
            print(f"raw observations saved to {REPO_ROOT / args.save_obs}")

    seen = set()
    for s in steps:
        for o in s["obs"]:
            seen |= set(o.keys())
    if not seen:
        raise SystemExit("no markers detected at all")
    ref_id = min(seen)
    entries = chain_map(steps, ref_id, args.black_size_m, cam_mtx, dist)
    missing = sorted(seen - set(entries))
    marker_map = {
        "frame": "map",
        "dictionary": entries[ref_id]["dictionary"],
        "markers": [entries[k] for k in sorted(entries)],
    }
    out_path = REPO_ROOT / args.out
    out_path.write_text(
        json.dumps(marker_map, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    fence = suggest_geofence(entries)
    print(json.dumps({
        "ref_id": ref_id,
        "registered": sorted(entries),
        "missing": missing,
        "geofence_suggestion": fence,
        "out": str(out_path),
    }, indent=1))
    for e in marker_map["markers"]:
        print(f"  id={e['id']} ({e['x']:+.3f}, {e['y']:+.3f}) z={e['z']:.2f} "
              f"yaw={math.degrees(e['yaw']):+.1f}° {e['dictionary']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
