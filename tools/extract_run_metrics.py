#!/usr/bin/env python3
"""安全濾波器實車對比實驗：單輪指標抽取。

從一輪實車實驗（mcap bag + harness report JSON）抽出對比用指標，輸出單輪
指標 JSON。解析 bag 使用 mcap_ros2.reader（用法同 tools/extract_bag_traj.py）。

用法：
    python3 tools/extract_run_metrics.py <bag目錄> --report <field_goal_*.json> \
        -o <out.json> [--label safe_apf_S1]

指標定義見專案規格；時間戳一律取 mcap log_time_ns。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from mcap_ros2.reader import read_ros2_messages

# 圍欄多邊形（場地固定幾何，內正外負）。
FENCE_POLYGON = [(-0.08, -0.59), (2.24, -0.61), (2.27, 1.72), (0.34, 1.74)]

# cmd_vel 名目取樣率（安全濾波器 20Hz）。
CMD_DT = 1.0 / 20.0
# cmd_vel 兩 topic 最近鄰配對容忍。
PAIR_TOL_S = 0.1
# aruco 判定盲走的相鄰間隔門檻。
BLIND_GAP_S = 1.0


def _load_bag(mcap_path: str) -> dict[str, list]:
    """一次掃過 bag，收集各 topic 需要的欄位。

    回傳 dict：
      status: [(t, mode, debug_dict), ...]
      cmd_nav / cmd_safe: [(t, linear_x, angular_z), ...]
      aruco: [t, ...]
      fused: [(x, y), ...]
      odom: [(x, y), ...]
    """
    out: dict[str, list] = {
        "status": [], "cmd_nav": [], "cmd_safe": [],
        "aruco": [], "fused": [], "odom": [],
    }
    for m in read_ros2_messages(mcap_path):
        t = m.log_time_ns / 1e9
        topic = m.channel.topic
        r = m.ros_msg
        if topic == "/safety_gate/status":
            d = json.loads(r.data)
            out["status"].append((t, d.get("mode"), d.get("debug", {})))
        elif topic == "/cmd_vel_nav":
            out["cmd_nav"].append((t, r.linear.x, r.angular.z))
        elif topic == "/cmd_vel_safe":
            out["cmd_safe"].append((t, r.linear.x, r.angular.z))
        elif topic == "/aruco/pose":
            out["aruco"].append(t)
        elif topic == "/pose_fused":
            pp = r.pose.pose
            out["fused"].append((pp.position.x, pp.position.y))
        elif topic == "/odom":
            pp = r.pose.pose
            out["odom"].append((pp.position.x, pp.position.y))
    return out


def _min_wall_clearance(status: list) -> float | None:
    vals = [dbg.get("min_wall_d_m") for _, _, dbg in status if dbg.get("min_wall_d_m") is not None]
    return min(vals) if vals else None


def _mode_fractions(status: list) -> dict[str, float]:
    counts = {"PASS": 0, "MODIFIED": 0, "STOP": 0}
    total = 0
    for _, mode, _ in status:
        if mode in counts:
            counts[mode] += 1
        total += 1
    if total == 0:
        return {k: 0.0 for k in counts}
    return {k: counts[k] / total for k in counts}


def _intervention_l1(cmd_nav: list, cmd_safe: list, tol: float = PAIR_TOL_S) -> float | None:
    """對每個 nav 指令找最近鄰 safe 指令（容忍 tol），取 L1 差異均值。

    L1 = |v_nav - v_safe| + |w_nav - w_safe|。
    """
    if not cmd_nav or not cmd_safe:
        return None
    safe = sorted(cmd_safe)
    safe_t = [s[0] for s in safe]
    import bisect
    diffs = []
    for t, vn, wn in cmd_nav:
        i = bisect.bisect_left(safe_t, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(safe):
                dt = abs(safe_t[j] - t)
                if best is None or dt < best[0]:
                    best = (dt, safe[j])
        if best is not None and best[0] <= tol:
            _, (_, vs, ws) = best
            diffs.append(abs(vn - vs) + abs(wn - ws))
    if not diffs:
        return None
    return sum(diffs) / len(diffs)


def _smoothness_jerk(cmd_safe: list, dt: float = CMD_DT) -> float | None:
    """/cmd_vel_safe linear.x 的二階差分 RMS / dt²（名目 20Hz）。"""
    xs = [c[1] for c in sorted(cmd_safe)]
    if len(xs) < 3:
        return None
    acc = 0.0
    n = 0
    for i in range(1, len(xs) - 1):
        second = (xs[i + 1] - 2 * xs[i] + xs[i - 1]) / (dt * dt)
        acc += second * second
        n += 1
    return math.sqrt(acc / n) if n else None


def _motion_jerk(cmd_safe: list, dt: float = CMD_DT, v_min: float = 0.01) -> float | None:
    """運動中 jerk：只統計 |v|>v_min 的樣本窗——龜速爬行的濾波器
    raw jerk 天然低（「癱瘓」偽裝成「平滑」，2026-07-19 S1 對照發現）。"""
    xs = [c[1] for c in sorted(cmd_safe)]
    if len(xs) < 3:
        return None
    acc = 0.0
    n = 0
    for i in range(1, len(xs) - 1):
        if abs(xs[i]) <= v_min:
            continue
        second = (xs[i + 1] - 2 * xs[i] + xs[i - 1]) / (dt * dt)
        acc += second * second
        n += 1
    return math.sqrt(acc / n) if n else None


def _goal_reach(fused: list, goal_xy, tol: float = 0.15) -> tuple[bool, float | None]:
    """位置制到點判定：fused 首次進入 goal tol 圈的相對時刻。
    harness 判詞在 PID 鏈上被終段搖頭/視覺過期污染，不可信。"""
    if not fused or not goal_xy or goal_xy[0] is None:
        return False, None
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    # fused 列可能是 (x,y)（無時戳）或 (t,x,y,...)。
    has_t = len(fused[0]) >= 3
    t0 = fused[0][0] if has_t else None
    for i, row in enumerate(fused):
        if has_t:
            t, x, y = row[0], row[1], row[2]
        else:
            x, y = row[0], row[1]
        if math.hypot(x - gx, y - gy) <= tol:
            return True, (t - t0) if has_t else i * 0.05  # 無時戳用 20Hz 名目
    return False, None


def _blind_metrics(aruco: list, status: list, gap: float = BLIND_GAP_S) -> tuple[float, float | None]:
    """aruco 相鄰間隔 > gap 的總時長；status debug 的 max(blind_dist_m)。"""
    ts = sorted(aruco)
    blind_total = 0.0
    for a, b in zip(ts, ts[1:]):
        d = b - a
        if d > gap:
            blind_total += d
    dists = [dbg.get("blind_dist_m") for _, _, dbg in status if dbg.get("blind_dist_m") is not None]
    blind_max = max(dists) if dists else None
    return blind_total, blind_max


def _point_in_poly(x: float, y: float, poly: list) -> bool:
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _dist_point_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def _signed_dist_to_poly(x: float, y: float, poly: list) -> float:
    """點到多邊形邊界最短距離；內部為正、外部為負。"""
    n = len(poly)
    d = min(
        _dist_point_segment(x, y, poly[i][0], poly[i][1],
                            poly[(i + 1) % n][0], poly[(i + 1) % n][1])
        for i in range(n)
    )
    return d if _point_in_poly(x, y, poly) else -d


def _fence_metrics(fused: list, poly: list = FENCE_POLYGON) -> tuple[float | None, int]:
    if not fused:
        return None, 0
    signed = [_signed_dist_to_poly(x, y, poly) for x, y in fused]
    return min(signed), sum(1 for s in signed if s < 0)


def _odom_metrics(odom: list) -> tuple[float | None, float | None]:
    if len(odom) < 2:
        return (0.0 if odom else None), (0.0 if odom else None)
    net = math.hypot(odom[-1][0] - odom[0][0], odom[-1][1] - odom[0][1])
    path = 0.0
    for (x0, y0), (x1, y1) in zip(odom, odom[1:]):
        path += math.hypot(x1 - x0, y1 - y0)
    return net, path


def compute_metrics(bag_dir: str, report_path: str, label: str | None = None) -> dict:
    p = Path(bag_dir)
    mcap_path = str(p if p.suffix == ".mcap" else next(p.glob("*.mcap")))
    data = _load_bag(mcap_path)
    report = json.loads(Path(report_path).read_text())

    blind_total, blind_max = _blind_metrics(data["aruco"], data["status"])
    fence_min, fence_viol = _fence_metrics(data["fused"])
    odom_net, odom_path = _odom_metrics(data["odom"])
    reached, t_reach = _goal_reach(data["fused"], report.get("goal_xy"))

    return {
        "label": label or p.name,
        # 主淨空欄=圍欄有號距離（全濾波器通用）；min_wall_d 是 safe_apf 專屬 debug
        "min_wall_clearance_m": _min_wall_clearance(data["status"]),
        "success": bool(report.get("action_status") == "SUCCEEDED"),
        "time_to_goal_s": report.get("elapsed_s"),
        "goal_error_m": report.get("goal_position_error_m"),
        "reached_pos": reached,
        "t_reach_s": t_reach,
        "intervention_l1": _intervention_l1(data["cmd_nav"], data["cmd_safe"]),
        "smoothness_jerk": _smoothness_jerk(data["cmd_safe"]),
        "motion_jerk": _motion_jerk(data["cmd_safe"]),
        "mode_fractions": _mode_fractions(data["status"]),
        "blind_total_s": blind_total,
        "blind_max_dist_m": blind_max,
        "fence_min_signed_m": fence_min,
        "fence_violations": fence_viol,
        "odom_net_m": odom_net,
        "odom_path_m": odom_path,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", help="bag 目錄或 .mcap 檔")
    ap.add_argument("--report", required=True, help="harness report JSON (field_goal_*.json)")
    ap.add_argument("-o", "--out", required=True, help="輸出指標 JSON")
    ap.add_argument("--label", default=None, help="這輪的標籤，如 safe_apf_S1")
    args = ap.parse_args()

    metrics = compute_metrics(args.bag, args.report, args.label)
    Path(args.out).write_text(json.dumps(metrics, indent=2))
    print(f"wrote {args.out}")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
