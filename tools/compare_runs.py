#!/usr/bin/env python3
"""把多輪單輪指標（extract_run_metrics.py 的輸出）排成 markdown 對照表。

用法：
    python3 tools/compare_runs.py m1.json m2.json ... -o table.md

每列一輪（label），每欄一項指標；表下自動以粗體標出每欄最佳值。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

# 欄位定義：(metrics key, 表頭, 小數位, 何者為佳)。
# better: "max" 越大越好、"min" 越小越好、None 不評比。
COLUMNS = [
    ("reached_pos", "reached", None, "max"),
    ("t_reach_s", "t_reach(s)", 1, "min"),
    ("fence_min_signed_m", "clearance(m)", 3, "max"),
    ("fence_violations", "fence_viol", 0, "min"),
    ("intervention_l1", "intervene_L1", 4, "min"),
    ("motion_jerk", "motion_jerk", 3, "min"),
    ("blind_total_s", "blind(s)", 2, "min"),
    ("blind_max_dist_m", "blind_max(m)", 3, "min"),
    ("odom_net_m", "odom_net(m)", 3, None),
    ("odom_path_m", "odom_path(m)", 3, "min"),
    ("mode_PASS", "PASS", 3, "max"),
    ("mode_MODIFIED", "MODIFIED", 3, None),
    ("mode_STOP", "STOP", 3, "min"),
]


def _flatten(m: dict) -> dict:
    """展開 mode_fractions 成 mode_PASS / mode_MODIFIED / mode_STOP。"""
    flat = dict(m)
    mf = m.get("mode_fractions") or {}
    for k in ("PASS", "MODIFIED", "STOP"):
        flat[f"mode_{k}"] = mf.get(k)
    return flat


def _fmt(val, decimals) -> str:
    if val is None:
        return "-"
    if isinstance(val, bool):
        return "Y" if val else "N"
    if decimals is None:
        return str(val)
    return f"{val:.{decimals}f}"


def _numeric(val):
    if val is None:
        return None
    if isinstance(val, bool):
        return 1.0 if val else 0.0
    if isinstance(val, (int, float)):
        return float(val)
    return None


def build_table(metrics: list[dict]) -> str:
    rows = [_flatten(m) for m in metrics]
    labels = [r.get("label", f"run{i}") for i, r in enumerate(rows)]

    # 找每欄最佳列索引（可並列）。
    best_idx: dict[str, set] = {}
    for key, _, _, better in COLUMNS:
        if better is None:
            continue
        nums = [(_numeric(r.get(key)), i) for i, r in enumerate(rows)]
        nums = [(v, i) for v, i in nums if v is not None]
        if not nums:
            continue
        target = max(v for v, _ in nums) if better == "max" else min(v for v, _ in nums)
        best_idx[key] = {i for v, i in nums if v == target}

    header = ["label"] + [h for _, h, _, _ in COLUMNS]
    lines = ["| " + " | ".join(header) + " |",
             "| " + " | ".join(["---"] * len(header)) + " |"]
    for i, r in enumerate(rows):
        cells = [labels[i]]
        for key, _, dec, _ in COLUMNS:
            s = _fmt(r.get(key), dec)
            if key in best_idx and i in best_idx[key] and s != "-":
                s = f"**{s}**"
            cells.append(s)
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("metrics", nargs="+", help="單輪指標 JSON 檔（多份）")
    ap.add_argument("-o", "--out", required=True, help="輸出 markdown 表")
    args = ap.parse_args()

    metrics = [json.loads(Path(p).read_text()) for p in args.metrics]
    table = build_table(metrics)
    Path(args.out).write_text(table)
    print(f"wrote {args.out}")
    print(table)


if __name__ == "__main__":
    main()
