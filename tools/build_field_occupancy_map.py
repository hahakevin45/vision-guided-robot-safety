#!/usr/bin/env python3
"""Generate a Nav2 occupancy map for the synthetic public field fixture."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import numpy as np

# Synthetic public field; intentionally unrelated to any measured site.
WALL_CORNERS = [(0.0, -0.7), (2.5, -0.6), (2.4, 1.9), (0.2, 1.8)]

RESOLUTION = 0.02
PADDING = 0.30

# 牆＝0（占用）、場內＝254（自由）、場外＝205（未知）
VAL_WALL = 0
VAL_FREE = 254
VAL_UNKNOWN = 205


def point_to_segment_dist(wx: np.ndarray, wy: np.ndarray, p1: tuple[float, float], p2: tuple[float, float]) -> np.ndarray:
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return np.sqrt((wx - x1) ** 2 + (wy - y1) ** 2)
    t = ((wx - x1) * dx + (wy - y1) * dy) / L2
    t = np.clip(t, 0.0, 1.0)
    px = x1 + t * dx
    py = y1 + t * dy
    return np.sqrt((wx - px) ** 2 + (wy - py) ** 2)


def point_in_polygon(wx: np.ndarray, wy: np.ndarray, corners: list[tuple[float, float]]) -> np.ndarray:
    inside = np.zeros_like(wx, dtype=bool)
    n = len(corners)
    for i in range(n):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % n]

        # Ray casting
        cond_y = ((y1 <= wy) & (wy < y2)) | ((y2 <= wy) & (wy < y1))

        if np.any(cond_y):
            # Compute x intersection for the horizontal ray
            x_intersect = x1 + (wy - y1) * (x2 - x1) / (y2 - y1)
            intersect_mask = cond_y & (wx < x_intersect)
            inside ^= intersect_mask

    return inside


def world_to_pixel(wx: float, wy: float, origin_x: float, origin_y: float, res: float, H: int) -> tuple[int, int]:
    """世界座標轉像素座標。

    公式：
    col = round((wx - origin_x) / res)
    row = (H - 1) - round((wy - origin_y) / res)
    """
    col = round((wx - origin_x) / res)
    row = (H - 1) - round((wy - origin_y) / res)
    return col, row


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("ros2_ws/src/vgr_nav2_bringup/maps"),
        help="輸出目錄 (預設: ros2_ws/src/vgr_nav2_bringup/maps)"
    )
    args = parser.parse_args()

    # 計算範圍 (牆角外擴 PADDING 的軸對齊 bbox)
    xs = [c[0] for c in WALL_CORNERS]
    ys = [c[1] for c in WALL_CORNERS]

    xmin = round(min(xs) - PADDING, 4)
    xmax = round(max(xs) + PADDING, 4)
    ymin = round(min(ys) - PADDING, 4)
    ymax = round(max(ys) + PADDING, 4)

    # 確定 origin 座標 (影像左下角像素的世界座標)
    origin_x = xmin
    origin_y = ymin

    # 計算圖像維度 H 和 W
    W = int(round((xmax - xmin) / RESOLUTION)) + 1
    H = int(round((ymax - ymin) / RESOLUTION)) + 1

    # 建立網格像素中心的世界座標
    r_indices = np.arange(H)
    c_indices = np.arange(W)
    c_grid, r_grid = np.meshgrid(c_indices, r_indices)

    # 像素中心世界座標
    wx_grid = origin_x + c_grid * RESOLUTION
    wy_grid = origin_y + (H - 1 - r_grid) * RESOLUTION

    # Draw a narrow wall band without consuming the interior footprint margin.
    dist_min = np.full_like(wx_grid, np.inf)
    for i in range(4):
        p1 = WALL_CORNERS[i]
        p2 = WALL_CORNERS[(i + 1) % 4]
        dist = point_to_segment_dist(wx_grid, wy_grid, p1, p2)
        dist_min = np.minimum(dist_min, dist)

    is_inside = point_in_polygon(wx_grid, wy_grid, WALL_CORNERS)
    is_wall = ((dist_min <= 0.06) & (~is_inside)) | (dist_min <= 0.01)

    # 2. 場內判定 (在多邊形內且不是牆)
    is_inside_not_wall = is_inside & (~is_wall)

    # 初始化為未知
    img = np.full((H, W), VAL_UNKNOWN, dtype=np.uint8)

    # 填入牆與場內
    img[is_inside_not_wall] = VAL_FREE
    img[is_wall] = VAL_WALL

    # 輸出目錄
    args.out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = args.out_dir / "vgr_field.pgm"
    yaml_path = args.out_dir / "vgr_field.yaml"

    # 寫入 PGM P5
    pgm_header = f"P5\n{W} {H}\n255\n".encode("ascii")
    with open(pgm_path, "wb") as f:
        f.write(pgm_header)
        f.write(img.tobytes())

    # 寫入 YAML
    yaml_content = f"""image: vgr_field.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    yaml_path.write_text(yaml_content, encoding="utf-8")

    print(f"Successfully generated map files in {args.out_dir}:")
    print(f"  PGM: {pgm_path.name} (shape: {W}x{H})")
    print(f"  YAML: {yaml_path.name}")
    print(f"  Origin: [{origin_x}, {origin_y}, 0.0]")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
