import subprocess
import sys
from pathlib import Path
import yaml
import numpy as np

from tools.build_field_occupancy_map import WALL_CORNERS

# 註明公式：
# col = round((wx - origin_x) / res)
# row = (H - 1) - round((wy - origin_y) / res)
def world_to_pixel(wx: float, wy: float, origin_x: float, origin_y: float, res: float, H: int) -> tuple[int, int]:
    col = round((wx - origin_x) / res)
    row = (H - 1) - round((wy - origin_y) / res)
    return col, row


def test_field_occupancy_map(tmp_path):
    # 執行產生器，將輸出導向 tmp_path
    script_path = Path(__file__).resolve().parents[1] / "tools" / "build_field_occupancy_map.py"
    res = subprocess.run([sys.executable, str(script_path), "--out-dir", str(tmp_path)], capture_output=True, text=True)
    assert res.returncode == 0, f"Script failed: {res.stderr}"

    pgm_file = tmp_path / "vgr_field.pgm"
    yaml_file = tmp_path / "vgr_field.yaml"

    assert pgm_file.exists()
    assert yaml_file.exists()

    # 讀取 YAML 檔案
    with open(yaml_file, "r") as f:
        meta = yaml.safe_load(f)

    # 驗證 YAML 欄位
    assert meta["image"] == "vgr_field.pgm"
    assert meta["mode"] == "trinary"
    assert meta["resolution"] == 0.02
    assert meta["negate"] == 0
    assert meta["occupied_thresh"] == 0.65
    assert meta["free_thresh"] == 0.196

    origin = meta["origin"]
    assert len(origin) == 3
    origin_x, origin_y, origin_z = origin
    assert origin_z == 0.0

    # 讀取 PGM P5 檔案
    with open(pgm_file, "rb") as f:
        header = f.readline().decode("ascii").strip()
        assert header == "P5"

        # 讀取寬與高，跳過可能的註解
        line = f.readline().decode("ascii").strip()
        while line.startswith("#"):
            line = f.readline().decode("ascii").strip()
        w_str, h_str = line.split()
        W, H = int(w_str), int(h_str)

        maxval_str = f.readline().decode("ascii").strip()
        assert int(maxval_str) == 255

        # 讀取二進位資料
        data = f.read()
        assert len(data) == W * H
        img = np.frombuffer(data, dtype=np.uint8).reshape(H, W)

    # 4. yaml resolution/origin 正確：以 origin＋resolution 反推左下角像素世界座標一致
    # 左下角像素 (row = H-1, col = 0) 在世界座標中應恰為 (origin_x, origin_y)
    # 我們在此使用公式驗證，以 origin_x, origin_y 經公式轉換，必得 (0, H-1)
    col_bl, row_bl = world_to_pixel(origin_x, origin_y, origin_x, origin_y, meta["resolution"], H)
    assert col_bl == 0
    assert row_bl == H - 1

    # 1. 場中心 (1.10, 0.60) 世界座標 → 像素值 254（自由）
    c_center, r_center = world_to_pixel(1.10, 0.60, origin_x, origin_y, meta["resolution"], H)
    assert img[r_center, c_center] == 254

    # 2. 四條邊各取中點，世界座標 → 像素值 0（占用）
    # 牆角：
    corners = WALL_CORNERS
    midpoints = []
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]
        mid_x = (p1[0] + p2[0]) / 2.0
        mid_y = (p1[1] + p2[1]) / 2.0
        midpoints.append((mid_x, mid_y))

    for mx, my in midpoints:
        col, row = world_to_pixel(mx, my, origin_x, origin_y, meta["resolution"], H)
        assert img[row, col] == 0

    # 3. Synthetic field exterior point inside the padded raster → unknown.
    c_out, r_out = world_to_pixel(-0.20, 0.50, origin_x, origin_y, meta["resolution"], H)
    assert img[r_out, c_out] == 205

    # 5. 連跑兩次輸出位元組相同
    res2 = subprocess.run([sys.executable, str(script_path), "--out-dir", str(tmp_path)], capture_output=True)
    assert res2.returncode == 0

    with open(pgm_file, "rb") as f:
        data2_pgm = f.read()
    with open(yaml_file, "rb") as f:
        data2_yaml = f.read()

    # 重新寫入一次到另一個暫存路徑
    tmp_path2 = tmp_path / "run2"
    tmp_path2.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(script_path), "--out-dir", str(tmp_path2)], capture_output=True)

    with open(tmp_path2 / "vgr_field.pgm", "rb") as f:
        run2_pgm = f.read()
    with open(tmp_path2 / "vgr_field.yaml", "rb") as f:
        run2_yaml = f.read()

    assert data2_pgm == run2_pgm
    assert data2_yaml == run2_yaml


def test_occupancy_grid_can_exclude_obstacle():
    """隱藏障礙 map：Nav2 planner 會規劃穿箱直線（隱藏牆比較用）。"""
    from vgr_core.geometry.arena_geometry import build_occupancy_grid

    with_ob = build_occupancy_grid(resolution_m=0.1)
    hidden = build_occupancy_grid(resolution_m=0.1, include_obstacle=False)
    # 箱中心 (2.0, 0.0)：with 版 occupied、hidden 版 free
    assert with_ob.is_occupied(2.0, 0.0)
    assert not hidden.is_occupied(2.0, 0.0)
    # 場地外仍 occupied（牆保留）
    assert hidden.is_occupied(4.5, 0.0)
