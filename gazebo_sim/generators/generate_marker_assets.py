"""產生 Gazebo 牆面用的 ArUco marker PNG 資產。"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from gazebo_sim.generators.generate_arena_world import (
    MARKER_SIZE_M,
    MARKER_THICKNESS_M,
    SDF_VERSION,
)


DEFAULT_IDS = tuple(range(8))
DEFAULT_SIZE_PX = 512
DEFAULT_OUTPUT_DIR = "gazebo_sim/models/markers/textures"
DEFAULT_MODELS_DIR = "gazebo_sim/models"

# 與 vgr_driver.vision.ArucoDetector 預設字串保持一致。
ARUCO_DICTIONARY_NAME = "DICT_6X6_250"


def _add_text(parent: ET.Element, tag: str, text: object) -> ET.Element:
    elem = ET.SubElement(parent, tag)
    elem.text = str(text)
    return elem


def _fmt(value: float) -> str:
    return f"{value:.12g}"


def _box_size(x: float, y: float, z: float) -> str:
    return " ".join(_fmt(part) for part in (x, y, z))


def _aruco_dictionary(name: str = ARUCO_DICTIONARY_NAME):
    try:
        dictionary_id = getattr(cv2.aruco, name)
    except AttributeError as exc:
        raise ValueError(f"unknown ArUco dictionary: {name}") from exc
    if hasattr(cv2.aruco, "Dictionary_get"):
        return cv2.aruco.Dictionary_get(dictionary_id)
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def _parse_ids(text: str) -> tuple[int, ...]:
    """解析 CLI marker ID 清單，支援逗號與閉區間，例如 ``0-7,12``。"""
    ids: list[int] = []
    for raw_part in text.split(","):
        part = raw_part.strip()
        if not part:
            continue
        if "-" in part:
            start_text, end_text = part.split("-", 1)
            start = int(start_text)
            end = int(end_text)
            if end < start:
                raise ValueError(f"Invalid marker ID range: {part}")
            ids.extend(range(start, end + 1))
        else:
            ids.append(int(part))
    if not ids:
        raise ValueError("At least one marker ID is required")
    return tuple(ids)


def _draw_marker(marker_id: int, size_px: int,
                 dictionary_name: str = ARUCO_DICTIONARY_NAME) -> np.ndarray:
    """產生含白色 quiet zone 的單張 marker 影像。"""
    if size_px < 40:
        raise ValueError("size_px must be at least 40")
    module_px = size_px // 10
    marker_px = module_px * 8
    margin_px = (size_px - marker_px) // 2
    canvas = np.full((size_px, size_px), 255, dtype=np.uint8)
    dictionary = _aruco_dictionary(dictionary_name)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(
            dictionary,
            marker_id,
            marker_px,
            borderBits=1,
        )
    else:
        marker = np.zeros((marker_px, marker_px), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, marker_px, marker, 1)
    canvas[margin_px:margin_px + marker_px, margin_px:margin_px + marker_px] = marker
    # 輸出 3 通道：ogre2 的 PBR albedo_map 載入 8-bit 灰階 PNG 會靜默失敗
    # （實測渲染成黑面），必須是 RGB。
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def build_marker_model_sdf(model_name: str,
                           marker_size_m: float = MARKER_SIZE_M) -> str:
    """回傳單一 marker model SDF；貼圖路徑相對於 model.sdf。"""
    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    model = ET.SubElement(sdf, "model", {"name": model_name})
    _add_text(model, "static", "true")
    link = ET.SubElement(model, "link", {"name": f"{model_name}_link"})

    size = _box_size(MARKER_THICKNESS_M, marker_size_m, marker_size_m)
    collision = ET.SubElement(link, "collision", {"name": f"{model_name}_collision"})
    geometry = ET.SubElement(collision, "geometry")
    box = ET.SubElement(geometry, "box")
    _add_text(box, "size", size)

    visual = ET.SubElement(link, "visual", {"name": f"{model_name}_visual"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    _add_text(box, "size", size)
    material = ET.SubElement(visual, "material")
    # SDF material 的 diffuse/ambient 預設是黑色 (0,0,0,1)，只給 albedo_map
    # 會被乘成全黑（實測）。必須明確設白色讓貼圖原色呈現。
    _add_text(material, "ambient", "1 1 1 1")
    _add_text(material, "diffuse", "1 1 1 1")
    _add_text(material, "specular", "0 0 0 1")
    # 自發光：牆內側常背對太陽光，只靠 diffuse 會暗到偵測不到。
    # 注意純色 emissive 會蓋掉貼圖（實測全白）；必須用 emissive_map
    # 帶著 marker 圖案自發光，等效實車高反差印刷 marker。
    _add_text(material, "emissive", "1 1 1 1")
    pbr = ET.SubElement(material, "pbr")
    metal = ET.SubElement(pbr, "metal")
    # Fortress 對裸相對路徑靜默解析失敗（實測渲染成黑面）；
    # model 內貼圖也必須用完整 model:// URI，經 IGN_GAZEBO_RESOURCE_PATH 解析。
    texture_uri = f"model://{model_name}/materials/textures/{model_name}.png"
    _add_text(metal, "albedo_map", texture_uri)
    _add_text(metal, "emissive_map", texture_uri)

    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="unicode")


def build_marker_model_config(model_name: str,
                              sdf_filename: str = "model.sdf") -> str:
    """回傳 Gazebo model://marker_ID URI 解析所需的 manifest。"""
    model = ET.Element("model")
    _add_text(model, "name", model_name)
    _add_text(model, "version", "1.0")
    _add_text(model, "sdf", sdf_filename).set("version", SDF_VERSION)
    author = ET.SubElement(model, "author")
    _add_text(author, "name", "Vision Guided Robot")
    _add_text(model, "description", f"Gazebo Fortress ArUco wall marker {model_name} for VGR.")

    ET.indent(model, space="  ")
    return ET.tostring(model, encoding="unicode")


def _write_marker_model(marker_id: int, legacy_png_path: Path, models_dir: Path,
                        model_name: str, marker_size_m: float) -> list[Path]:
    model_dir = models_dir / model_name
    texture_dir = model_dir / "materials" / "textures"
    texture_dir.mkdir(parents=True, exist_ok=True)

    texture_path = texture_dir / f"{model_name}.png"
    shutil.copyfile(legacy_png_path, texture_path)

    sdf_path = model_dir / "model.sdf"
    sdf_path.write_text(build_marker_model_sdf(model_name, marker_size_m) + "\n",
                        encoding="utf-8")

    config_path = model_dir / "model.config"
    config_path.write_text(build_marker_model_config(model_name) + "\n", encoding="utf-8")
    return [texture_path, sdf_path, config_path]


def generate_marker_assets(
    ids: tuple[int, ...] = DEFAULT_IDS,
    size_px: int = DEFAULT_SIZE_PX,
    output_dir: str | os.PathLike[str] = DEFAULT_OUTPUT_DIR,
    models_dir: str | os.PathLike[str] = DEFAULT_MODELS_DIR,
    *,
    dictionary_name: str = ARUCO_DICTIONARY_NAME,
    model_prefix: str = "marker_",
    marker_size_m: float = MARKER_SIZE_M,
) -> list[Path]:
    """寫出舊 PNG 與標準 Gazebo marker models，回傳檔案路徑清單。"""
    if not model_prefix or "/" in model_prefix:
        raise ValueError("model_prefix must be a non-empty path-free prefix")
    if not math.isfinite(marker_size_m) or marker_size_m <= 0.0:
        raise ValueError("marker_size_m must be a finite positive value")
    directory = Path(output_dir)
    model_root = Path(models_dir)
    directory.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    for marker_id in ids:
        model_name = f"{model_prefix}{marker_id}"
        path = directory / f"{model_name}.png"
        image = _draw_marker(marker_id, size_px, dictionary_name)
        if not cv2.imwrite(str(path), image):
            raise OSError(f"Failed to write marker asset: {path}")
        paths.append(path)
        paths.extend(_write_marker_model(marker_id, path, model_root, model_name,
                                         marker_size_m))
    return paths


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="產生 Gazebo 牆面 ArUco marker PNG")
    parser.add_argument("--ids", default="0-7", help="marker ID 清單，例如 0-7 或 0,3,5")
    parser.add_argument("--size-px", type=int, default=DEFAULT_SIZE_PX, help="單張 PNG 邊長")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="輸出 textures 目錄")
    parser.add_argument("--models-dir", default=DEFAULT_MODELS_DIR, help="輸出 Gazebo marker model 根目錄")
    parser.add_argument("--dictionary", default=ARUCO_DICTIONARY_NAME, help="ArUco 字典，例如 DICT_6X6_250")
    parser.add_argument("--model-prefix", default="marker_", help="marker model 名稱前綴")
    parser.add_argument("--marker-size-m", type=float, default=MARKER_SIZE_M, help="marker 實體邊長（公尺）")
    args = parser.parse_args(argv)
    generate_marker_assets(
        _parse_ids(args.ids),
        args.size_px,
        args.output_dir,
        args.models_dir,
        dictionary_name=args.dictionary,
        model_prefix=args.model_prefix,
        marker_size_m=args.marker_size_m,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
