import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import pytest

from gazebo_sim.generators.generate_robot_sdf import CAMERA_FRONT_X_M, CAMERA_HEIGHT_M

from gazebo_sim.generators.generate_arena_world import MARKER_SIZE_M, MARKER_THICKNESS_M
from gazebo_sim.generators.generate_robot_sdf import build_robot_sdf


def _aruco_dictionary():
    if hasattr(cv2.aruco, "Dictionary_get"):
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)


def _aruco_parameters():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


def _detect(gray, dictionary, parameters):
    # OpenCV 5 removed the cv2.aruco.detectMarkers free function and returns
    # ids with shape (N,) where OpenCV 4 returns (N, 1).
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=parameters
        )
    if ids is not None and ids.ndim == 1:
        ids = ids.reshape(-1, 1)
    return corners, ids, rejected


def _detect_marker_id(path: Path) -> int:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    assert image is not None
    _corners, ids, _rejected = _detect(
        image, _aruco_dictionary(), _aruco_parameters()
    )
    assert ids is not None
    assert len(ids) == 1
    return int(ids[0][0])


def _parse_pose(text: str) -> tuple[float, float, float, float, float, float]:
    return tuple(float(part) for part in text.split())


def _marker_visuals(root: ET.Element) -> list[ET.Element]:
    return [
        visual
        for visual in root.iter("visual")
        if visual.attrib.get("name", "").startswith("marker_")
    ]


def _marker_includes(root: ET.Element) -> list[ET.Element]:
    return [
        include
        for include in root.iter("include")
        if (include.findtext("uri") or "").startswith("model://marker_")
    ]


def test_marker_asset_generator_writes_detectable_default_markers(tmp_path):
    output_dir = tmp_path / "textures"
    models_dir = tmp_path / "models"

    subprocess.run(
        [
            sys.executable,
            "-m",
            "gazebo_sim.generators.generate_marker_assets",
            "--output-dir",
            str(output_dir),
            "--models-dir",
            str(models_dir),
        ],
        check=True,
    )

    for marker_id in range(8):
        path = output_dir / f"marker_{marker_id}.png"
        assert path.exists()
        assert _detect_marker_id(path) == marker_id

        model_dir = models_dir / f"marker_{marker_id}"
        texture_path = model_dir / "materials" / "textures" / f"marker_{marker_id}.png"
        assert (model_dir / "model.config").exists()
        assert texture_path.exists()
        assert _detect_marker_id(texture_path) == marker_id

        sdf = ET.parse(model_dir / "model.sdf").getroot()
        assert sdf.attrib["version"] == "1.8"
        model = sdf.find("model")
        assert model is not None
        assert model.attrib["name"] == f"marker_{marker_id}"
        assert model.findtext("static") == "true"
        assert model.findtext(".//box/size") == f"{MARKER_THICKNESS_M:.12g} {MARKER_SIZE_M:.12g} {MARKER_SIZE_M:.12g}"
        albedo_map = sdf.findtext(".//material/pbr/metal/albedo_map")
        # 必須是完整 model:// URI：Fortress 對裸相對路徑靜默解析失敗（渲染成黑面）。
        assert albedo_map == f"model://marker_{marker_id}/materials/textures/marker_{marker_id}.png"
        relative_texture = albedo_map.removeprefix(f"model://marker_{marker_id}/")
        assert (model_dir / relative_texture).exists()

        config = ET.parse(model_dir / "model.config").getroot()
        assert config.findtext("name") == f"marker_{marker_id}"
        assert config.findtext("sdf") == "model.sdf"


def test_marker_asset_generator_accepts_custom_ids_and_size(tmp_path):
    output_dir = tmp_path / "textures"
    models_dir = tmp_path / "models"

    subprocess.run(
        [
            sys.executable,
            "-m",
            "gazebo_sim.generators.generate_marker_assets",
            "--ids",
            "2,5",
            "--size-px",
            "256",
            "--output-dir",
            str(output_dir),
            "--models-dir",
            str(models_dir),
        ],
        check=True,
    )

    assert sorted(path.name for path in output_dir.glob("marker_*.png")) == [
        "marker_2.png",
        "marker_5.png",
    ]
    image = cv2.imread(str(output_dir / "marker_2.png"), cv2.IMREAD_GRAYSCALE)
    assert image.shape == (256, 256)
    assert sorted(path.name for path in models_dir.glob("marker_*")) == [
        "marker_2",
        "marker_5",
    ]


def test_arena_world_writes_marker_map_and_places_markers_on_inner_walls(tmp_path):
    world_path = tmp_path / "worlds" / "vgr_arena.world"
    marker_map_path = tmp_path / "models" / "markers" / "marker_map.json"

    subprocess.run(
        [
            sys.executable,
            "-m",
            "gazebo_sim.generators.generate_arena_world",
            "--output",
            str(world_path),
            "--marker-map-output",
            str(marker_map_path),
        ],
        check=True,
    )

    data = json.loads(marker_map_path.read_text(encoding="utf-8"))
    markers = data["markers"]
    assert [marker["id"] for marker in markers] == list(range(8))
    assert {marker["size_m"] for marker in markers} == {MARKER_SIZE_M}

    by_id = {marker["id"]: marker for marker in markers}
    expected = {
        0: (4.0 - 0.001, -1 / 3, 0.10, math.pi),
        1: (4.0 - 0.001, 1 / 3, 0.10, math.pi),
        2: (0.0 + 0.001, -1 / 3, 0.10, 0.0),
        3: (0.0 + 0.001, 1 / 3, 0.10, 0.0),
        4: (4 / 3, 1.0 - 0.001, 0.10, -math.pi / 2),
        5: (8 / 3, 1.0 - 0.001, 0.10, -math.pi / 2),
        6: (4 / 3, -1.0 + 0.001, 0.10, math.pi / 2),
        7: (8 / 3, -1.0 + 0.001, 0.10, math.pi / 2),
    }
    for marker_id, pose in expected.items():
        marker = by_id[marker_id]
        assert marker["x"] == pytest.approx(pose[0])
        assert marker["y"] == pytest.approx(pose[1])
        assert marker["z"] == pytest.approx(pose[2])
        assert marker["yaw"] == pytest.approx(pose[3])


def test_world_xml_omits_sensors_by_default_and_includes_marker_models(tmp_path):
    world_path = tmp_path / "worlds" / "vgr_arena.world"
    marker_map_path = tmp_path / "models" / "markers" / "marker_map.json"
    texture_dir = tmp_path / "models" / "markers" / "textures"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "gazebo_sim.generators.generate_marker_assets",
            "--output-dir",
            str(texture_dir),
        ],
        check=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "gazebo_sim.generators.generate_arena_world",
            "--output",
            str(world_path),
            "--marker-map-output",
            str(marker_map_path),
        ],
        check=True,
    )

    root = ET.parse(world_path).getroot()
    plugins = {plugin.attrib.get("name"): plugin for plugin in root.iter("plugin")}
    assert "ignition::gazebo::systems::Sensors" not in plugins

    assert _marker_visuals(root) == []
    assert [include.findtext("uri") for include in _marker_includes(root)] == [
        f"model://marker_{marker_id}" for marker_id in range(8)
    ]
    assert list(root.iter("albedo_map")) == []


def test_world_xml_with_sensors_adds_ogre2_sensors_plugin(tmp_path):
    world_path = tmp_path / "worlds" / "vgr_arena_vision.world"
    marker_map_path = tmp_path / "models" / "markers" / "marker_map.json"

    subprocess.run(
        [
            sys.executable,
            "-m",
            "gazebo_sim.generators.generate_arena_world",
            "--with-sensors",
            "--output",
            str(world_path),
            "--marker-map-output",
            str(marker_map_path),
        ],
        check=True,
    )

    root = ET.parse(world_path).getroot()
    plugins = {plugin.attrib.get("name"): plugin for plugin in root.iter("plugin")}
    sensors = plugins["ignition::gazebo::systems::Sensors"]
    assert sensors.attrib["filename"] == "libignition-gazebo-sensors-system.so"
    assert sensors.findtext("render_engine") == "ogre2"


def test_robot_sdf_has_camera_sensor_and_camera_info_matches_sdf(tmp_path):
    model_path = tmp_path / "vgr_diff_drive" / "model.sdf"

    subprocess.run(
        [
            sys.executable,
            "-m",
            "gazebo_sim.generators.generate_robot_sdf",
            "--output",
            str(model_path),
        ],
        check=True,
    )

    root = ET.parse(model_path).getroot()
    sensor = root.find(".//sensor[@name='front_camera']")
    assert sensor is not None
    assert sensor.attrib["type"] == "camera"
    # The public model uses a 0.20 m front offset and 0.17 m camera height.
    assert _parse_pose(sensor.findtext("pose")) == pytest.approx(
        (CAMERA_FRONT_X_M, 0.0, CAMERA_HEIGHT_M, 0.0, 0.0, 0.0)
    )
    assert sensor.findtext("topic") == "/camera/image_raw"
    assert float(sensor.findtext("update_rate")) == pytest.approx(15.0)
    camera = sensor.find("camera")
    assert camera is not None
    assert float(camera.findtext("horizontal_fov")) == pytest.approx(1.2)
    assert int(camera.findtext("image/width")) == 640
    assert int(camera.findtext("image/height")) == 480

    info = json.loads((model_path.parent / "camera_info.json").read_text(encoding="utf-8"))
    expected_fx = 640 / (2.0 * math.tan(1.2 / 2.0))
    assert info == {
        "width": 640,
        "height": 480,
        "horizontal_fov_rad": 1.2,
        "fx": pytest.approx(expected_fx),
        "fy": pytest.approx(expected_fx),
        "cx": 320.0,
        "cy": 240.0,
        "topic": "/camera/image_raw",
    }
