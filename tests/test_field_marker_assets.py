"""Task 1: 5x5 field marker assets use a parameterized dictionary, prefix, and size."""
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import pytest

from gazebo_sim.generators.generate_marker_assets import generate_marker_assets


def _dictionary(name: str):
    value = getattr(cv2.aruco, name)
    getter = getattr(cv2.aruco, "getPredefinedDictionary", None)
    return getter(value) if getter else cv2.aruco.Dictionary_get(value)


def _detected_id(path: Path, dictionary_name: str) -> int:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    assert image is not None
    dictionary = _dictionary(dictionary_name)
    if hasattr(cv2.aruco, "ArucoDetector"):
        _corners, ids, _rejected = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    else:
        _corners, ids, _rejected = cv2.aruco.detectMarkers(image, dictionary)
    assert ids is not None and len(ids) == 1
    return int(ids.reshape(-1)[0])


def test_field_assets_use_5x5_dictionary_prefix_and_support_size(tmp_path):
    textures = tmp_path / "textures"
    models = tmp_path / "models"
    generate_marker_assets(
        ids=tuple(range(6)), output_dir=textures, models_dir=models,
        dictionary_name="DICT_5X5_50", model_prefix="field_marker_",
        marker_size_m=0.1875,
    )
    for marker_id in range(6):
        model_name = f"field_marker_{marker_id}"
        assert _detected_id(textures / f"{model_name}.png", "DICT_5X5_50") == marker_id
        sdf = ET.parse(models / model_name / "model.sdf").getroot()
        assert sdf.find("model").attrib["name"] == model_name
        assert sdf.findtext(".//box/size") == "0.002 0.1875 0.1875"
        assert sdf.findtext(".//albedo_map") == (
            f"model://{model_name}/materials/textures/{model_name}.png")


def test_field_generation_does_not_create_legacy_model_names(tmp_path):
    generate_marker_assets(
        ids=(0, 1), output_dir=tmp_path / "textures", models_dir=tmp_path / "models",
        dictionary_name="DICT_5X5_50", model_prefix="field_marker_",
        marker_size_m=0.1875,
    )
    assert not (tmp_path / "models" / "marker_0").exists()
    assert (tmp_path / "models" / "field_marker_0").exists()


@pytest.mark.parametrize("bad_size", [
    float("nan"),
    float("inf"),
    float("-inf"),
    0.0,
    -0.25,
])
def test_rejects_non_finite_or_non_positive_marker_size(tmp_path, bad_size):
    textures = tmp_path / "textures"
    models = tmp_path / "models"
    with pytest.raises(ValueError):
        generate_marker_assets(
            ids=(0,), output_dir=textures, models_dir=models,
            dictionary_name="DICT_5X5_50", model_prefix="field_marker_",
            marker_size_m=bad_size,
        )
    assert not textures.exists()
    assert not models.exists()
