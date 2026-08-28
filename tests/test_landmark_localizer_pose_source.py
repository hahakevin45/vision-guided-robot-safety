from pathlib import Path


LOCALIZER = Path("nav2_integration/landmark_localizer.py")


def test_landmark_localizer_defaults_to_fused_pose_source() -> None:
    source = LOCALIZER.read_text(encoding="utf-8")
    assert 'declare_parameter("pose_source", "fused")' in source
    assert 'PoseWithCovarianceStamped, "/pose_fused"' in source
    assert 'PoseStamped, "/aruco/pose"' in source
    assert 'pose_source not in ("fused", "aruco")' in source


def test_landmark_localizer_fused_callback_uses_nested_pose() -> None:
    source = LOCALIZER.read_text(encoding="utf-8")
    assert "msg.pose.pose" in source
    assert "def _on_fused_pose" in source
    assert "def _on_aruco_pose" in source
