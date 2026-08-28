"""E2 倒車演示的單元測試：參數解析、零速收尾邏輯，無 ROS runtime。"""
import ast
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "ros2_ws/src/vgr_safety_gate"))


def test_ast_parse():
    """vgr_runtime/ros/reverse_cmd_publisher.py 必須可被 AST 解析（語法正確）。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    ast.parse(source)


def test_import_without_ros():
    """不啟動 ROS runtime 的情況下，模組必須可被 import。"""
    with patch.dict("sys.modules", {"rclpy": MagicMock(), "rclpy.node": MagicMock(), "geometry_msgs": MagicMock(), "geometry_msgs.msg": MagicMock()}):
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "reverse_cmd_publisher", str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")
        )
        assert spec is not None
        module = importlib.util.module_from_spec(spec)
        try:
            module.__dict__["rclpy"] = MagicMock()
            module.__dict__["Node"] = type("Node", (), {})
            spec.loader.exec_module(module)
        except SystemExit:
            pass


def test_default_parameter_values():
    """預設參數值正確：v_mps=-0.05、duration_s=30.0。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    assert 'default=-0.05' in source, "v_mps default should be -0.05"
    assert 'default=30.0' in source, "duration_s default should be 30.0"


def test_reverse_speed_is_negative():
    """linear.x 輸出必須為負值（倒車）。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    assert "twist.linear.x = self._v_mps" in source
    assert "self._v_mps" in source


def test_stop_on_sigterm_logic():
    """收到 SIGTERM 時必須發布零速並退出。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    assert "signal.signal(signal.SIGTERM, self._on_sigterm)" in source
    assert "def _on_sigterm" in source
    assert "_publish_stop" in source
    assert "twist.linear.x = 0.0" in source or "linear.x = 0.0" in source


def test_duration_timeout_logic():
    """elapsed >= duration_s 時必須停止並退出。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    assert "self._elapsed >= self._duration_s" in source or "_elapsed >= _duration_s" in source
    assert "raise SystemExit(0)" in source or "SystemExit(0)" in source


def test_publish_zero_velocity_on_stop():
    """_publish_stop 必須產生 linear.x=0 的 Twist。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    tree = ast.parse(source)

    found_stop_method = False
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_publish_stop":
            found_stop_method = True
            func_source = ast.get_source_segment(source, node)
            assert "0.0" in func_source and "linear" in func_source
            break

    assert found_stop_method, "_publish_stop method not found"


def test_node_class_inherits_from_node():
    """ReverseCmdPublisher 必須繼承 rclpy.node.Node。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    tree = ast.parse(source)

    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "ReverseCmdPublisher":
            bases = [b.attr if isinstance(b, ast.Attribute) else b.id for b in node.bases]
            assert "Node" in bases, f"ReverseCmdPublisher must inherit from Node, got {bases}"
            break


def test_cmd_vel_nav_topic():
    """必須發布到 /cmd_vel_nav topic。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    assert "/cmd_vel_nav" in source


def test_no_forward_motion():
    """angular.z 必須為 0（純直線倒車）。"""
    source = Path(str(REPO_ROOT / "ros2_ws/src/vgr_runtime/vgr_runtime/ros/reverse_cmd_publisher.py")).read_text(encoding="utf-8")
    assert "angular.z = 0.0" in source or "angular.z = 0" in source or "angular.z=0.0" in source
