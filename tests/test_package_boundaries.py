"""Package-boundary contracts for the Plan B+ cutover."""
from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCES = ROOT / "ros2_ws" / "src"


def _import_roots(package: str) -> set[str]:
    roots: set[str] = set()
    source_root = PACKAGE_SOURCES / package / package
    for path in source_root.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                roots.update(alias.name.split(".", 1)[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                roots.add(node.module.split(".", 1)[0])
    return roots


def test_vgr_core_has_no_infrastructure_imports() -> None:
    forbidden = {
        "rclpy",
        "geometry_msgs",
        "nav_msgs",
        "std_msgs",
        "tf2_ros",
        "gazebo_msgs",
        "serial",
        "phase1",
        "phase2",
        "nav2_integration",
    }
    assert _import_roots("vgr_core").isdisjoint(forbidden)


def test_vgr_driver_depends_only_on_stdlib_and_vgr_core() -> None:
    forbidden = {
        "rclpy",
        "geometry_msgs",
        "nav_msgs",
        "std_msgs",
        "tf2_ros",
        "gazebo_msgs",
        "phase1",
        "phase2",
        "nav2_integration",
    }
    assert _import_roots("vgr_driver").isdisjoint(forbidden)
