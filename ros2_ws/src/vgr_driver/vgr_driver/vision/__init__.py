"""vgr_driver.vision — OpenCV-based ArUco detection and camera utilities.

Requires OpenCV (cv2) at runtime; no ROS/Gazebo dependencies.
"""
from __future__ import annotations

from .aruco import ArucoDetector, detect_markers, draw_detection_overlay
from .camera_orientation import CAMERA_ROTATE_180, upright

__all__ = ['ArucoDetector', 'CAMERA_ROTATE_180', 'detect_markers', 'draw_detection_overlay', 'upright']
