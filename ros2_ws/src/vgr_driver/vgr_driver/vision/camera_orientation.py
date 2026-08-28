"""Single source of truth for live-camera mounting orientation.

When `CAMERA_ROTATE_180` is enabled, `upright()` rotates each live frame before
detection so downstream command and pose conventions remain unchanged.
Recorded regression footage is already upright and does not use this transform.
"""
from __future__ import annotations

import cv2
import numpy as np

# 相機是否 180° 倒放安裝。硬體改回正放時把這個改成 False —— 全 repo 唯一要動的開關。
CAMERA_ROTATE_180 = True


def upright(frame: np.ndarray, rotate_180: bool = CAMERA_ROTATE_180) -> np.ndarray:
    """相機 180° 倒放時把原始影像轉正；正放（rotate_180=False）就原樣回傳。"""
    return cv2.rotate(frame, cv2.ROTATE_180) if rotate_180 else frame
