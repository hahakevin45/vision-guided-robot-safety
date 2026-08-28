"""ArUco marker detection for vision-guided robotics.

Detects a single ArUco marker and outputs normalized image coordinates.
"""
from __future__ import annotations

from time import monotonic

import cv2
import numpy as np

from vgr_core.model import Detection


def detect_markers(gray: np.ndarray, dictionary, parameters):
    """Run ArUco detection across OpenCV 4.5 through 5.x.

    OpenCV 5 removed the ``cv2.aruco.detectMarkers`` free function; the
    ``ArucoDetector`` class that replaces it exists from 4.7 onward.

    Returns ``(corners, ids, rejected)`` with ``ids`` normalized to shape
    ``(N, 1)`` — OpenCV 4 returns ``(N, 1)`` but OpenCV 5 returns ``(N,)``,
    and callers here index it as ``ids[i][0]``.
    """
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


class ArucoDetector:
    """Detects a single ArUco marker and outputs normalized image coordinates."""

    def __init__(self, dictionary_name: str = "DICT_6X6_250") -> None:
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}")
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = self._create_dictionary(dictionary_id)
        self.parameters = self._create_detector_parameters()

    def detect(
        self, frame: np.ndarray, frame_index: int, timestamp: float | None = None
    ) -> Detection:
        """Process a single OpenCV frame and return a Detection.

        If timestamp is provided it is used for the event timeline;
        otherwise monotonic clock is used so the same detector works
        with both recorded video and live cameras.
        """
        start = monotonic()
        event_ts = monotonic() if timestamp is None else timestamp
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect_markers(gray, self.dictionary, self.parameters)
        latency_ms = (monotonic() - start) * 1000.0

        if ids is None or len(ids) == 0:
            return Detection(
                detected=False,
                frame_index=frame_index,
                timestamp=event_ts,
                latency_ms=latency_ms,
            )

        h, w = frame.shape[:2]
        best_idx = self._largest_marker_index(corners)
        pts = corners[best_idx][0]
        center = pts.mean(axis=0)
        area = cv2.contourArea(pts.astype(np.float32))
        frame_area = float(w * h)
        area_ratio = max(0.0, min(1.0, area / frame_area))
        confidence = max(0.0, min(1.0, area_ratio * 250.0))

        return Detection(
            detected=True,
            frame_index=frame_index,
            timestamp=event_ts,
            center_x=float(center[0] / w),
            center_y=float(center[1] / h),
            area_ratio=area_ratio,
            confidence=confidence,
            marker_id=int(ids[best_idx][0]),
            latency_ms=latency_ms,
        )

    @staticmethod
    def _largest_marker_index(corners: tuple[np.ndarray, ...] | list[np.ndarray]) -> int:
        """Return index of the largest-area marker (closest/most prominent)."""
        areas = [cv2.contourArea(corner[0].astype(np.float32)) for corner in corners]
        return int(np.argmax(areas))

    @staticmethod
    def _create_dictionary(dictionary_id: int):
        if hasattr(cv2.aruco, "Dictionary_get"):
            return cv2.aruco.Dictionary_get(dictionary_id)
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    @staticmethod
    def _create_detector_parameters():
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            return cv2.aruco.DetectorParameters_create()
        return cv2.aruco.DetectorParameters()


def draw_detection_overlay(
    frame: np.ndarray,
    detection: Detection,
    command_text: str,
    safety_text: str,
) -> np.ndarray:
    """Produce a debug video overlay for a detection."""
    output = frame.copy()
    h, w = output.shape[:2]
    left_x = int(w * 0.38)
    right_x = int(w * 0.62)
    cv2.line(output, (left_x, 0), (left_x, h), (255, 200, 0), 2)
    cv2.line(output, (right_x, 0), (right_x, h), (255, 200, 0), 2)

    if detection.detected and detection.center_x is not None and detection.center_y is not None:
        cx = int(detection.center_x * w)
        cy = int(detection.center_y * h)
        cv2.drawMarker(output, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, thickness=3)
        cv2.circle(output, (cx, cy), 14, (0, 255, 0), 2)

    lines = [
        f"command: {command_text}",
        f"safety: {safety_text}",
        f"detected: {detection.detected}",
        f"latency: {detection.latency_ms:.2f} ms",
    ]
    for idx, line in enumerate(lines):
        y = 36 + idx * 32
        cv2.putText(output, line, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 5)
        cv2.putText(output, line, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
    return output
