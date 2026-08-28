"""vgr_core.control — pipeline instrumentation and diagnostics.

Pure stdlib; no ROS, no Gazebo.
"""
from __future__ import annotations

from .diagnostics import Diagnostics, _avg, _p95

__all__ = ['Diagnostics', '_avg', '_p95']
