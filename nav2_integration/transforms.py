"""Migrated to vgr_core.motion. Update callers to import from vgr_core.motion instead."""
from vgr_core.motion import Pose2D, map_to_odom, wrap_angle

__all__ = ['Pose2D', 'map_to_odom', 'wrap_angle']
