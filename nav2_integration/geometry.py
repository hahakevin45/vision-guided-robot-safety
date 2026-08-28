"""Migrated to vgr_core.geometry. Update callers to import from vgr_core.geometry instead."""
from vgr_core.geometry import (
    ARENA_BOUNDS,
    MAP_PADDING_M,
    MAP_RESOLUTION_M,
    NAV_OBSTACLE,
    Box2D,
    OccupancyGrid,
    build_occupancy_grid,
)

__all__ = [
    'ARENA_BOUNDS',
    'MAP_RESOLUTION_M',
    'MAP_PADDING_M',
    'NAV_OBSTACLE',
    'Box2D',
    'OccupancyGrid',
    'build_occupancy_grid',
]
