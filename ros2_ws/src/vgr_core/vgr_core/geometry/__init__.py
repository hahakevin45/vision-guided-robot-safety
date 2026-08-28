"""Arena geometry, geofence and occupancy grid utilities."""
from __future__ import annotations

from .arena_geometry import (
    ARENA,
    ARENA_BOUNDS,
    MAP_PADDING_M,
    MAP_RESOLUTION_M,
    NAV_OBSTACLE,
    Box2D,
    OccupancyGrid,
    build_occupancy_grid,
)

__all__ = [
    'ARENA',
    'ARENA_BOUNDS',
    'MAP_RESOLUTION_M',
    'MAP_PADDING_M',
    'NAV_OBSTACLE',
    'Box2D',
    'OccupancyGrid',
    'build_occupancy_grid',
]
