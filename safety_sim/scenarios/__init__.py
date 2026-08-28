"""標準情境庫與註冊表。"""
from __future__ import annotations

from ..scenario import Scenario
from .advanced import (make_s4_intermittent_blackout,
                       make_s5_wheel_asymmetry_drift, make_s6_corner_approach,
                       make_s7_nominal_waypoint)
from .basic import make_s1_wall_rush, make_s2_marker_blackout, make_s3_nav_runaway
from .sapf import make_s8_single_obstacle_detour

_FACTORIES = {
    "S1": make_s1_wall_rush,
    "S2": make_s2_marker_blackout,
    "S3": make_s3_nav_runaway,
    "S4": make_s4_intermittent_blackout,
    "S5": make_s5_wheel_asymmetry_drift,
    "S6": make_s6_corner_approach,
    "S7": make_s7_nominal_waypoint,
    "S8": make_s8_single_obstacle_detour,
}


def get_scenario(name: str) -> Scenario:
    try:
        return _FACTORIES[name]()
    except KeyError:
        raise ValueError(f"unknown scenario {name!r}; available: {sorted(_FACTORIES)}") from None


def all_scenario_names() -> list[str]:
    return sorted(_FACTORIES)
