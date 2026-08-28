"""安全層 filter 集合。每個方法一個模組，介面見 safety_sim.types.SafetyFilter。"""
from __future__ import annotations

from .backup_mps import BackupMpsFilter
from .cbf import CbfFilter
from .clamp_watchdog import ClampWatchdogFilter
from .geofence_vo import GeofenceVoFilter
from .gf_dwa import GfDwaFilter
from .iccbf import IccbfFilter
from .nh_vo import NhVoFilter
from .passthrough import PassthroughFilter
from .safe_apf import SafeApfFilter
from .safe_apf_new import SafeApfNewFilter

_REGISTRY = {
    "passthrough": PassthroughFilter,
    "clamp_watchdog": ClampWatchdogFilter,
    "cbf": CbfFilter,
    "backup_mps": BackupMpsFilter,
    "gf_dwa": GfDwaFilter,
    "geofence_vo": GeofenceVoFilter,
    "iccbf": IccbfFilter,
    "safe_apf": SafeApfFilter,
    "safe_apf_new": SafeApfNewFilter,
    "nh_vo": NhVoFilter,
}


def make_filter(name: str, **kwargs):
    try:
        return _REGISTRY[name](**kwargs)
    except KeyError:
        raise ValueError(f"unknown filter {name!r}; available: {sorted(_REGISTRY)}") from None


def available_filters() -> list[str]:
    return sorted(_REGISTRY)
