"""Migrated to vgr_core.motion. Update callers to import from vgr_core.motion instead."""
from vgr_core.motion import DifferentialOdometry, EncoderConfig, OdomState

__all__ = ['DifferentialOdometry', 'EncoderConfig', 'OdomState']
