"""Domain models for VGR hardware protocol and safety state machines."""
from __future__ import annotations

from .command import CommandConfig, CommandGenerator, SafetyGovernor
from .models import (
    CommandDecision,
    CommandID,
    Detection,
    ErrorCode,
    MCUResponse,
    MCUState,
    MotorIntent,
    SafetyState,
)

__all__ = [
    # enums
    'CommandID',
    'SafetyState',
    'MCUState',
    'MotorIntent',
    'ErrorCode',
    # dataclasses
    'Detection',
    'CommandDecision',
    'MCUResponse',
    # command logic
    'CommandConfig',
    'CommandGenerator',
    'SafetyGovernor',
]
