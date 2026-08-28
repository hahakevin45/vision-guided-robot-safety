"""Hardware driver session layer for VGR.

Exports the portable session classes: PosixSerial, ControllerBridge,
HardwareBridgeSession, HardwareBridgeConfig, HardwareSample, FaultInjectingSerial,
MockMCU, MockController, SerialController, Controller.
No ROS or Gazebo dependencies.
"""
from __future__ import annotations

from .controller_bridge import BridgeExchange, ControllerBridge, EncoderExchange
from .controllers import Controller, MockController, SerialController
from .hardware_bridge import (
    BridgeProtocol,
    HardwareBridgeConfig,
    HardwareBridgeSession,
    HardwareSample,
    HardwareFault,
    odom_payload,
)
from .fault_inject import FaultInjectingSerial
from .mock_mcu import MockMCU
from .serial_transport import BAUD_RATES, PosixSerial, SerialStats

__all__ = [
    # serial
    'BAUD_RATES',
    'PosixSerial',
    'SerialStats',
    # bridge
    'BridgeExchange',
    'BridgeProtocol',
    'ControllerBridge',
    'EncoderExchange',
    # session
    'FaultInjectingSerial',
    'HardwareBridgeConfig',
    'HardwareBridgeSession',
    'HardwareSample',
    'HardwareFault',
    'odom_payload',
    # controller
    'Controller',
    'MockController',
    'MockMCU',
    'SerialController',
]
