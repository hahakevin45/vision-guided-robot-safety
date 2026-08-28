"""Single-owner STM32 command and encoder session for real robot bring-up.

Portable: no rclpy, no Gazebo, no ROS message types.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from vgr_core.motion import (
    DiffDriveParams,
    DifferentialOdometry,
    EncoderConfig,
    OdomState,
    quaternion_from_yaw,
    twist_to_wheel_counts,
)
from vgr_core.model import CommandID
from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.fault_inject import FaultInjectingSerial


class BridgeProtocol(Protocol):
    def send_command(self, command: CommandID):
        """Send a command and return an exchange containing ``latency_ms``."""

    def send_set_wheel_speed(self, left_cps: int, right_cps: int):
        """Send wheel targets and return an exchange containing ``latency_ms``."""

    def read_encoders(self):
        """Return an exchange containing ``packet`` and ``latency_ms``."""


class HardwareFault(RuntimeError):
    """A latched serial failure that requires restarting the bridge process."""


@dataclass(frozen=True)
class HardwareBridgeConfig:
    allow_motion: bool = False
    cmd_timeout_s: float = 0.20
    wheel_base_m: float = 0.165
    wheel_diameter_m: float = 0.065
    left_counts_per_rev: float = 750.0
    right_counts_per_rev: float = 749.0
    left_encoder_sign: int = 1
    right_encoder_sign: int = 1
    max_counts_per_s: int = 120


@dataclass(frozen=True)
class HardwareSample:
    odom: OdomState
    raw_left: int
    raw_right: int
    left_target_cps: int
    right_target_cps: int
    command_latency_ms: float
    encoder_latency_ms: float
    fault: str | None


def _planar_covariance(
    x: float,
    y: float,
    yaw: float,
) -> list[float]:
    covariance = [0.0] * 36
    covariance[0] = x
    covariance[7] = y
    covariance[14] = 1.0e6
    covariance[21] = 1.0e6
    covariance[28] = 1.0e6
    covariance[35] = yaw
    return covariance


def odom_payload(state: OdomState) -> dict[str, object]:
    """Convert OdomState into ROS-independent message field values."""
    qx, qy, qz, qw = quaternion_from_yaw(state.theta)
    return {
        "frame_id": "odom",
        "child_frame_id": "base_link",
        "position": {"x": state.x, "y": state.y},
        "quaternion": (qx, qy, qz, qw),
        "twist": {
            "linear_x": state.linear_mps,
            "angular_z": state.angular_rad_s,
        },
        "pose_covariance": _planar_covariance(0.0004, 0.0004, 0.0025),
        "twist_covariance": _planar_covariance(0.0025, 1.0e6, 0.01),
    }


class HardwareBridgeSession:
    """ROS-independent, serialized owner of command and encoder exchanges."""

    def __init__(
        self,
        bridge: ControllerBridge | BridgeProtocol,
        config: HardwareBridgeConfig,
    ) -> None:
        self.bridge = bridge
        self.config = config
        self._odom = DifferentialOdometry(EncoderConfig(
            wheel_base_m=config.wheel_base_m,
            wheel_diameter_m=config.wheel_diameter_m,
            left_counts_per_rev=config.left_counts_per_rev,
            right_counts_per_rev=config.right_counts_per_rev,
            left_sign=config.left_encoder_sign,
            right_sign=config.right_encoder_sign,
        ))
        self._drive_params = DiffDriveParams(
            wheel_base_m=config.wheel_base_m,
            wheel_diameter_m=config.wheel_diameter_m,
            left_counts_per_rev=config.left_counts_per_rev,
            right_counts_per_rev=config.right_counts_per_rev,
            max_counts_per_s=config.max_counts_per_s,
        )
        self._linear_mps = 0.0
        self._angular_rad_s = 0.0
        self._last_cmd_stamp_s: float | None = None
        self._fault: str | None = None
        self._stopped = False

    @property
    def fault(self) -> str | None:
        return self._fault

    def startup(self, stamp_s: float) -> HardwareSample:
        command = self.bridge.send_command(CommandID.STOP)
        encoders = self.bridge.read_encoders()
        state = self._odom.update(
            encoders.packet.left_count,
            encoders.packet.right_count,
            stamp_s,
        )
        return HardwareSample(
            odom=state,
            raw_left=encoders.packet.left_count,
            raw_right=encoders.packet.right_count,
            left_target_cps=0,
            right_target_cps=0,
            command_latency_ms=command.latency_ms,
            encoder_latency_ms=encoders.latency_ms,
            fault=None,
        )

    def update_command(
        self,
        linear_mps: float,
        angular_rad_s: float,
        *,
        stamp_s: float,
    ) -> None:
        self._linear_mps = linear_mps
        self._angular_rad_s = angular_rad_s
        self._last_cmd_stamp_s = stamp_s

    def cycle(self, stamp_s: float) -> HardwareSample:
        if self._fault is not None:
            raise HardwareFault(f"hardware fault latched: {self._fault}")
        left_cps, right_cps = self._targets(stamp_s)
        try:
            if left_cps == 0 and right_cps == 0:
                command = self.bridge.send_command(CommandID.STOP)
            else:
                command = self.bridge.send_set_wheel_speed(left_cps, right_cps)
            encoders = self.bridge.read_encoders()
            state = self._odom.update(
                encoders.packet.left_count,
                encoders.packet.right_count,
                stamp_s,
            )
            return HardwareSample(
                odom=state,
                raw_left=encoders.packet.left_count,
                raw_right=encoders.packet.right_count,
                left_target_cps=left_cps,
                right_target_cps=right_cps,
                command_latency_ms=command.latency_ms,
                encoder_latency_ms=encoders.latency_ms,
                fault=None,
            )
        except Exception as exc:
            self._fault = f"{type(exc).__name__}: {exc}"
            try:
                self.bridge.send_command(CommandID.STOP)
            except Exception:
                pass
            raise HardwareFault(self._fault) from exc

    def stop(self) -> None:
        if self._stopped:
            return
        self.bridge.send_command(CommandID.STOP)
        self._stopped = True

    def _targets(self, stamp_s: float) -> tuple[int, int]:
        if not self.config.allow_motion or self._last_cmd_stamp_s is None:
            return (0, 0)
        if stamp_s - self._last_cmd_stamp_s > self.config.cmd_timeout_s:
            return (0, 0)
        return twist_to_wheel_counts(
            self._linear_mps,
            self._angular_rad_s,
            self._drive_params,
        )
