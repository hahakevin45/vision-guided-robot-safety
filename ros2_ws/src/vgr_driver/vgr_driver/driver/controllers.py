"""Controller abstraction for the VGR hardware pipeline."""
from __future__ import annotations

from vgr_core.model import CommandID, MCUResponse
from vgr_core.protocol import CommandPacket, encode_command
from .controller_bridge import ControllerBridge
from .mock_mcu import MockMCU
from .serial_transport import PosixSerial


class Controller:
    """Pipeline's abstraction over the control board.

    Phase 1 uses MockController; Phase 2 switches to SerialController,
    so vision and safety logic don't need to know whether the underlying
    MCU is simulated or real.
    """

    def send(self, command: CommandID, timestamp: float | None = None) -> MCUResponse:
        ...

    def tick(self, timestamp: float | None = None) -> MCUResponse | None:
        ...

    def close(self) -> None:
        ...

    def resync(self) -> MCUResponse | None:
        ...


class MockController:
    """MCU stand-in for pre-hardware development."""

    def __init__(self, command_timeout_s: float) -> None:
        self.mcu = MockMCU(command_timeout_s=command_timeout_s)
        self.sequence = 0

    def send(self, command: CommandID, timestamp: float | None = None) -> MCUResponse:
        packet = CommandPacket(sequence=self.sequence, command=command)
        response = self.mcu.receive(encode_command(packet), timestamp=timestamp)
        self.sequence = (self.sequence + 1) & 0xFF
        return response

    def tick(self, timestamp: float | None = None) -> MCUResponse | None:
        return self.mcu.tick(timestamp=timestamp)

    def close(self) -> None:
        return None

    def resync(self) -> MCUResponse | None:
        return self.send(CommandID.HEARTBEAT)


class SerialController:
    """Sends commands to real STM32 via serial bridge."""

    def __init__(
        self,
        device: str,
        baudrate: int = 115200,
        timeout_s: float = 0.5,
    ) -> None:
        self.serial = PosixSerial(device=device, baudrate=baudrate, timeout_s=timeout_s)
        self.serial.open()
        self.bridge = ControllerBridge(self.serial)

    def send(self, command: CommandID, timestamp: float | None = None) -> MCUResponse:
        del timestamp
        exchange = self.bridge.send_command(command)
        from vgr_core.model import ErrorCode
        accepted = exchange.state.error == ErrorCode.OK
        return MCUResponse(
            state=exchange.state.state,
            error=exchange.state.error,
            sequence=exchange.state.sequence,
            accepted=accepted,
            message="accepted" if accepted else exchange.state.error.name.lower(),
            latency_ms=exchange.latency_ms,
            motor_intent=exchange.state.motor_intent,
        )

    def tick(self, timestamp: float | None = None) -> MCUResponse | None:
        del timestamp
        return None

    def close(self) -> None:
        self.serial.close()

    def resync(self) -> MCUResponse | None:
        return self.send(CommandID.HEARTBEAT)
