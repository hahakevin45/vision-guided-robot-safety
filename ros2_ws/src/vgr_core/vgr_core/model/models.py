from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from time import monotonic
from typing import Optional


class CommandID(IntEnum):
    """Host 端送往 MCU 的高階命令 ID。

    使用 IntEnum 是因為這些值最後要被放進二進位封包，並和 STM32 韌體端的
    C enum 對齊。程式內使用 `CommandID.FORWARD` 保持可讀性，封包內使用
    `int(CommandID.FORWARD)` 取得實際 byte value。
    """

    STOP = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    FORWARD = 3
    HEARTBEAT = 4
    READ_ENCODERS = 5
    SET_WHEEL_SPEED = 6


class SafetyState(IntEnum):
    """Host 端 safety governor 的安全狀態。"""

    IDLE = 0
    TRACKING = 1
    SAFE_STOP = 2
    FAULT = 3


class MCUState(IntEnum):
    """MCU 回報給 host 的控制板狀態。"""

    IDLE = 0
    ARMED = 1
    TRACKING = 2
    SAFE_STOP = 3
    FAULT = 4


class MotorIntent(IntEnum):
    """MCU 目前打算輸出的馬達動作；Phase 2.5 先作 dry-run telemetry。"""

    STOP = 0
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3


class ErrorCode(IntEnum):
    """MCU 或 mock MCU 回報的錯誤碼。"""

    OK = 0
    BAD_HEADER = 1
    BAD_CHECKSUM = 2
    BAD_SEQUENCE = 3
    INVALID_COMMAND = 4
    COMMAND_TIMEOUT = 5
    FORCED_FAULT = 6


@dataclass(frozen=True)
class Detection:
    """單一影像幀的視覺偵測結果。

    center_x / center_y 使用 0.0 到 1.0 的正規化座標，讓後續控制邏輯不依賴
    影像解析度。area_ratio 代表 marker 面積佔整張影像的比例，可用來粗略判斷
    目標是否太近。
    """

    detected: bool
    frame_index: int
    timestamp: float
    center_x: Optional[float] = None
    center_y: Optional[float] = None
    area_ratio: float = 0.0
    confidence: float = 0.0
    marker_id: Optional[int] = None
    latency_ms: float = 0.0


@dataclass(frozen=True)
class CommandDecision:
    """Safety governor 對某一幀 proposed command 的判斷結果。"""

    command: CommandID
    safety_state: SafetyState
    reason: str
    accepted_by_governor: bool
    timestamp: float = field(default_factory=monotonic)


@dataclass(frozen=True)
class MCUResponse:
    """MCU 接收命令後回傳給 pipeline 的抽象回應。"""

    state: MCUState
    error: ErrorCode
    sequence: int
    accepted: bool
    message: str
    latency_ms: float
    motor_intent: MotorIntent = MotorIntent.STOP
