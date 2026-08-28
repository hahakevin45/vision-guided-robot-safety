from __future__ import annotations

from dataclasses import dataclass

from .models import CommandDecision, CommandID, Detection, SafetyState


@dataclass(frozen=True)
class CommandConfig:
    """視覺命令與安全邏輯的主要門檻值。"""

    left_threshold: float = 0.38
    right_threshold: float = 0.62
    min_confidence: float = 0.05
    close_area_ratio: float = 0.045
    target_lost_timeout_s: float = 0.35
    max_command_rate_hz: float = 15.0


class CommandGenerator:
    def __init__(self, config: CommandConfig) -> None:
        self.config = config

    def from_detection(self, detection: Detection) -> tuple[CommandID, str]:
        """把單一幀的偵測結果轉成高階移動命令。

        這裡只負責「視覺意圖」：目標在左邊就左轉、右邊就右轉、
        中間就前進。真正能不能送到控制板，交給 SafetyGovernor 判斷。
        """

        if not detection.detected:
            return CommandID.STOP, "target_not_detected"
        if detection.confidence < self.config.min_confidence:
            return CommandID.STOP, "low_confidence"
        if detection.area_ratio >= self.config.close_area_ratio:
            return CommandID.STOP, "target_too_close"
        if detection.center_x is None:
            return CommandID.STOP, "missing_center"
        if detection.center_x < self.config.left_threshold:
            return CommandID.TURN_LEFT, "target_left"
        if detection.center_x > self.config.right_threshold:
            return CommandID.TURN_RIGHT, "target_right"
        return CommandID.FORWARD, "target_centered"


class SafetyGovernor:
    def __init__(self, config: CommandConfig) -> None:
        self.config = config
        self._last_detection_ts: float | None = None
        self._last_emit_ts: float | None = None
        self._safe_state = SafetyState.IDLE

    def evaluate(self, detection: Detection, proposed: CommandID, reason: str) -> CommandDecision:
        """檢查高階命令是否安全，決定是否真正送往 MCU。

        governor 是高階視覺和底層控制板之間的安全邊界：
        即使 vision 產生了命令，只要目標長時間消失、可信度不足、
        或命令頻率太高，都會被限制或轉成 SAFE_STOP。
        """

        now = detection.timestamp
        if detection.detected and detection.confidence >= self.config.min_confidence:
            self._last_detection_ts = now

        # 系統啟動後尚未看過可信目標時，預設停在安全狀態。
        if self._last_detection_ts is None:
            return self._decision(CommandID.STOP, SafetyState.SAFE_STOP, "no_valid_target_seen", False, now)

        lost_for = now - self._last_detection_ts
        # 目標消失超過門檻後，直接切到 SAFE_STOP；這是視覺系統失效時的 fail-safe。
        if lost_for > self.config.target_lost_timeout_s:
            return self._decision(
                CommandID.STOP,
                SafetyState.SAFE_STOP,
                f"target_lost_timeout_{lost_for:.3f}s",
                False,
                now,
            )

        min_period = 1.0 / self.config.max_command_rate_hz
        # 限制送往 MCU 的命令頻率，避免視覺幀率抖動造成控制板收到過密命令。
        if self._last_emit_ts is not None and now - self._last_emit_ts < min_period:
            return self._decision(proposed, self._safe_state, "command_rate_limited", False, now)

        if proposed == CommandID.STOP:
            state = SafetyState.SAFE_STOP
        else:
            state = SafetyState.TRACKING

        return self._decision(proposed, state, reason, True, now)

    def _decision(
        self,
        command: CommandID,
        state: SafetyState,
        reason: str,
        accepted: bool,
        now: float,
    ) -> CommandDecision:
        if accepted:
            self._last_emit_ts = now
        self._safe_state = state
        return CommandDecision(
            command=command,
            safety_state=state,
            reason=reason,
            accepted_by_governor=accepted,
            timestamp=now,
        )
