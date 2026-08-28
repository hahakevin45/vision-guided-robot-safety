from vgr_core.model import CommandConfig, CommandGenerator, SafetyGovernor
from vgr_core.model import CommandID, Detection, SafetyState


def _detection(x, confidence=0.5, area_ratio=0.01):
    return Detection(
        detected=True,
        frame_index=0,
        timestamp=0.0,
        center_x=x,
        center_y=0.5,
        area_ratio=area_ratio,
        confidence=confidence,
    )


def test_marker_left_maps_to_turn_left():
    generator = CommandGenerator(CommandConfig())
    command, reason = generator.from_detection(_detection(0.2))

    assert command == CommandID.TURN_LEFT
    assert reason == "target_left"


def test_marker_right_maps_to_turn_right():
    generator = CommandGenerator(CommandConfig())
    command, reason = generator.from_detection(_detection(0.8))

    assert command == CommandID.TURN_RIGHT
    assert reason == "target_right"


def test_marker_center_maps_to_forward():
    generator = CommandGenerator(CommandConfig())
    command, reason = generator.from_detection(_detection(0.5))

    assert command == CommandID.FORWARD
    assert reason == "target_centered"


def test_lost_target_maps_to_stop():
    generator = CommandGenerator(CommandConfig())
    command, reason = generator.from_detection(
        Detection(detected=False, frame_index=0, timestamp=0.0)
    )

    assert command == CommandID.STOP
    assert reason == "target_not_detected"


def test_governor_enters_safe_stop_after_target_lost_timeout():
    config = CommandConfig(target_lost_timeout_s=0.1, max_command_rate_hz=100.0)
    governor = SafetyGovernor(config)

    first = _detection(0.5)
    first = Detection(
        detected=first.detected,
        frame_index=0,
        timestamp=0.0,
        center_x=first.center_x,
        center_y=first.center_y,
        area_ratio=first.area_ratio,
        confidence=first.confidence,
    )
    decision = governor.evaluate(first, CommandID.FORWARD, "target_centered")
    assert decision.safety_state == SafetyState.TRACKING

    lost = Detection(detected=False, frame_index=10, timestamp=0.2)
    decision = governor.evaluate(lost, CommandID.STOP, "target_not_detected")

    assert decision.command == CommandID.STOP
    assert decision.safety_state == SafetyState.SAFE_STOP
    assert not decision.accepted_by_governor
