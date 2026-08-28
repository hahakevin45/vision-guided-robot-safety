from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_firmware_state_packet_contract_includes_motor_intent():
    header = (ROOT / "firmware/common/protocol.h").read_text()

    assert "#define VGR_STATE_PACKET_LEN 10u" in header
    assert "vgr_motor_intent_t" in header
    assert "motor_intent;" in header


def test_firmware_encoder_packet_contract_is_separate_from_state_packet():
    header = (ROOT / "firmware/common/protocol.h").read_text()

    assert "#define VGR_ENCODER_PACKET_TYPE 0x81u" in header
    assert "#define VGR_ENCODER_PACKET_LEN 14u" in header
    assert "VGR_CMD_READ_ENCODERS = 5" in header
    assert "int32_t left_count;" in header
    assert "int32_t right_count;" in header


def test_firmware_set_wheel_speed_contract():
    header = (ROOT / "firmware/common/protocol.h").read_text()

    assert "VGR_CMD_SET_WHEEL_SPEED = 6" in header
    assert "#define VGR_SET_WHEEL_SPEED_PACKET_LEN 10u" in header
    assert "int16_t left_counts_per_s;" in header
