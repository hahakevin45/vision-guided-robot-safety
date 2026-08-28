from vgr_core.model import CommandID
from vgr_core.protocol import CommandPacket, decode_command, encode_command, encode_set_wheel_speed
from vgr_driver.driver import FaultInjectingSerial


class RecordingSerial:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))


def test_fault_injector_none_preserves_command_bytes():
    serial = RecordingSerial()
    injector = FaultInjectingSerial(serial, mode="none", at_s=-1, count=10)
    raw = encode_set_wheel_speed(3, 42, 42)

    injector.write(raw, now_s=100.0)

    assert serial.writes == [raw]


def test_fault_injector_bad_checksum_corrupts_only_requested_actuator_frames():
    serial = RecordingSerial()
    injector = FaultInjectingSerial(serial, mode="bad_checksum", at_s=5.0, count=2)
    raw = encode_set_wheel_speed(3, 42, 42)
    read = encode_command(CommandPacket(sequence=4, command=CommandID.READ_ENCODERS))

    injector.write(raw, now_s=4.9)
    injector.write(raw, now_s=5.0)
    injector.write(raw, now_s=5.1)
    injector.write(raw, now_s=5.2)
    injector.write(read, now_s=5.3)

    assert serial.writes[0] == raw
    assert serial.writes[1][:-1] == raw[:-1]
    assert serial.writes[1][-1] != raw[-1]
    assert serial.writes[2][-1] != raw[-1]
    assert serial.writes[3] == raw
    assert serial.writes[4] == read
    for corrupted in serial.writes[1:3]:
        try:
            decode_command(corrupted)
        except ValueError as exc:
            assert "checksum" in str(exc)
        else:
            raise AssertionError("injected packet unexpectedly decoded")


def test_fault_injector_garbage_keeps_frame_length_and_uses_requested_window():
    serial = RecordingSerial()
    injector = FaultInjectingSerial(serial, mode="garbage", at_s=0.0, count=2)
    raw = encode_set_wheel_speed(3, 42, 42)

    injector.write(raw, now_s=0.0)
    injector.write(raw, now_s=0.1)
    injector.write(raw, now_s=0.2)

    assert len(serial.writes[0]) == len(raw)
    assert len(serial.writes[1]) == len(raw)
    assert serial.writes[0] != raw
    assert serial.writes[1] != raw
    assert serial.writes[2] == raw
