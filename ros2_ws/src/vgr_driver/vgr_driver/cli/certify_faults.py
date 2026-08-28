from __future__ import annotations

import argparse
import json
import os
import pty
import time
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode
from vgr_core.protocol import CommandPacket, encode_command

from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver.serial_transport import PosixSerial
from vgr_core.protocol import STATE_PACKET_LEN, decode_state


def main() -> int:
    parser = argparse.ArgumentParser(description="Certify STM32 serial fault handling.")
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--report", type=Path, default=Path("outputs/serial_fault_certification.json"))
    args = parser.parse_args()

    master_fd = None
    slave_fd = None
    mock_mcu = None
    device = args.device
    using_pty = device is None
    if using_pty:
        master_fd, slave_fd = pty.openpty()
        device = os.ttyname(slave_fd)
        mock_mcu = MockSerialMCU(master_fd, timeout_s=args.timeout_s)
        mock_mcu.start()

    result = {
        "pass": False,
        "device": device,
        "using_pty_mock_mcu": using_pty,
        "faults": [],
        "checks": {},
        "error": None,
    }

    try:
        with PosixSerial(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            if not using_pty and args.settle_s > 0:
                time.sleep(args.settle_s)
                serial.flush_input()

            result["faults"].append(run_bad_checksum(serial))
            result["faults"].append(run_sequence_gap(serial))

        result["checks"] = {
            "bad_checksum_rejected": _fault_ok(result["faults"], "bad_checksum", "BAD_CHECKSUM"),
            "sequence_gap_rejected": _fault_ok(result["faults"], "sequence_gap", "BAD_SEQUENCE"),
        }
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - hardware certification preserves failures.
        result["error"] = str(exc)
    finally:
        if mock_mcu is not None:
            mock_mcu.stop()
        if master_fd is not None:
            os.close(master_fd)
        if slave_fd is not None:
            os.close(slave_fd)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if result["pass"]:
        print("SERIAL FAULT CERTIFICATION: PASS")
        return 0
    print("SERIAL FAULT CERTIFICATION: FAIL")
    return 1


def run_bad_checksum(serial: PosixSerial) -> dict:
    """故意破壞 checksum，驗證 STM32 會拒絕損壞封包。"""

    raw = bytearray(encode_command(CommandPacket(sequence=7, command=CommandID.FORWARD)))
    raw[-1] ^= 0xFF
    serial.write(bytes(raw))
    state = decode_state(serial.read_exact(STATE_PACKET_LEN))
    return {
        "name": "bad_checksum",
        "tx_hex": bytes(raw).hex(" "),
        "state_sequence": state.sequence,
        "mcu_state": state.state.name,
        "mcu_error": state.error.name,
        "expected_error": ErrorCode.BAD_CHECKSUM.name,
        "pass": state.error == ErrorCode.BAD_CHECKSUM,
    }


def run_sequence_gap(serial: PosixSerial) -> dict:
    """故意跳過 sequence，驗證 STM32 會偵測 host/serial 傳輸不同步。"""

    heartbeat = encode_command(CommandPacket(sequence=0, command=CommandID.HEARTBEAT))
    serial.write(heartbeat)
    heartbeat_state = decode_state(serial.read_exact(STATE_PACKET_LEN))

    gap = encode_command(CommandPacket(sequence=2, command=CommandID.FORWARD))
    serial.write(gap)
    gap_state = decode_state(serial.read_exact(STATE_PACKET_LEN))
    return {
        "name": "sequence_gap",
        "resync": {
            "tx_hex": heartbeat.hex(" "),
            "state_sequence": heartbeat_state.sequence,
            "mcu_state": heartbeat_state.state.name,
            "mcu_error": heartbeat_state.error.name,
        },
        "tx_hex": gap.hex(" "),
        "state_sequence": gap_state.sequence,
        "mcu_state": gap_state.state.name,
        "mcu_error": gap_state.error.name,
        "expected_error": ErrorCode.BAD_SEQUENCE.name,
        "pass": gap_state.error == ErrorCode.BAD_SEQUENCE,
    }


def _fault_ok(faults: list[dict], name: str, expected_error: str) -> bool:
    return any(
        fault.get("name") == name
        and fault.get("pass") is True
        and fault.get("mcu_error") == expected_error
        for fault in faults
    )


if __name__ == "__main__":
    raise SystemExit(main())
