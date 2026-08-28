from __future__ import annotations

import argparse
import json
import os
import pty
import time
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver.serial_transport import PosixSerial


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Certify STM32 encoder snapshot telemetry."
    )
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--spin-s", type=float, default=0.5)
    parser.add_argument("--require-count-change", action="store_true")
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/encoder_signal_certification.json"),
    )
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
        "require_count_change": args.require_count_change,
        "snapshots": [],
        "checks": {},
        "error": None,
    }

    try:
        with PosixSerial(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            if not using_pty and args.settle_s > 0:
                time.sleep(args.settle_s)
                serial.flush_input()
            bridge = ControllerBridge(serial)
            heartbeat = bridge.send_command(CommandID.HEARTBEAT)
            first = bridge.read_encoders()
            result["snapshots"].append(_snapshot_result("initial", first))
            forward = None
            second = None
            try:
                if args.require_count_change:
                    forward = bridge.send_command(CommandID.FORWARD)
                    time.sleep(args.spin_s)
                    second = bridge.read_encoders()
                    result["snapshots"].append(_snapshot_result("after_spin", second))
            finally:
                stop = bridge.send_command(CommandID.STOP)

        state_ok = (
            heartbeat.state.error == ErrorCode.OK
            and stop.state.error == ErrorCode.OK
            and (forward is None or forward.state.error == ErrorCode.OK)
        )
        sequence_ok = all(
            item["sequence"] == item["packet_sequence"] for item in result["snapshots"]
        )
        read_snapshot = bool(result["snapshots"])
        count_changed = True
        if args.require_count_change and second is not None:
            count_changed = (
                first.packet.left_count != second.packet.left_count
                or first.packet.right_count != second.packet.right_count
            )
        result["checks"] = {
            "opened_serial_device": True,
            "heartbeat_and_stop_ok": state_ok,
            "read_encoder_snapshot": read_snapshot,
            "sequence_echo_ok": sequence_ok,
            "count_changed_if_required": count_changed,
        }
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - CLI must report device failures clearly.
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
        print("ENCODER SIGNAL CERTIFICATION: PASS")
        return 0
    print("ENCODER SIGNAL CERTIFICATION: FAIL")
    return 1


def _snapshot_result(label: str, exchange) -> dict[str, object]:
    return {
        "label": label,
        "sequence": exchange.sequence,
        "packet_sequence": exchange.packet.sequence,
        "left_count": exchange.packet.left_count,
        "right_count": exchange.packet.right_count,
        "flags": exchange.packet.flags,
        "latency_ms": exchange.latency_ms,
    }


if __name__ == "__main__":
    raise SystemExit(main())
