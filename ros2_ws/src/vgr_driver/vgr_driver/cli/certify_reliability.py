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


COMMAND_PATTERN = [
    CommandID.FORWARD,
    CommandID.TURN_LEFT,
    CommandID.TURN_RIGHT,
    CommandID.STOP,
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Certify Phase 2 serial reliability without motors.")
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--report", type=Path, default=Path("outputs/phase2_reliability_report.json"))
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
        "cycles": args.cycles,
        "checks": {},
        "resync": [],
        "soak": {},
        "error": None,
    }

    try:
        first = run_session(device, args, cycles=args.cycles)
        time.sleep(0.1)
        second = run_session(device, args, cycles=4)
        result["resync"] = [first["resync"], second["resync"]]
        result["soak"] = first["soak"]

        result["checks"] = {
            "first_resync_ok": first["resync"]["mcu_error"] == ErrorCode.OK.name,
            "second_resync_after_reopen_ok": second["resync"]["mcu_error"] == ErrorCode.OK.name,
            "soak_all_errors_ok": first["soak"]["mcu_errors_ok"],
            "soak_sequence_echo_ok": first["soak"]["sequence_echo_ok"],
            "soak_completed_cycles": first["soak"]["completed_cycles"] == args.cycles,
        }
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - hardware certification preserves failure detail.
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
    print(json.dumps({"pass": result["pass"], "checks": result["checks"], "soak": result["soak"]}, indent=2))
    if result["pass"]:
        print("PHASE 2 RELIABILITY: PASS")
        return 0
    print("PHASE 2 RELIABILITY: FAIL")
    return 1


def run_session(device: str, args: argparse.Namespace, cycles: int) -> dict:
    """開啟一次 serial session，先 resync，再連續送固定命令序列。"""

    with PosixSerial(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
        if args.settle_s > 0:
            time.sleep(args.settle_s if cycles == args.cycles else 0.2)
            serial.flush_input()
        bridge = ControllerBridge(serial)
        resync = bridge.send_command(CommandID.HEARTBEAT)
        exchanges = []
        for index in range(cycles):
            command = COMMAND_PATTERN[index % len(COMMAND_PATTERN)]
            exchange = bridge.send_command(command)
            exchanges.append(
                {
                    "cycle": index,
                    "command": command.name,
                    "sequence": exchange.sequence,
                    "state_sequence": exchange.state.sequence,
                    "mcu_state": exchange.state.state.name,
                    "mcu_error": exchange.state.error.name,
                    "latency_ms": exchange.latency_ms,
                }
            )
    return {
        "resync": {
            "sequence": resync.sequence,
            "state_sequence": resync.state.sequence,
            "mcu_state": resync.state.state.name,
            "mcu_error": resync.state.error.name,
            "latency_ms": resync.latency_ms,
        },
        "soak": summarize_soak(exchanges),
        "exchanges": exchanges,
    }


def summarize_soak(exchanges: list[dict]) -> dict:
    """整理 soak test 的錯誤、sequence echo 與 latency 統計。"""

    latencies = [float(exchange["latency_ms"]) for exchange in exchanges]
    return {
        "completed_cycles": len(exchanges),
        "mcu_errors_ok": all(exchange["mcu_error"] == ErrorCode.OK.name for exchange in exchanges),
        "sequence_echo_ok": all(exchange["sequence"] == exchange["state_sequence"] for exchange in exchanges),
        "avg_latency_ms": sum(latencies) / len(latencies) if latencies else 0.0,
        "max_latency_ms": max(latencies) if latencies else 0.0,
        "commands": exchanges,
    }


if __name__ == "__main__":
    raise SystemExit(main())
