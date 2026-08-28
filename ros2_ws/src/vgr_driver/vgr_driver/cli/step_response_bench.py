"""Raised-wheel speed-step response for motor time-constant estimation.

The sequence records a STOP baseline, repeated `SET_WHEEL_SPEED` commands, and
post-STOP coast decay through `READ_ENCODERS`. Wheels must remain raised.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vgr_core.model import CommandID

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.serial_transport import PosixSerial


def _sample(bridge: ControllerBridge, t0: float, samples: list[dict]) -> None:
    exchange = bridge.read_encoders()
    samples.append({
        "t": time.monotonic() - t0,
        "left": exchange.packet.left_count,
        "right": exchange.packet.right_count,
        "latency_ms": exchange.latency_ms,
    })


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--target-counts-per-s", type=int, default=600)
    parser.add_argument("--baseline-s", type=float, default=1.0)
    parser.add_argument("--step-s", type=float, default=3.0)
    parser.add_argument("--coast-s", type=float, default=3.0)
    parser.add_argument("--sample-interval-s", type=float, default=0.02)
    parser.add_argument("--command-interval-s", type=float, default=0.1,
                        help="步階期間重送命令的間隔，必須 << firmware watchdog 500ms")
    parser.add_argument("--report", type=Path,
                        default=Path("outputs/step_response_bench.json"))
    args = parser.parse_args()

    result: dict[str, object] = {
        "pass": False,
        "device": args.device,
        "target_counts_per_s": args.target_counts_per_s,
        "phases": {"baseline_s": args.baseline_s, "step_s": args.step_s,
                   "coast_s": args.coast_s},
        "samples": [],
        "error": None,
    }
    samples: list[dict] = []

    try:
        with PosixSerial(device=args.device, baudrate=args.baudrate,
                         timeout_s=args.timeout_s) as serial:
            time.sleep(args.settle_s)
            serial.flush_input()
            bridge = ControllerBridge(serial)
            bridge.send_command(CommandID.HEARTBEAT)
            bridge.send_command(CommandID.STOP)

            t0 = time.monotonic()

            # 基線（靜止）
            while time.monotonic() - t0 < args.baseline_s:
                _sample(bridge, t0, samples)
                time.sleep(args.sample_interval_s)

            # 步階：持續重送 SET_WHEEL_SPEED（躲 watchdog），高頻取樣
            step_start = time.monotonic() - t0
            result["step_start_t"] = step_start
            last_cmd = 0.0
            while time.monotonic() - t0 < args.baseline_s + args.step_s:
                now = time.monotonic()
                if now - last_cmd >= args.command_interval_s:
                    bridge.send_set_wheel_speed(args.target_counts_per_s,
                                                args.target_counts_per_s)
                    last_cmd = now
                _sample(bridge, t0, samples)
                time.sleep(args.sample_interval_s)

            # STOP + coast 衰減
            stop_t = time.monotonic() - t0
            result["stop_t"] = stop_t
            bridge.send_command(CommandID.STOP)
            while time.monotonic() - t0 < args.baseline_s + args.step_s + args.coast_s:
                _sample(bridge, t0, samples)
                time.sleep(args.sample_interval_s)

            bridge.send_command(CommandID.STOP)
            result["samples"] = samples
            result["pass"] = len(samples) > 50
    except Exception as exc:  # noqa: BLE001 - 量測工具要把錯誤寫進報告
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["samples"] = samples

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"samples={len(samples)} pass={result['pass']} error={result['error']}")
    print(f"report={args.report}")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
