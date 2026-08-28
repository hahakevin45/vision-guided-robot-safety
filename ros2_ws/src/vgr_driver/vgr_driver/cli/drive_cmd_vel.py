"""用 (v, ω) 差動指令直接驅動車子，經 serial SET_WHEEL_SPEED——不需要 ROS。

這是「cmd_vel 控車」的非 ROS 硬體測試工具：驗證逆運動學在真車上左右方向/
差速正確。與 ROS 的 cmd_vel_bridge 共用 diff_drive_kinematics，所以這裡驗過，
Nav2 那條路的數學也一樣對。

安全：低速上限、硬性時間上限、每個 interval 重送（配合韌體 deadman），離開/
例外/逾時一律送 STOP。**務必架空、手放 12V kill switch 再跑。**

範例（原地緩慢左轉 1.5 秒）：
    python3 -m vgr_driver.cli.drive_cmd_vel --device /dev/ttyACM0 \
        --linear-x 0.0 --angular-z 0.6 --duration-s 1.5
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vgr_core.model import CommandID
from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_core.motion import DiffDriveParams, twist_to_wheel_counts
from vgr_driver.driver.serial_transport import PosixSerial

# 初期硬體測試用的保守上限，遠低於韌體 900。要更快自行提高。
DEFAULT_MAX_COUNTS_PER_S = 400
HARD_DURATION_CAP_S = 5.0


def _build_params(args: argparse.Namespace) -> DiffDriveParams:
    return DiffDriveParams(
        wheel_base_m=args.wheel_base_m,
        wheel_diameter_m=args.wheel_diameter_cm / 100.0,
        left_counts_per_rev=args.left_counts_per_rev,
        right_counts_per_rev=args.right_counts_per_rev,
        max_counts_per_s=args.max_counts_per_s,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Drive the car from a single (v, w) twist over serial (no ROS)."
    )
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--linear-x", type=float, default=0.0, help="前進速度 v (m/s)")
    parser.add_argument("--angular-z", type=float, default=0.0, help="旋轉速度 ω (rad/s, +為左轉)")
    parser.add_argument("--duration-s", type=float, default=2.0)
    parser.add_argument("--command-interval-s", type=float, default=0.1)
    parser.add_argument("--wheel-base-m", type=float, default=0.165)
    parser.add_argument("--wheel-diameter-cm", type=float, default=6.5)
    parser.add_argument("--left-counts-per-rev", type=float, default=750.0)
    parser.add_argument("--right-counts-per-rev", type=float, default=749.0)
    parser.add_argument("--max-counts-per-s", type=int, default=DEFAULT_MAX_COUNTS_PER_S)
    parser.add_argument("--report", type=Path, default=Path("outputs/drive_cmd_vel_report.json"))
    args = parser.parse_args()

    duration_s = min(args.duration_s, HARD_DURATION_CAP_S)
    params = _build_params(args)
    left_cps, right_cps = twist_to_wheel_counts(args.linear_x, args.angular_z, params)

    result: dict[str, object] = {
        "pass": False,
        "device": args.device,
        "twist": {"linear_x": args.linear_x, "angular_z": args.angular_z},
        "target_counts_per_s": {"left": left_cps, "right": right_cps},
        "duration_s": duration_s,
        "max_counts_per_s": args.max_counts_per_s,
        "ended_with_stop": False,
        "encoder_delta": {},
        "error": None,
    }

    print(f"twist v={args.linear_x} ω={args.angular_z}  →  目標 counts/s: "
          f"left={left_cps} right={right_cps}  (上限 {args.max_counts_per_s}, {duration_s}s)")

    try:
        with PosixSerial(device=args.device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            bridge = ControllerBridge(serial)
            try:
                if args.settle_s > 0:
                    time.sleep(args.settle_s)
                    serial.flush_input()
                bridge.send_command(CommandID.HEARTBEAT)
                before = bridge.read_encoders().packet

                deadline = time.monotonic() + duration_s
                while time.monotonic() < deadline:
                    bridge.send_set_wheel_speed(left_cps, right_cps)
                    time.sleep(args.command_interval_s)

                after = bridge.read_encoders().packet
                result["encoder_delta"] = {
                    "left": after.left_count - before.left_count,
                    "right": after.right_count - before.right_count,
                }
                result["pass"] = True
            finally:
                # STOP while the serial port is still open (deadman is the
                # firmware-side backstop, but always send an explicit STOP).
                try:
                    stop = bridge.send_command(CommandID.STOP)
                    result["ended_with_stop"] = stop.state.state.name in ("SAFE_STOP", "STOP")
                except Exception as exc:  # noqa: BLE001
                    result["stop_error"] = f"{type(exc).__name__}: {exc}"
    except Exception as exc:  # noqa: BLE001 - 硬體 CLI 要保留失敗細節並務必 STOP。
        result["error"] = f"{type(exc).__name__}: {exc}"

    result["pass"] = bool(result["pass"] and result["ended_with_stop"] and result["error"] is None)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("DRIVE CMD_VEL: PASS" if result["pass"] else "DRIVE CMD_VEL: FAIL")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
