#!/usr/bin/env python3
"""編碼器斷訊／暴衝排查（2026-07-15）。

症狀：閉環運動中編碼器計數凍結、馬達持續轉動（無回授暴衝）。
本工具下溫和短脈衝（預設 ±120 cps、每次 1s、左右交替、脈衝間停 1s），
20Hz 記錄每一筆：命令、編碼器計數、MCU state/error、序號回聲。
內建看門狗：0.5s 無計數進展立即 STOP 中止該脈衝並標記 DROPOUT。

用法（在 Pi 上、12V 開、車輪懸空或淨空 50cm）：
    python3 tools/diagnose_encoder_dropout.py [--pulses 10] [--cruise 120]
輸出 outputs/encoder_dropout_diag.json＋逐脈衝摘要。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from vgr_core.model import CommandID  # noqa: E402
from vgr_driver.driver import ControllerBridge  # noqa: E402
from vgr_driver.driver import PosixSerial  # noqa: E402

POLL_S = 0.05
WATCH_S = 0.5


def run_pulse(bridge, left_cps: int, right_cps: int, duration_s: float):
    """單一脈衝：回傳 (records, dropout_flag)。"""
    initial = bridge.read_encoders()
    init = (initial.packet.left_count, initial.packet.right_count)
    records = []
    dropout = False
    window = []
    watch_n = int(WATCH_S / POLL_S)
    start = time.monotonic()
    try:
        while time.monotonic() - start < duration_s:
            spd = bridge.send_set_wheel_speed(left_cps, right_cps)
            enc = bridge.read_encoders()
            cur = (enc.packet.left_count, enc.packet.right_count)
            records.append({
                "t": round(time.monotonic() - start, 3),
                "cmd": [left_cps, right_cps],
                "counts": list(cur),
                "delta": [cur[0] - init[0], cur[1] - init[1]],
                "mcu_state": spd.state.state.name,
                "mcu_error": spd.state.error.name,
                "motor_intent": spd.state.motor_intent.name,
                "enc_flags": enc.packet.flags,
                "enc_seq_ok": enc.sequence == enc.packet.sequence,
            })
            window.append(cur)
            if len(window) > watch_n:
                moved = max(abs(cur[0] - window[0][0]),
                            abs(cur[1] - window[0][1]))
                expect = max(abs(left_cps), abs(right_cps)) * WATCH_S
                if moved < 0.2 * expect:
                    dropout = True
                    break
                window.pop(0)
            time.sleep(POLL_S)
    finally:
        bridge.send_command(CommandID.STOP)
    return records, dropout


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--pulses", type=int, default=10)
    parser.add_argument("--cruise", type=int, default=120)
    parser.add_argument("--pulse-s", type=float, default=1.0)
    parser.add_argument("--report",
                        default="outputs/encoder_dropout_diag.json")
    parser.add_argument("--arm-wait-s", type=float, default=10.0,
                        help="開埠（STM32 重置）後的上電窗口：此期間持續"
                             "發 STOP，操作者把 12V 打開。開埠時 12V 應為關！")
    args = parser.parse_args()

    report = {"pulses": [], "dropouts": 0}
    with PosixSerial(device=args.device, baudrate=115200,
                     timeout_s=0.5) as serial:
        time.sleep(2.5)                 # STM32 開機（12V 關 → 暴衝無害）
        serial.flush_input()
        bridge = ControllerBridge(serial)
        hb = bridge.send_command(CommandID.HEARTBEAT)
        bridge.send_command(CommandID.STOP)
        print(f"== 上電窗口 {args.arm_wait_s:.0f}s：現在把 12V 打開 ==",
              flush=True)
        arm_start = time.monotonic()
        while time.monotonic() - arm_start < args.arm_wait_s:
            bridge.send_command(CommandID.STOP)
            time.sleep(0.2)
        boot_enc = bridge.read_encoders()
        report["boot_counts"] = [boot_enc.packet.left_count,
                                 boot_enc.packet.right_count]
        print(f"boot encoder counts: {report['boot_counts']}"
              f"（非零 = 開機期間馬達動過）")

        for k in range(args.pulses):
            # 左右交替旋轉方向，模擬掃描的使用模式
            sign = 1 if k % 2 == 0 else -1
            records, dropout = run_pulse(
                bridge, sign * args.cruise, -sign * args.cruise, args.pulse_s)
            final = records[-1]["delta"] if records else [0, 0]
            expect = int(args.cruise * args.pulse_s)
            verdict = "DROPOUT" if dropout else "ok"
            report["pulses"].append({
                "pulse": k, "sign": sign, "dropout": dropout,
                "final_delta": final, "expected_abs": expect,
                "records": records,
            })
            report["dropouts"] += int(dropout)
            print(f"pulse {k:2d} ({'R' if sign>0 else 'L'}): "
                  f"ΔL={final[0]:+5d} ΔR={final[1]:+5d} "
                  f"(expect ±{expect}) {verdict}")
            time.sleep(1.0)
        bridge.send_command(CommandID.STOP)

    out = REPO_ROOT / args.report
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1))
    print(f"report: {out}  dropouts={report['dropouts']}/{args.pulses}")
    return 0 if report["dropouts"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
