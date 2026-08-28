from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a Phase 2 validation report.")
    parser.add_argument("--batch", type=Path, default=Path("outputs/phase2_e2e_batch_report.json"))
    parser.add_argument("--faults", type=Path, default=Path("outputs/real_mcu_fault_certification.json"))
    parser.add_argument("--serial", type=Path, default=Path("outputs/real_mcu_serial_certification.json"))
    parser.add_argument("--reliability", type=Path, default=Path("outputs/phase2_reliability_report.json"))
    parser.add_argument("--motor-intent", type=Path, default=Path("outputs/real_mcu_motor_intent_certification.json"))
    parser.add_argument("--ros2-topics", type=Path, default=Path("outputs/ros2_topic_certification.json"))
    parser.add_argument("--output", type=Path, default=Path("outputs/phase2_validation_report.md"))
    args = parser.parse_args()

    batch = read_json(args.batch)
    faults = read_json(args.faults)
    serial = read_json(args.serial)
    reliability = read_json_optional(args.reliability)
    motor_intent = read_json_optional(args.motor_intent)
    ros2_topics = read_json_optional(args.ros2_topics)
    markdown = render_report(
        batch=batch,
        faults=faults,
        serial=serial,
        reliability=reliability,
        motor_intent=motor_intent,
        ros2_topics=ros2_topics,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(markdown, encoding="utf-8")
    print(f"Wrote {args.output}")
    passed = report_passed(batch, faults, serial, reliability, motor_intent, ros2_topics)
    print("PHASE 2 VALIDATION REPORT: PASS" if passed else "PHASE 2 VALIDATION REPORT: FAIL")
    return 0 if passed else 1


def read_json(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def read_json_optional(path: Path) -> dict | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def report_passed(
    batch: dict,
    faults: dict,
    serial: dict,
    reliability: dict | None = None,
    motor_intent: dict | None = None,
    ros2_topics: dict | None = None,
) -> bool:
    optional_ok = True
    if reliability is not None:
        optional_ok = optional_ok and bool(reliability.get("pass"))
    if motor_intent is not None:
        optional_ok = optional_ok and bool(motor_intent.get("pass"))
    if ros2_topics is not None:
        optional_ok = optional_ok and bool(ros2_topics.get("pass"))
    return bool(batch.get("pass") and faults.get("pass") and serial.get("pass") and optional_ok)


def render_report(
    batch: dict,
    faults: dict,
    serial: dict,
    reliability: dict | None = None,
    motor_intent: dict | None = None,
    ros2_topics: dict | None = None,
) -> str:
    lines = [
        "# Phase 2 Validation Report",
        "",
        "## Summary",
        "",
        f"- Serial bridge certification: `{status(serial)}`",
        f"- E2E batch certification: `{status(batch)}`",
        f"- Fault injection certification: `{status(faults)}`",
        f"- Reliability certification: `{status(reliability)}`" if reliability is not None else "- Reliability certification: `not provided`",
        f"- Motor intent certification: `{status(motor_intent)}`" if motor_intent is not None else "- Motor intent certification: `not provided`",
        f"- ROS2 topic certification: `{status(ros2_topics)}`" if ros2_topics is not None else "- ROS2 topic certification: `not provided`",
        f"- Controller: `{batch.get('controller', 'unknown')}`",
        f"- Device: `{batch.get('device') or serial.get('device') or 'unknown'}`",
        "",
        "## Serial Bridge",
        "",
        "| Command | Sequence | MCU State | Error | Motor Intent | Latency ms |",
        "| --- | ---: | --- | --- | --- | ---: |",
    ]
    for exchange in serial.get("exchanges", []):
        lines.append(
            "| {command} | {sequence} | `{mcu_state}` | `{mcu_error}` | `{motor_intent}` | {latency_ms:.3f} |".format(
                command=exchange.get("command", "unknown"),
                sequence=exchange.get("sequence", 0),
                mcu_state=exchange.get("mcu_state", "unknown"),
                mcu_error=exchange.get("mcu_error", "unknown"),
                motor_intent=exchange.get("motor_intent", "unknown"),
                latency_ms=float(exchange.get("latency_ms", 0.0)),
            )
        )

    e2e_latencies = collect_e2e_latencies(batch)
    lines.extend(
        [
            "",
            "## E2E Batch",
            "",
            f"- Videos: `{batch.get('summary', {}).get('video_count', 0)}`",
            f"- Passed videos: `{batch.get('summary', {}).get('passed_videos', 0)}`",
            f"- Total frames: `{batch.get('summary', {}).get('total_frames', 0)}`",
            f"- Total MCU accepted: `{batch.get('summary', {}).get('total_mcu_accepted', 0)}`",
            f"- Total MCU rejected: `{batch.get('summary', {}).get('total_mcu_rejected', 0)}`",
            f"- Avg MCU latency: `{mean(e2e_latencies):.3f} ms`" if e2e_latencies else "- Avg MCU latency: `n/a`",
            f"- P95 MCU latency: `{p95(e2e_latencies):.3f} ms`" if e2e_latencies else "- P95 MCU latency: `n/a`",
            "",
            "| Video | Pass | Expected | Frames | Detections | MCU Accepted | MCU Rejected | Avg MCU ms | P95 MCU ms |",
            "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for video in batch.get("videos", []):
        summary = video.get("summary", {})
        lines.append(
            "| {video} | `{passed}` | `{expected}` | {frames} | {detections} | {mcu_accepted} | {mcu_rejected} | {avg:.3f} | {p95v:.3f} |".format(
                video=video.get("video", "unknown"),
                passed="PASS" if video.get("pass") else "FAIL",
                expected=video.get("expected_behavior", "unknown"),
                frames=summary.get("frames", 0),
                detections=summary.get("detections", 0),
                mcu_accepted=summary.get("mcu_accepted", 0),
                mcu_rejected=summary.get("mcu_rejected", 0),
                avg=summary.get("avg_mcu_latency_ms", 0.0),
                p95v=summary.get("p95_mcu_latency_ms", 0.0),
            )
        )

    lines.extend(
        [
            "",
            "## Fault Injection",
            "",
            "| Fault | Pass | Expected Error | MCU State | MCU Error |",
            "| --- | --- | --- | --- | --- |",
        ]
    )
    for fault in faults.get("faults", []):
        lines.append(
            "| {name} | `{passed}` | `{expected}` | `{state}` | `{error}` |".format(
                name=fault.get("name", "unknown"),
                passed="PASS" if fault.get("pass") else "FAIL",
                expected=fault.get("expected_error", "unknown"),
                state=fault.get("mcu_state", "unknown"),
                error=fault.get("mcu_error", "unknown"),
            )
        )

    if reliability is not None:
        soak = reliability.get("soak", {})
        lines.extend(
            [
                "",
                "## Reliability",
                "",
                f"- Completed cycles: `{soak.get('completed_cycles', 0)}`",
                f"- Soak errors OK: `{soak.get('mcu_errors_ok', False)}`",
                f"- Sequence echo OK: `{soak.get('sequence_echo_ok', False)}`",
                f"- Avg latency: `{soak.get('avg_latency_ms', 0.0):.3f} ms`",
                f"- Max latency: `{soak.get('max_latency_ms', 0.0):.3f} ms`",
            ]
        )

    if motor_intent is not None:
        lines.extend(
            [
                "",
                "## Motor Intent",
                "",
                "| Command | MCU State | Error | Expected Intent | Reported Intent |",
                "| --- | --- | --- | --- | --- |",
            ]
        )
        for exchange in motor_intent.get("exchanges", []):
            lines.append(
                "| {command} | `{mcu_state}` | `{mcu_error}` | `{expected}` | `{actual}` |".format(
                    command=exchange.get("command", "unknown"),
                    mcu_state=exchange.get("mcu_state", "unknown"),
                    mcu_error=exchange.get("mcu_error", "unknown"),
                    expected=exchange.get("expected_motor_intent", "unknown"),
                    actual=exchange.get("motor_intent", "unknown"),
                )
            )

    if ros2_topics is not None:
        summary = ros2_topics.get("summary", {})
        lines.extend(
            [
                "",
                "## ROS2 Topics",
                "",
                f"- Topic certification: `{status(ros2_topics)}`",
                f"- Topic counts: `{summary.get('topic_counts', {})}`",
                f"- Commands: `{summary.get('commands', {})}`",
                f"- MCU states: `{summary.get('mcu_states', {})}`",
                f"- MCU errors: `{summary.get('mcu_errors', {})}`",
            ]
        )

    lines.extend(
        [
            "",
            "## Conclusion",
            "",
            conclusion(batch, faults, serial, reliability, motor_intent, ros2_topics),
            "",
        ]
    )
    return "\n".join(lines)


def status(report: dict) -> str:
    if report is None:
        return "not provided"
    return "PASS" if report.get("pass") else "FAIL"


def collect_e2e_latencies(batch: dict) -> list[float]:
    latencies = []
    for video in batch.get("videos", []):
        summary = video.get("summary", {})
        count = int(summary.get("mcu_accepted", 0))
        if count > 0:
            latencies.extend([float(summary.get("avg_mcu_latency_ms", 0.0))] * count)
    return latencies


def p95(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[int(round((len(ordered) - 1) * 0.95))]


def conclusion(
    batch: dict,
    faults: dict,
    serial: dict,
    reliability: dict | None = None,
    motor_intent: dict | None = None,
    ros2_topics: dict | None = None,
) -> str:
    if report_passed(batch, faults, serial, reliability, motor_intent, ros2_topics):
        return (
            "Phase 2 is validated for host-to-STM32 command transport, MCU state "
            "telemetry, repeated e2e video-command execution, rejection of bad "
            "checksum / sequence-gap packets, reliability checks, dry-run motor "
            "intent telemetry, and ROS2 topic publication."
        )
    return "Phase 2 validation is incomplete; inspect failed sections above."


if __name__ == "__main__":
    raise SystemExit(main())
