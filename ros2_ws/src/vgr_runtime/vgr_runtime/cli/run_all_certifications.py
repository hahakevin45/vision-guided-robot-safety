from __future__ import annotations

import argparse
import json
import os
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the full Phase 2 certification suite.")
    parser.add_argument("--controller", choices=["mock", "serial"], default="mock")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--video", default="marker_video/marker_left.webm")
    parser.add_argument("--output", type=Path, default=Path("outputs/phase2_all_certifications.json"))
    args = parser.parse_args()

    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp/vision_guided_robot_ros_logs")

    commands = build_commands(args)
    results = []
    for name, command in commands:
        print(f"\n=== {name} ===")
        completed = subprocess.run(command, env=env)
        results.append({"name": name, "returncode": completed.returncode, "command": command})
        if completed.returncode != 0:
            break

    passed = all(result["returncode"] == 0 for result in results) and len(results) == len(commands)
    payload = {
        "pass": passed,
        "controller": args.controller,
        "device": args.device if args.controller == "serial" else None,
        "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps({"pass": passed, "completed": len(results), "expected": len(commands)}, indent=2))
    print("PHASE 2 ALL CERTIFICATIONS: PASS" if passed else "PHASE 2 ALL CERTIFICATIONS: FAIL")
    return 0 if passed else 1


def build_commands(args: argparse.Namespace) -> list[tuple[str, list[str]]]:
    controller_args = []
    if args.controller == "serial":
        controller_args = ["--device", args.device, "--baudrate", str(args.baudrate)]

    serial_bridge = ["python3", "-m", "vgr_driver.cli.certify_serial_bridge"]
    if args.controller == "serial":
        serial_bridge.extend(controller_args)
    serial_bridge.extend(["--report", "outputs/real_mcu_serial_certification.json" if args.controller == "serial" else "outputs/serial_bridge_certification.json"])

    e2e_batch = ["python3", "-m", "vgr_driver.cli.run_e2e_batch", "--controller", args.controller]
    if args.controller == "serial":
        e2e_batch.extend(controller_args)
    e2e_batch.extend(["--report", "outputs/phase2_e2e_batch_report.json" if args.controller == "serial" else "outputs/phase2_e2e_batch_mock_report.json"])

    faults = ["python3", "-m", "vgr_driver.cli.certify_faults"]
    if args.controller == "serial":
        faults.extend(controller_args)
    faults.extend(["--report", "outputs/real_mcu_fault_certification.json" if args.controller == "serial" else "outputs/serial_fault_certification_mock.json"])

    reliability = ["python3", "-m", "vgr_driver.cli.certify_reliability", "--cycles", str(args.cycles)]
    if args.controller == "serial":
        reliability.extend(controller_args)
    reliability.extend(["--report", "outputs/phase2_reliability_report.json" if args.controller == "serial" else "outputs/phase2_reliability_mock_report.json"])

    motor_intent = ["python3", "-m", "vgr_driver.cli.certify_motor_intent"]
    if args.controller == "serial":
        motor_intent.extend(controller_args)
    motor_intent.extend(["--report", "outputs/real_mcu_motor_intent_certification.json" if args.controller == "serial" else "outputs/motor_intent_certification_mock.json"])

    ros2_topics = [
        "python3",
        "-m",
        "vgr_runtime.cli.certify_ros2_topics",
        "--controller",
        args.controller,
        "--video",
        args.video,
        "--report",
        "outputs/ros2_topic_certification.json" if args.controller == "serial" else "outputs/ros2_topic_certification_mock.json",
    ]
    if args.controller == "serial":
        ros2_topics.extend(controller_args)

    validation = ["python3", "-m", "vgr_driver.cli.generate_validation_report", "--output", "outputs/phase2_validation_report.md"]
    if args.controller == "mock":
        validation = [
            "python3",
            "-m",
            "vgr_driver.cli.generate_validation_report",
            "--serial",
            "outputs/serial_bridge_certification.json",
            "--batch",
            "outputs/phase2_e2e_batch_mock_report.json",
            "--faults",
            "outputs/serial_fault_certification_mock.json",
            "--reliability",
            "outputs/phase2_reliability_mock_report.json",
            "--motor-intent",
            "outputs/motor_intent_certification_mock.json",
            "--ros2-topics",
            "outputs/ros2_topic_certification_mock.json",
            "--output",
            "outputs/phase2_validation_report_mock.md",
        ]

    return [
        ("serial_bridge", serial_bridge),
        ("e2e_batch", e2e_batch),
        ("fault_injection", faults),
        ("reliability", reliability),
        ("motor_intent", motor_intent),
        ("ros2_topics", ros2_topics),
        ("validation_report", validation),
    ]


if __name__ == "__main__":
    raise SystemExit(main())
