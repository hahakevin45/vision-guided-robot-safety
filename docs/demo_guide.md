# Demo Guide

This guide gives a short path for demonstrating the project without motors.

Commands below that take `--video marker_video/...` need your own ArUco recording:
`marker_video/` is not shipped with the repo. Any clip showing the markers works.

## Demo 1: Live Camera Safety

Keep the marker out of view:

```bash
python3 -m vgr_driver.cli.certify_camera \
  --camera-index 0 \
  --frames 90 \
  --report outputs/live_camera_certification.json
```

Expected:

```text
CAMERA CERTIFICATION: PASS
```

This demonstrates that no target results in `STOP` / `SAFE_STOP`.

## Demo 2: STM32 Serial Bridge

```bash
python3 -m vgr_driver.cli.certify_serial_bridge \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --report outputs/real_mcu_serial_certification.json
```

Expected:

```text
SERIAL BRIDGE CERTIFICATION: PASS
```

This demonstrates host-to-controller command/state packet exchange.

## Demo 3: Vision To STM32 Batch

```bash
python3 -m vgr_driver.cli.run_e2e_batch \
  --controller serial \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --report outputs/phase2_e2e_batch_report.json
```

Expected:

```text
PHASE 2 E2E BATCH: PASS
```

This demonstrates all prerecorded marker cases:

- left marker -> `TURN_LEFT`
- right marker -> `TURN_RIGHT`
- lost marker -> `SAFE_STOP`
- close/up marker -> `STOP`

## Demo 4: Fault Injection

```bash
python3 -m vgr_driver.cli.certify_faults \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --report outputs/real_mcu_fault_certification.json
```

Expected:

```text
SERIAL FAULT CERTIFICATION: PASS
```

Covered cases:

- bad checksum -> `BAD_CHECKSUM`
- sequence gap -> `BAD_SEQUENCE`

## Demo 5: ROS2 Topics

Terminal A:

```bash
export ROS_LOG_DIR=/tmp/vision_guided_robot_ros_logs

python3 -m vgr_runtime.cli.ros2_e2e_bridge \
  --video marker_video/marker_left.webm \
  --controller serial \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --max-frames 120
```

Terminal B:

```bash
ros2 topic list
ros2 topic echo /robot/high_level_command
ros2 topic echo /mcu/state
```

Automated topic certification:

```bash
python3 -m vgr_runtime.cli.certify_ros2_topics \
  --controller serial \
  --device /dev/ttyACM0 \
  --video marker_video/marker_left.webm \
  --report outputs/ros2_topic_certification.json
```

Expected:

```text
ROS2 TOPIC CERTIFICATION: PASS
```

## Full Regression

```bash
export ROS_LOG_DIR=/tmp/vision_guided_robot_ros_logs

python3 -m vgr_runtime.cli.run_all_certifications \
  --controller serial \
  --device /dev/ttyACM0 \
  --baudrate 115200
```

Current Phase 2 firmware must be flashed before this run because MCU state
packets are 10 bytes and include `motor_intent`.

Expected:

```text
PHASE 2 ALL CERTIFICATIONS: PASS
```
