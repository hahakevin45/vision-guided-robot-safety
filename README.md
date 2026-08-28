# Vision-Guided Robot Safety Runtime

A safety-aware command execution environment for a vision-guided mobile robot.

This project demonstrates how an unreliable high-level vision source is converted into
bounded commands, checked by a host-side safety governor, transmitted over a binary
serial protocol, and validated by an STM32 controller state machine.

```text
camera / video
  -> ArUco detection
  -> high-level command
  -> safety governor
  -> serial packet
  -> STM32 state machine
  -> ROS 2 diagnostics
```

## Portfolio Demo: Adaptive Safety Radius

[![Adaptive safety-radius demo](media/adaptive_safety_radius_demo.jpg)](media/adaptive_safety_radius_demo.mp4)

During ArUco loss, dead-reckoning uncertainty expands the wall-clearance radius
from **0.20 m to 0.50 m**. The safety filter slows and redirects the robot near
the inflated boundary. A brief visual reacquisition resets the radius, after
which the robot enters the goal's 0.15 m tolerance region.

Recorded evidence in the linked three-view video:

- synchronized aerial motion, onboard camera, and estimated 2D pose;
- dynamic safety radius, wall distance, drift estimate, and filter mode;
- 11.43 s ArUco outage, 0 fence violations, and 0.148 m final position error.

The robot maintains safe clearance throughout the visual outage, resets drift
upon reacquisition, and successfully reaches the target position.

## What This Project Is

A robot that detects ArUco markers, issues motion commands, validates them through
a safety layer, and executes them through a real microcontroller — running on real
hardware with actual encoders.

The safety layer is evaluated quantitatively: **10 filter algorithms across
8 adversarial scenarios** with reproducible metrics. `safe_apf` is the deployed
command filter; `safe_apf_new` is the full-field planner and the only method in
the matrix that completes the obstacle-detour scenario.

**Open-source research prototype.** Demonstrates safe autonomous navigation,
dynamic potential field planning, and real-hardware MCU integration.

## Architecture

```
ros2_ws/src/vgr_core/          Motion math, binary serial protocol, model enums, safety types
ros2_ws/src/vgr_driver/        Vision pipeline, CLI tools, serial driver
ros2_ws/src/vgr_runtime/       ROS 2 bridge and runtime diagnostics
ros2_ws/src/vgr_safety_gate/   Safety filter node (safe_apf by default)
ros2_ws/src/vgr_nav2_bringup/  Nav 2 params, maps, behavior trees
safety_sim/                    Safety filter evaluation framework (10 filters × 8 scenarios)
gazebo_sim/                    Gazebo + Nav 2 integration
nav2_integration/              Pure-Python Nav 2 geometry, odometry, transforms
firmware/                      STM 32 HAL + portable C (no HAL dependency in common/)
```

**Package constraints enforced in CI:**
- `vgr_core` — no ROS, Gazebo, serial, or phase1/phase2 imports
- `vgr_driver` — `vgr_core` plus an explicit OpenCV/NumPy vision layer
- `vgr_runtime` — `rclpy` + `vgr_core` + `vgr_driver`

## Safety Layer Evaluation

| Filter | Approach | S1 wall | S2 marker lost | S3 Nav overrun | S4-S6 | S7 | S8 detour |
|---|---|---|---|---|---|---|---|
| passthrough | no filter | FAIL | FAIL | FAIL | FAIL ×3 | PASS | FAIL |
| clamp_watchdog | speed clamp | FAIL | PASS | PASS | FAIL ×3 | PASS | FAIL |
| **safe_apf** | deployed APF filter | **PASS** | **PASS** | **PASS** | **PASS** | **PASS** | FAIL |
| **safe_apf_new** | full-field SAPF planner | **PASS** | **PASS** | **PASS** | **PASS** | **PASS** | **PASS** |
| cbf | control barrier function | PASS | PASS | PASS | PASS | PASS | FAIL |
| iccbf | input-constrained CBF | PASS | PASS | PASS | PASS | PASS | FAIL |
| geofence_vo | geofence + velocity obstacle | PASS | PASS | PASS | PASS | PASS | FAIL |
| gf_dwa | geofence + DWA | PASS | PASS | PASS | PASS | PASS | FAIL |
| nh_vo | nearest-horizon VO | PASS | PASS | PASS | PASS | PASS | FAIL |
| backup_mps | backup MPC | PASS | PASS | PASS | PASS | PASS | FAIL |

The 10 × 8 matrix reports `min_clearance [m]`,
`time_to_stop_after_fault [s]`, `intervention_ratio`, and `cmd_distortion`.

Run the comparison matrix:

```bash
python3 -m safety_sim compare --output /tmp/safety_compare.md
```

## Quick Start

Install the pure-Python simulation and development dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install ".[demo,dev]"
```

```bash
mkdir -p outputs
# Safety simulation (no hardware required)
python3 -m safety_sim list
python3 -m safety_sim compare --output /tmp/safety_compare.md

# Run the demo on a recorded video (no camera needed).
# Demo footage is not shipped — drop your own ArUco clips into marker_video/.
python3 -m vgr_driver.cli.run_demo \
  --video marker_video/marker_left.webm \
  --max-frames 300 \
  --report outputs/demo_report.json

# Certify live camera safety
python3 -m vgr_driver.cli.certify_camera \
  --camera-index 0 \
  --frames 90 \
  --report outputs/camera_certification.json

# ROS 2 and hardware commands below use the system ROS installation.
# Do not install rclpy from pip.
deactivate
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2_ws/src \
  --packages-select vgr_core vgr_driver vgr_runtime vgr_safety_gate vgr_nav2_bringup
source install/setup.bash

# Full Phase 2 certification suite (requires STM 32 on /dev/ttyACM0)
export ROS_LOG_DIR=/tmp/vgr_ros_logs
python3 -m vgr_runtime.cli.run_all_certifications \
  --controller serial \
  --device /dev/ttyACM0 \
  --baudrate 115200

# Gazebo + Nav 2 acceptance gates
./gazebo_sim/scripts/run_nav2_scenario.sh ground_truth pseudo
./gazebo_sim/scripts/run_nav2_scenario.sh wheel_odom pseudo
```

## ROS 2 Topics

The e2e bridge publishes JSON strings on:

```
/vision/target
/robot/high_level_command
/mcu/state
/diagnostics
```

## Tests

```bash
export PYTHONPATH=".:ros2_ws/src/vgr_core:ros2_ws/src/vgr_driver:ros2_ws/src/vgr_runtime:ros2_ws/src/vgr_safety_gate"
python3 -m pytest tests/ -q
```

Verified public snapshot: **776 passed / 9 skipped**. The suite needs no robot
hardware; ROS 2 and serial interfaces are mocked for unit tests. The skipped
tests require private field bags or cover a documented reference-marker
visibility limit.

## Hardware Evidence

| Test | Result | Date |
|---|---|---|
| Adaptive-radius field run | Position reached, 0.148 m error, 0 fence violations | 2026-07-19 |
| Pi Jazzy Nav 2 + real encoders `/odom` 10 cm goal | PASS | 2026-07-13 |
| Pi Jazzy Nav 2 + real encoders `/odom` 1 m / 5 s | PASS (5.652 s) | 2026-07-13 |
| Dual wheel bench, encoder interrupt counting | PASS | — |
| Right wheel 5-rev camera/encoder agreement | PASS, 3.47° delta | — |
| Left wheel 5-rev camera/encoder agreement | PASS, 4.58° delta | — |
| Nav 2 Gazebo wheel odom + pseudo ArUco obstacle goal | PASS | — |
| STM 32 serial bridge | PASS | — |
| ROS 2 smoke / diagnostics / topic certification | PASS | — |
| Serial fault injection | PASS | — |
| Reliability / reconnect certification | PASS | — |

| Document | Content |
|---|---|
| `docs/safety_layer_tutorial.md` | Safety layer methods tutorial with representative parameters |
| `docs/safety_sim_guide.md` | Safety simulation environment guide |
| `docs/protocol_v1.md` | Binary serial protocol specification |
| `docs/nav2_sim_guide.md` | Nav 2 Gazebo simulation guide |
| `docs/stm32_uart_smoke_test.md` | STM 32 UART smoke test procedure |

## License

MIT — see `LICENSE`.
