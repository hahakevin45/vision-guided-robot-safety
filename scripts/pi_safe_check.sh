#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-$HOME/vision_guided_robot}"
DEVICE="${DEVICE:-/dev/ttyACM0}"
BAUDRATE="${BAUDRATE:-115200}"
CAMERA_INDEX="${CAMERA_INDEX:-0}"
FRAMES="${FRAMES:-90}"

cd "$ROOT"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

mkdir -p outputs

python3 -m vgr_driver.cli.certify_camera \
  --camera-index "$CAMERA_INDEX" \
  --frames "$FRAMES" \
  --report outputs/pi_safe_camera_certification.json

python3 -m vgr_runtime.cli.ros2_smoke_test \
  --timeout-s 3.0

python3 -m vgr_runtime.cli.certify_ros2_topics --controller mock \
  --max-frames 60 \
  --timeout-s 12 \
  --report outputs/pi_safe_ros2_mock_topic_certification.json

python3 -m vgr_runtime.cli.certify_ros2_safe_serial \
  --device "$DEVICE" \
  --baudrate "$BAUDRATE" \
  --settle-s 0.5 \
  --publish-ros2 \
  --report outputs/pi_safe_ros2_serial_certification.json

echo "PI SAFE CHECK: PASS"
