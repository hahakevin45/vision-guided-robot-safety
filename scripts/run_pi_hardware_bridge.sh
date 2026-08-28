#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m vgr_runtime.ros.hardware_bridge --ros-args \
  -p use_sim_time:=false \
  -p device:="${VGR_SERIAL_DEVICE:-/dev/ttyACM0}" \
  -p baudrate:="${VGR_SERIAL_BAUD:-115200}" \
  -p serial_timeout_s:="${SERIAL_TIMEOUT_S:-0.10}" \
  -p settle_s:="${SERIAL_SETTLE_S:-0.50}" \
  -p poll_hz:="${POLL_HZ:-20.0}" \
  -p allow_motion:=${ALLOW_MOTION:-false} \
  -p cmd_timeout_s:="${CMD_TIMEOUT_S:-0.20}" \
  -p max_counts_per_s:=${MAX_COUNTS_PER_S:-120}
