#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-}"
OUTPUT_DIR="$REPO_ROOT/outputs/nav2_pi"
NAV2_OVERLAY="${VGR_NAV2_WS:-$HOME/nav2_jazzy_ws}/install/setup.bash"
STAMP="$(date +%Y%m%d_%H%M%S)"
PIDS=()
CLEANUP_TOPIC="/cmd_vel_safe"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS2CLI_NO_DAEMON=1
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"

set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$OUTPUT_DIR"

cleanup() {
  local pid
  set +e
  timeout 2 ros2 topic pub --once "$CLEANUP_TOPIC" geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
  if [[ "$CLEANUP_TOPIC" == "/cmd_vel_nav" ]]; then
    sleep 0.2
  fi
  for pid in "${PIDS[@]}"; do
    kill -- "-$pid" >/dev/null 2>&1
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid" >/dev/null 2>&1
  done
}
trap cleanup EXIT INT TERM

start_hardware() {
  local allow_motion="$1"
  local max_counts="$2"
  setsid env ALLOW_MOTION="$allow_motion" MAX_COUNTS_PER_S="$max_counts" \
    "$REPO_ROOT/scripts/run_pi_hardware_bridge.sh" \
    >"$OUTPUT_DIR/${STAMP}_hardware.log" 2>&1 &
  PIDS+=("$!")
  sleep 2
  kill -0 "${PIDS[-1]}"
}

copy_latest() {
  local source="$1"
  local name="$2"
  cp "$source" "$OUTPUT_DIR/${name}_latest.json"
}

require_fresh_pass() {
  local name="$1"
  python3 - "$OUTPUT_DIR/${name}_latest.json" "$name" <<'PY'
import json
from pathlib import Path
import sys
import time

path = Path(sys.argv[1])
name = sys.argv[2]
if not path.is_file():
    raise SystemExit(f"missing prerequisite report: {path}")
report = json.loads(path.read_text(encoding="utf-8"))
age_s = time.time() - path.stat().st_mtime
if report.get("pass") is not True:
    raise SystemExit(f"prerequisite did not pass: {name}")
if age_s < 0.0 or age_s > 900.0:
    raise SystemExit(f"prerequisite is stale: {name} age={age_s:.1f}s")
print(f"fresh prerequisite PASS: {name} age={age_s:.1f}s")
PY
}

check_lifecycle() {
  local log="$1"
  local node state attempt
  local lifecycle_pass=true
  : >"$log"
  for node in map_server planner_server controller_server behavior_server bt_navigator; do
    state=""
    for attempt in {1..5}; do
      state="$(timeout 4 ros2 lifecycle get "/$node" 2>&1 || true)"
      if [[ "$state" == *active* ]]; then break; fi
      sleep 1
    done
    printf '%s: %s\n' "$node" "$state" | tee -a "$log"
    if [[ "$state" != *active* ]]; then lifecycle_pass=false; fi
  done
  [[ "$lifecycle_pass" == true ]]
}

write_goal_failure() {
  local report="$1"
  local mode="$2"
  local reason="$3"
  python3 - "$report" "$mode" "$reason" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
report = {"mode": sys.argv[2], "pass": False, "reasons": [sys.argv[3]]}
path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
}

case "$MODE" in
  stationary)
    REPORT="$OUTPUT_DIR/stationary_${STAMP}.json"
    start_hardware false 120
    python3 -m vgr_runtime.cli.pi_nav2_bench \
      --mode stationary \
      --duration-s 10.0 \
      --report "$REPORT"
    copy_latest "$REPORT" stationary
    ;;
  nav2)
    REPORT="$OUTPUT_DIR/nav2_${STAMP}.json"
    LIFECYCLE_LOG="$OUTPUT_DIR/${STAMP}_lifecycle.log"
    TF_LOG="$OUTPUT_DIR/${STAMP}_tf.log"
    STATUS_LOG="$OUTPUT_DIR/${STAMP}_hardware_status.log"
    start_hardware false 120
    setsid "$REPO_ROOT/scripts/run_pi_nav2_native.sh" \
      >"$OUTPUT_DIR/${STAMP}_nav2.log" 2>&1 &
    PIDS+=("$!")
    sleep 12
    lifecycle_pass=false
    if check_lifecycle "$LIFECYCLE_LOG"; then lifecycle_pass=true; fi
    timeout 6 ros2 run tf2_ros tf2_echo map base_link >"$TF_LOG" 2>&1 || true
    timeout 4 ros2 topic echo --once --full-length /hardware/status \
      >"$STATUS_LOG" 2>&1 || true
    tf_pass=false
    status_pass=false
    if grep -Eq "At time|Translation" "$TF_LOG"; then tf_pass=true; fi
    if grep -q 'left_target_cps.*0' "$STATUS_LOG" && \
       grep -q 'right_target_cps.*0' "$STATUS_LOG"; then
      status_pass=true
    fi
    python3 - "$REPORT" "$lifecycle_pass" "$tf_pass" "$status_pass" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
checks = {
    "lifecycle_active": sys.argv[2] == "true",
    "tf_map_to_base_link": sys.argv[3] == "true",
    "hardware_targets_zero": sys.argv[4] == "true",
    "no_nav2_goal_sent": True,
}
report = {"mode": "nav2", "checks": checks, "pass": all(checks.values())}
path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
    copy_latest "$REPORT" nav2
    python3 - "$REPORT" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
print(("PI_BENCH_PASS" if report["pass"] else "PI_BENCH_FAIL") + " mode=nav2")
raise SystemExit(0 if report["pass"] else 1)
PY
    ;;
  goal10cm)
    if [ "${VGR_WHEELS_RAISED:-}" != "YES" ]; then
      echo "goal10cm requires VGR_WHEELS_RAISED=YES" >&2
      exit 2
    fi
    require_fresh_pass stationary
    require_fresh_pass nav2
    REPORT="$OUTPUT_DIR/goal10cm_${STAMP}.json"
    LIFECYCLE_LOG="$OUTPUT_DIR/${STAMP}_goal10cm_lifecycle.log"
    TF_LOG="$OUTPUT_DIR/${STAMP}_goal10cm_tf.log"
    start_hardware true 120
    setsid "$REPO_ROOT/scripts/run_pi_nav2_native.sh" \
      map:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/maps/vgr_5x5_center.yaml" \
      params_file:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/config/nav2_goal_bench_params.yaml" \
      >"$OUTPUT_DIR/${STAMP}_goal10cm_nav2.log" 2>&1 &
    PIDS+=("$!")
    sleep 12
    if ! kill -0 "${PIDS[-1]}"; then
      write_goal_failure "$REPORT" goal10cm "Nav2 process exited during startup"
      copy_latest "$REPORT" goal10cm
      exit 1
    fi
    if ! check_lifecycle "$LIFECYCLE_LOG"; then
      write_goal_failure "$REPORT" goal10cm "Nav2 lifecycle nodes did not all become active"
      copy_latest "$REPORT" goal10cm
      exit 1
    fi
    timeout 6 ros2 run tf2_ros tf2_echo map base_link >"$TF_LOG" 2>&1 || true
    if ! grep -Eq "At time|Translation" "$TF_LOG"; then
      write_goal_failure "$REPORT" goal10cm "map to base_link TF was unavailable"
      copy_latest "$REPORT" goal10cm
      exit 1
    fi
    if [[ ! -f "$NAV2_OVERLAY" ]]; then
      write_goal_failure "$REPORT" goal10cm "native Nav2 overlay is missing: $NAV2_OVERLAY"
      copy_latest "$REPORT" goal10cm
      exit 1
    fi
    set +u
    source "$NAV2_OVERLAY"
    set -u
    set +e
    timeout 30 python3 -m vgr_runtime.cli.pi_nav2_goal_bench \
      --goal-x 0.10 \
      --goal-y 0.0 \
      --goal-yaw 0.0 \
      --timeout-s 20.0 \
      --max-linear-mps 0.03 \
      --max-angular-rad-s 0.25 \
      --stale-s 0.20 \
      --wheels-raised YES \
      --report "$REPORT"
    harness_rc=$?
    set -e
    if [[ ! -f "$REPORT" ]]; then
      write_goal_failure "$REPORT" goal10cm "goal harness exited without a report (rc=$harness_rc)"
      harness_rc=1
    fi
    copy_latest "$REPORT" goal10cm
    exit "$harness_rc"
    ;;
  goal10cm_gate)
    CLEANUP_TOPIC="/cmd_vel_nav"
    if [ "${VGR_WHEELS_RAISED:-}" != "YES" ]; then
      echo "goal10cm_gate requires VGR_WHEELS_RAISED=YES" >&2
      exit 2
    fi
    require_fresh_pass stationary
    require_fresh_pass nav2
    REPORT="$OUTPUT_DIR/goal10cm_gate_${STAMP}.json"
    LIFECYCLE_LOG="$OUTPUT_DIR/${STAMP}_goal10cm_gate_lifecycle.log"
    TF_LOG="$OUTPUT_DIR/${STAMP}_goal10cm_gate_tf.log"
    SAFETY_STATUS_LOG="$OUTPUT_DIR/${STAMP}_goal10cm_gate_safety_status.log"
    SAFETY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
    start_hardware true 120
    setsid "$REPO_ROOT/scripts/run_pi_nav2_native.sh" \
      map:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/maps/vgr_5x5_center.yaml" \
      params_file:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/config/nav2_goal_bench_params.yaml" \
      >"$OUTPUT_DIR/${STAMP}_goal10cm_gate_nav2.log" 2>&1 &
    PIDS+=("$!")
    sleep 12
    if ! kill -0 "${PIDS[-1]}"; then
      write_goal_failure "$REPORT" goal10cm_gate "Nav2 process exited during startup"
      copy_latest "$REPORT" goal10cm_gate
      exit 1
    fi
    if ! check_lifecycle "$LIFECYCLE_LOG"; then
      write_goal_failure "$REPORT" goal10cm_gate "Nav2 lifecycle nodes did not all become active"
      copy_latest "$REPORT" goal10cm_gate
      exit 1
    fi
    timeout 6 ros2 run tf2_ros tf2_echo map base_link >"$TF_LOG" 2>&1 || true
    if ! grep -Eq "At time|Translation" "$TF_LOG"; then
      write_goal_failure "$REPORT" goal10cm_gate "map to base_link TF was unavailable"
      copy_latest "$REPORT" goal10cm_gate
      exit 1
    fi
    if [[ ! -f "$NAV2_OVERLAY" ]]; then
      write_goal_failure "$REPORT" goal10cm_gate "native Nav2 overlay is missing: $NAV2_OVERLAY"
      copy_latest "$REPORT" goal10cm_gate
      exit 1
    fi
    if [[ ! -f "$SAFETY_SETUP" ]]; then
      write_goal_failure "$REPORT" goal10cm_gate "safety gate overlay is missing: $SAFETY_SETUP"
      copy_latest "$REPORT" goal10cm_gate
      exit 1
    fi
    set +u
    source "$SAFETY_SETUP"
    set -u
    setsid ros2 run vgr_safety_gate bench_pseudo_pose \
      >"$OUTPUT_DIR/${STAMP}_goal10cm_gate_pseudo_pose.log" 2>&1 &
    PIDS+=("$!")
    setsid ros2 run vgr_safety_gate safety_gate_node --ros-args \
      -p filter_name:=safe_apf \
      -p max_v_mps:=0.03 \
      -p max_omega_rad_s:=0.25 \
      -p nav_timeout_s:=0.2 \
      -p control_hz:=20.0 \
      -p use_sim_time:=false \
      -p 'geofence:=[-2.5,-2.5, 2.5,-2.5, 2.5,2.5, -2.5,2.5]' \
      >"$OUTPUT_DIR/${STAMP}_goal10cm_gate_safety.log" 2>&1 &
    PIDS+=("$!")
    sleep 3
    if ! kill -0 "${PIDS[-2]}" || ! kill -0 "${PIDS[-1]}"; then
      write_goal_failure "$REPORT" goal10cm_gate "pseudo-pose or safety gate exited during startup"
      copy_latest "$REPORT" goal10cm_gate
      exit 1
    fi
    if ! timeout 5 ros2 service type /aruco/set_dropout | grep -q 'std_srvs/srv/SetBool'; then
      write_goal_failure "$REPORT" goal10cm_gate "pseudo-pose dropout service was unavailable"
      copy_latest "$REPORT" goal10cm_gate
      exit 1
    fi
    setsid timeout 35 ros2 topic echo --full-length /safety_gate/status \
      >"$SAFETY_STATUS_LOG" 2>&1 &
    PIDS+=("$!")
    set +u
    source "$NAV2_OVERLAY"
    set -u
    set +e
    timeout 30 python3 -m vgr_runtime.cli.pi_nav2_goal_bench \
      --goal-x 0.10 \
      --goal-y 0.0 \
      --goal-yaw 0.0 \
      --timeout-s 20.0 \
      --max-linear-mps 0.03 \
      --max-angular-rad-s 0.25 \
      --stale-s 0.20 \
      --external-relay \
      --wheels-raised YES \
      --report "$REPORT"
    harness_rc=$?
    set -e
    if [[ ! -f "$REPORT" ]]; then
      write_goal_failure "$REPORT" goal10cm_gate "external-relay harness exited without a report (rc=$harness_rc)"
      harness_rc=1
    fi
    copy_latest "$REPORT" goal10cm_gate
    exit "$harness_rc"
    ;;
  speed1m)
    if [ "${VGR_WHEELS_RAISED:-}" != "YES" ]; then
      echo "speed1m requires VGR_WHEELS_RAISED=YES" >&2
      exit 2
    fi
    require_fresh_pass stationary
    require_fresh_pass nav2
    HIGH_SPEED_REPORT="$OUTPUT_DIR/high_speed_${STAMP}.json"
    set +e
    timeout 22 python3 -m vgr_driver.cli.pi_high_speed_bench \
      --device "${VGR_SERIAL_DEVICE:-/dev/ttyACM0}" \
      --wheels-raised YES \
      --report "$HIGH_SPEED_REPORT"
    high_speed_rc=$?
    set -e
    if [[ ! -f "$HIGH_SPEED_REPORT" ]]; then
      python3 - "$HIGH_SPEED_REPORT" "$high_speed_rc" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
report = {
    "mode": "high_speed",
    "pass": False,
    "reasons": [f"high-speed bench exited without a report (rc={sys.argv[2]})"],
}
path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
      high_speed_rc=1
    fi
    copy_latest "$HIGH_SPEED_REPORT" high_speed
    if [[ "$high_speed_rc" -ne 0 ]]; then
      exit "$high_speed_rc"
    fi
    if fuser "${VGR_SERIAL_DEVICE:-/dev/ttyACM0}" >/dev/null 2>&1; then
      echo "serial device remained busy after high-speed gate" >&2
      exit 1
    fi

    GOAL_REPORT="$OUTPUT_DIR/goal1m_${STAMP}.json"
    LIFECYCLE_LOG="$OUTPUT_DIR/${STAMP}_goal1m_lifecycle.log"
    TF_LOG="$OUTPUT_DIR/${STAMP}_goal1m_tf.log"
    start_hardware true 900
    setsid "$REPO_ROOT/scripts/run_pi_nav2_native.sh" \
      map:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/maps/vgr_5x5_center.yaml" \
      params_file:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/config/nav2_speed_bench_params.yaml" \
      >"$OUTPUT_DIR/${STAMP}_goal1m_nav2.log" 2>&1 &
    PIDS+=("$!")
    sleep 12
    if ! kill -0 "${PIDS[-1]}"; then
      write_goal_failure "$GOAL_REPORT" goal1m "Nav2 process exited during 1 m startup"
      copy_latest "$GOAL_REPORT" goal1m
      exit 1
    fi
    if ! check_lifecycle "$LIFECYCLE_LOG"; then
      write_goal_failure "$GOAL_REPORT" goal1m "Nav2 lifecycle nodes did not all become active"
      copy_latest "$GOAL_REPORT" goal1m
      exit 1
    fi
    timeout 6 ros2 run tf2_ros tf2_echo map base_link >"$TF_LOG" 2>&1 || true
    if ! grep -Eq "At time|Translation" "$TF_LOG"; then
      write_goal_failure "$GOAL_REPORT" goal1m "map to base_link TF was unavailable"
      copy_latest "$GOAL_REPORT" goal1m
      exit 1
    fi
    if [[ ! -f "$NAV2_OVERLAY" ]]; then
      write_goal_failure "$GOAL_REPORT" goal1m "native Nav2 overlay is missing: $NAV2_OVERLAY"
      copy_latest "$GOAL_REPORT" goal1m
      exit 1
    fi
    set +u
    source "$NAV2_OVERLAY"
    set -u
    set +e
    timeout 22 python3 -m vgr_runtime.cli.pi_nav2_1m_bench \
      --goal-x 1.00 \
      --goal-y 0.0 \
      --goal-yaw 0.0 \
      --timeout-s 12.0 \
      --max-linear-mps 0.20 \
      --max-angular-rad-s 0.25 \
      --stale-s 0.20 \
      --wheels-raised YES \
      --report "$GOAL_REPORT"
    goal_rc=$?
    set -e
    if [[ ! -f "$GOAL_REPORT" ]]; then
      write_goal_failure "$GOAL_REPORT" goal1m "1 m goal harness exited without a report (rc=$goal_rc)"
      goal_rc=1
    fi
    copy_latest "$GOAL_REPORT" goal1m
    exit "$goal_rc"
    ;;
  motion)
    if [ "${VGR_WHEELS_RAISED:-}" != "YES" ]; then
      echo "motion requires VGR_WHEELS_RAISED=YES" >&2
      exit 2
    fi
    REPORT="$OUTPUT_DIR/motion_${STAMP}.json"
    start_hardware true 120
    python3 -m vgr_runtime.cli.pi_nav2_bench \
      --mode motion \
      --command-s 0.5 \
      --linear-mps 0.02 \
      --angular-rad-s 0.20 \
      --wheels-raised YES \
      --report "$REPORT"
    copy_latest "$REPORT" motion
    ;;
  *)
    echo "usage: $0 stationary|nav2|motion|goal10cm|goal10cm_gate|speed1m" >&2
    exit 2
    ;;
esac
