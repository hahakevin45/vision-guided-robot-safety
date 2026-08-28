#!/bin/bash
# E3 STM32 韌體防線演練：低速、輪子離牆至少 1.5 m，12V 流程照舊。
#
# 安全前提：現場有人、輪子離牆 >=1.5m、低速 v=+0.05 m/s；12V 先關，
# 節點就緒後才開 12V，再透過 VGR_START_GATE 放行。不要在未確認場地時執行。
#
# MODE=KILL：起跑後殺 bridge；bridge 死後不再送任何 host 命令，等 STM32
# 自身 command timeout 停輪。MODE=FAULT：保留 bridge，以定時壞幀注入演練。
# bag 會在 KILL 時因 /odom 擁有者死亡而斷流，這是預期；最後一筆 /odom 到
# 實體停住的延遲必須事後用 bag/encoder 證據分析，不以 bag 斷流時間冒充停車時間。
#
# 用法：
#   VGR_WHEELS_RAISED=YES VGR_FIRMWARE_DRILL_MODE=KILL  bash scripts/run_pi_firmware_drill.sh
#   VGR_WHEELS_RAISED=YES VGR_FIRMWARE_DRILL_MODE=FAULT bash scripts/run_pi_firmware_drill.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${VGR_OUTPUT_DIR:-$REPO_ROOT/outputs/nav2_pi}"
STAMP="$(date +%Y%m%d_%H%M%S)"
MODE="${VGR_FIRMWARE_DRILL_MODE:-KILL}"
KILL_AT_S="${VGR_KILL_AT_S:-5}"
FAULT_AT_S="${VGR_FAULT_AT_S:-5}"
FAULT_MODE="${VGR_FAULT_INJECT_MODE:-bad_checksum}"
FAULT_COUNT="${VGR_FAULT_INJECT_COUNT:-10}"
DRILL_SECONDS="${VGR_DRILL_SECONDS:-12}"
POLL_HZ="${POLL_HZ:-20.0}"
PIDS=()
BRIDGE_PID=""
CMD_PID=""
BAG_PID=""

case "$MODE" in
  KILL|FAULT) ;;
  *) echo "VGR_FIRMWARE_DRILL_MODE must be KILL or FAULT" >&2; exit 2 ;;
esac
if [[ "${VGR_WHEELS_RAISED:-}" != "YES" && "${VGR_REHEARSAL:-}" != "YES" ]]; then
  echo "firmware drill requires VGR_WHEELS_RAISED=YES (or VGR_REHEARSAL=YES)" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS2CLI_NO_DAEMON=1
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$OUTPUT_DIR"

cleanup() {
  set +e
  local pid
  # KILL 的驗收區間不送 stop；此處只在演練結束後清理程序。
  for pid in "${PIDS[@]}"; do
    kill -9 -- "-$pid" >/dev/null 2>&1
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid" >/dev/null 2>&1
  done
}
trap cleanup EXIT INT TERM

HW_LOG="$OUTPUT_DIR/${STAMP}_firmware_drill_bridge.log"
if [[ "$MODE" == "FAULT" ]]; then
  INJECT_ARGS=(
    -p "fault_inject_mode:=$FAULT_MODE"
    -p "fault_inject_at_s:=$FAULT_AT_S"
    -p "fault_inject_count:=$FAULT_COUNT"
  )
else
  INJECT_ARGS=(-p fault_inject_mode:=none -p fault_inject_at_s:=-1.0)
fi

echo "[E3] 起 hardware bridge mode=$MODE poll_hz=$POLL_HZ"
setsid env ALLOW_MOTION=true MAX_COUNTS_PER_S=120 \
  python3 -m vgr_runtime.ros.hardware_bridge --ros-args \
  -p use_sim_time:=false \
  -p "device:=${VGR_SERIAL_DEVICE:-/dev/ttyACM0}" \
  -p "baudrate:=${VGR_SERIAL_BAUD:-115200}" \
  -p "serial_timeout_s:=${SERIAL_TIMEOUT_S:-0.10}" \
  -p "settle_s:=${SERIAL_SETTLE_S:-0.50}" \
  -p "poll_hz:=$POLL_HZ" \
  -p allow_motion:=true \
  -p cmd_timeout_s:="${CMD_TIMEOUT_S:-0.20}" \
  -p max_counts_per_s:=120 \
  "${INJECT_ARGS[@]}" \
  >"$HW_LOG" 2>&1 &
BRIDGE_PID="$!"
PIDS+=("$BRIDGE_PID")
sleep 3
kill -0 "$BRIDGE_PID" || { echo "E3_FAIL bridge startup"; exit 1; }

if [[ -n "${VGR_START_GATE:-}" ]]; then
  rm -f "$VGR_START_GATE"
  echo "[E3] 等待起跑：touch $VGR_START_GATE 後 ${VGR_START_DELAY_S:-3} 秒起跑"
  until [[ -f "$VGR_START_GATE" ]]; do sleep 0.5; done
  sleep "${VGR_START_DELAY_S:-3}"
fi

BAG_DIR="$OUTPUT_DIR/${STAMP}_firmware_drill_bag"
setsid ros2 bag record -o "$BAG_DIR" \
  /odom /hardware/status /cmd_vel_safe \
  >"$OUTPUT_DIR/${STAMP}_firmware_drill_bag.log" 2>&1 &
BAG_PID="$!"
PIDS+=("$BAG_PID")
sleep 1

echo "[E3] 起跑：正向 v=+0.05 m/s @20Hz，共 ${DRILL_SECONDS}s"
setsid timeout "$DRILL_SECONDS" ros2 topic pub -r 20 /cmd_vel_safe \
  geometry_msgs/msg/Twist '{linear: {x: 0.05}, angular: {z: 0.0}}' \
  >"$OUTPUT_DIR/${STAMP}_firmware_drill_cmd.log" 2>&1 &
CMD_PID="$!"
PIDS+=("$CMD_PID")
RUN_START="$(date +%s.%N)"

if [[ "$MODE" == "KILL" ]]; then
  echo "[E3] KILL scheduled at +${KILL_AT_S}s; no host command after bridge kill"
  sleep "$KILL_AT_S"
  KILL_TS="$(date +%s.%N)"
  kill -9 -- "-$CMD_PID" >/dev/null 2>&1 || true
  kill -9 -- "-$BRIDGE_PID" >/dev/null 2>&1 || true
  echo "[E3] bridge killed at epoch=$KILL_TS"
  echo "[E3] encoder evidence: analyze last /odom before $KILL_TS against physical stop; bag discontinuity is expected"
else
  echo "[E3] FAULT scheduled at bridge +${FAULT_AT_S}s mode=$FAULT_MODE count=$FAULT_COUNT"
  sleep "$DRILL_SECONDS"
  echo "[E3] fault evidence: grep 'fault injection started' '$HW_LOG'"
fi

sleep "${VGR_POST_RUN_S:-3}"
echo "E3_DRILL_OUTPUT=$OUTPUT_DIR"
echo "E3_DRILL_BAG=$BAG_DIR"
echo "E3_DRILL_BRIDGE_LOG=$HW_LOG"
