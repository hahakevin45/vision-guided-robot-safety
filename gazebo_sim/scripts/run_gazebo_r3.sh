#!/bin/bash
# R3 virtual-geofence Gazebo experiment.
#
# 地上沒有任何實體障礙；禁區線只存在於 map/geofence（線 x=2.0，禁 x>2.0）。
#   sapf_new arm：safety_gate filter=safe_apf_new + fixed goal (3.0,0) + geofence 虛擬線。
#   passthrough arm：safety_gate filter=passthrough + sapf_nominal（同一 obstacle-free
#   attractive command，spec 4.2）；越線後由有限 capture timeout 停止。
# 兩 arm 都啟動 sapf_nominal：對 sapf_new 它是 motion authorization（SAPF 自產命令）。
#
# 評估：trace 的 /sim/true_pose 序列 → evaluate_r3_trace（true_pose 只進 evaluator）。
#   sapf_new：crossed=False 且 min true clearance >= 0.05。
#   passthrough：crossed=True（負對照必須越線）。
#
# 用法：
#   bash gazebo_sim/scripts/run_gazebo_r3.sh --all --out /tmp/r3_gazebo
#   bash gazebo_sim/scripts/run_gazebo_r3.sh --arm sapf_new --out /tmp/r3_gazebo
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
WORLD_SRC="${SAPF_WORLD:-$REPO_ROOT/gazebo_sim/worlds/vgr_arena.world}"
LINE_X="${LINE_X:-2.0}"
GOAL_X=3.0
GOAL_Y=0.0
SPAWN_POSE="0.5 0 0 0 0 0"
ARM=""
OUT_DIR=""
ALL=0
RUN_ID_ARG="single"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm) ARM="$2"; shift 2 ;;
    --out) OUT_DIR="$2"; shift 2 ;;
    --run-id) RUN_ID_ARG="$2"; shift 2 ;;
    --all) ALL=1; shift ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$OUT_DIR" ]] || { echo "need --out DIR" >&2; exit 2; }
mkdir -p "$OUT_DIR"

export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS2CLI_NO_DAEMON=1

run_one() {
  local arm="$1" run_id="$2"
  local TMP_DIR="$OUT_DIR/tmp_${arm}_${run_id}"
  mkdir -p "$TMP_DIR/home"
  local TRACE="$OUT_DIR/${arm}_${run_id}.jsonl"
  local RUN_LOG="$OUT_DIR/${arm}_${run_id}.log"
  local PIDS=()

  cleanup() {
    for pid in "${PIDS[@]:-}"; do
      kill "$pid" 2>/dev/null || true
    done
    for pid in "${PIDS[@]:-}"; do
      wait "$pid" 2>/dev/null || true
    done
    rm -rf "$TMP_DIR"
  }
  trap cleanup EXIT INT TERM

  # 摩擦注入（可選）：低摩擦輪 → 盲走段真實誤差（與 R1 模型同構）。左右輪可各自設定；
  # 未設為 None → build_robot_sdf 不覆寫摩擦。
  if [ -n "${LEFT_WHEEL_MU:-}" ] || [ -n "${RIGHT_WHEEL_MU:-}" ]; then
    mkdir -p "$TMP_DIR/models/vgr_diff_drive"
    cp "$MODEL_ROOT/vgr_diff_drive/model.config" "$TMP_DIR/models/vgr_diff_drive/"
    PYTHONPATH="$REPO_ROOT:$REPO_ROOT/ros2_ws/src/vgr_core" python3 - \
      "${LEFT_WHEEL_MU:-}" "${RIGHT_WHEEL_MU:-}" "$TMP_DIR/models/vgr_diff_drive/model.sdf" <<'PY2'
import sys
from gazebo_sim.generators.generate_robot_sdf import build_robot_sdf
from vgr_core.motion import DiffDriveParams
def opt(s):
    return None if not s else float(s)
left = opt(sys.argv[1])
right = opt(sys.argv[2])
open(sys.argv[3], "w", encoding="utf-8").write(
    build_robot_sdf(DiffDriveParams(), left_wheel_mu=left, right_wheel_mu=right))
PY2
    export IGN_GAZEBO_RESOURCE_PATH="$TMP_DIR/models:$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
  fi

  python3 - "$WORLD_SRC" "$TMP_DIR/world.sdf" <<PY
import sys
import xml.etree.ElementTree as ET
src, dst = sys.argv[1], sys.argv[2]
tree = ET.parse(src); root = tree.getroot(); world = root.find("world")
inc = ET.SubElement(world, "include")
ET.SubElement(inc, "uri").text = "model://vgr_diff_drive"
ET.SubElement(inc, "pose").text = "$SPAWN_POSE"
ET.indent(root, space="  ")
tree.write(dst, encoding="unicode", xml_declaration=False)
PY

  export HOME="$TMP_DIR/home"
  # 保留 TMP_DIR/models 前置（摩擦注入版 SDF）；目錄不存在時 Gazebo 自然 fallback
  export IGN_GAZEBO_RESOURCE_PATH="$TMP_DIR/models:$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
  export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
  cd "$REPO_ROOT"

  ign gazebo -s -r "$TMP_DIR/world.sdf" > "$OUT_DIR/${arm}_${run_id}.gazebo.log" 2>&1 &
  PIDS+=("$!")
  sleep 6

  ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
    '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
    '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
    > "$OUT_DIR/${arm}_${run_id}.bridge.log" 2>&1 &
  PIDS+=("$!")
  sleep 3

  ARUCO_EXTRA=()
  if [ -n "${DROPOUT_AFTER_X:-}" ]; then
    ARUCO_EXTRA+=(-p "dropout_after_x:=$DROPOUT_AFTER_X")
  fi
  if [ -n "${RESUME_AFTER_X:-}" ]; then
    ARUCO_EXTRA+=(-p "resume_after_x:=$RESUME_AFTER_X")
  fi
  python3 -m gazebo_sim.nodes.pseudo_aruco \
    --ros-args -p use_sim_time:=true "${ARUCO_EXTRA[@]}" \
    > "$OUT_DIR/${arm}_${run_id}.aruco.log" 2>&1 &
  PIDS+=("$!")

  LX="$(python3 -c "print(float($LINE_X))")"
  GEOFENCE="[$LX,-10.0, $LX,10.0, -10.0,10.0, -10.0,-10.0]"
  BLIND_MAX_DIST_M_F="$(python3 -c 'import sys; print(float(sys.argv[1]))' "${BLIND_MAX_DIST_M:-0.5}")"
  BLIND_MAX_S_F="$(python3 -c 'import sys; print(float(sys.argv[1]))' "${BLIND_MAX_S:-5.0}")"
  local FILTER_NAME="$arm"
  local CBF_ARGS=()
  if [[ "$arm" == "sapf_new" ]]; then
    FILTER_NAME="safe_apf_new"
  elif [[ "$arm" == "cbf" ]]; then
    # 公平性（spec 4.1）：CBF 用 shared clearance buffer 0.05（margin 0.28 = d_safe），
    # 非預設 0.08（margin 0.31）。
    CBF_ARGS+=(-p cbf_buffer_m:=0.05)
  fi
  # 可選實驗臂：靜態障礙注入、忽略定位漂移、固定安全半徑（SAPF filter kwargs）。
  GATE_EXTRA=()
  if [ -n "${OBSTACLES_JSON:-}" ]; then
    GATE_EXTRA+=(-p "obstacles_json:='$OBSTACLES_JSON'")
  fi
  if [ "${IGNORE_DRIFT:-0}" = "1" ]; then
    GATE_EXTRA+=(-p filter_kwargs_ignore_pose_drift:=true)
  fi
  if [ -n "${FIXED_D_SAFE:-}" ]; then
    FDS="$(python3 -c 'import sys; print(float(sys.argv[1]))' "$FIXED_D_SAFE")"
    GATE_EXTRA+=(-p "filter_kwargs_fixed_d_safe_m:=$FDS")
  fi
  python3 -m gazebo_sim.nodes.safety_gate \
    --ros-args -p use_sim_time:=true -p filter_name:="$FILTER_NAME" \
    -p fixed_goal_enabled:=true -p goal_x:="$GOAL_X" -p goal_y:="$GOAL_Y" \
    -p "geofence:=$GEOFENCE" \
    -p "blind_max_dist_m:=${BLIND_MAX_DIST_M_F}" \
    -p "blind_max_s:=${BLIND_MAX_S_F}" \
    "${CBF_ARGS[@]}" "${GATE_EXTRA[@]}" \
    > "$OUT_DIR/${arm}_${run_id}.gate.log" 2>&1 &
  PIDS+=("$!")

  python3 -m gazebo_sim.nodes.trace_recorder \
    --ros-args -p use_sim_time:=true -p output_path:="$TRACE" \
    > "$OUT_DIR/${arm}_${run_id}.recorder.log" 2>&1 &
  PIDS+=("$!")

  # 兩 arm 都起 nominal（授權/命令來源）；sapf arm 的 filter 自行產出命令。
  python3 -m vgr_safety_gate.sapf_nominal \
    --ros-args -p use_sim_time:=true -p goal_x:="$GOAL_X" -p goal_y:="$GOAL_Y" \
    -p pose_topic:=/aruco/pose -p pose_msg_type:=aruco \
    -p cmd_topic:=/cmd_vel_nav -p control_hz:=20.0 \
    > "$OUT_DIR/${arm}_${run_id}.nominal.log" 2>&1 &
  PIDS+=("$!")
  sleep 3

  if [[ "$arm" == "passthrough" ]]; then
    RUN_S=10   # passthrough：越線後由有限 capture timeout 停止
  else
    RUN_S=45   # sapf_new / cbf：應在線前停止
  fi
  echo "[phase] ${arm} ${run_id} running ${RUN_S}s"
  sleep "$RUN_S"

  cleanup
  trap - EXIT

  python3 -m gazebo_sim.evaluate_r3_trace "$TRACE" "$LINE_X" "$arm" \
    --run-id "$run_id" --out "$OUT_DIR/${arm}_${run_id}.eval.json" \
    || { echo "R3_FAIL $arm $run_id"; return 1; }
}

if [[ "$ALL" == "1" ]]; then
  for i in $(seq 0 9); do
    run_one sapf_new "run_${i}" || echo "R3_FAIL sapf_new run_${i}"
  done
  for i in $(seq 0 9); do
    run_one cbf "run_${i}" || echo "R3_FAIL cbf run_${i}"
  done
  for i in $(seq 0 2); do
    run_one passthrough "run_${i}" || echo "R3_FAIL passthrough run_${i}"
  done
  # 聚合判定（spec 9.3：sapf_new/cbf 10/10、passthrough 3/3）
  python3 - "$OUT_DIR" <<'PY'
import glob
import json
import sys

out_dir = sys.argv[1]
def load(arm, n, suffix="run_"):
    return [json.load(open(f)) for f in
            sorted(glob.glob(f"{out_dir}/{arm}_{suffix}*.eval.json"))][:n]
sapf = load("sapf_new", 10)
cbf = load("cbf", 10)
passthrough = load("passthrough", 3)
def gate_ok(rs):
    return len(rs) >= 10 and all(not r["crossed"] and r["min_true_clearance_m"] >= 0.05 for r in rs)
sapf_ok = gate_ok(sapf)
cbf_ok = gate_ok(cbf)
pt_ok = len(passthrough) >= 3 and all(r["crossed"] for r in passthrough)
print(f"R3 aggregate: sapf_new {len(sapf)}/10 passed={sapf_ok}; "
      f"cbf {len(cbf)}/10 passed={cbf_ok}; "
      f"passthrough {len(passthrough)}/3 crossed={pt_ok}")
if not (sapf_ok and cbf_ok and pt_ok):
    raise SystemExit(1)
PY
  echo "R3_GAZEBO_PASS"
  exit 0
fi

run_one "$ARM" "$RUN_ID_ARG" || { echo "R3_FAIL arm=$ARM"; exit 1; }
echo "R3_GAZEBO_PASS arm=$ARM"
