#!/bin/bash
# GS1/GS2：Gazebo + ROS2 safety stack 一鍵驗收。
# 注意：ROS setup 會讀到未設定變數；必須在 set -u 之前 source。
source /opt/ros/humble/setup.bash
set -euo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
  echo "用法：$0 <GS1|GS2|GS3> <filter_name> [pseudo|vision]" >&2
  exit 2
fi

SCENARIO="$1"
FILTER_NAME="$2"
POSE_SOURCE="${3:-pseudo}"
case "$SCENARIO" in
  GS1) PROFILE="gs1_wall_rush"; RUN_BEFORE_FAULT_S=35; RUN_AFTER_FAULT_S=0 ;;
  GS2) PROFILE="gs2_blackout"; RUN_BEFORE_FAULT_S=10; RUN_AFTER_FAULT_S=10 ;;
  GS3) PROFILE="gs3_sapf_single_obstacle"; RUN_BEFORE_FAULT_S=120; RUN_AFTER_FAULT_S=0 ;;
  *) echo "未知情境：$SCENARIO（只支援 GS1、GS2 或 GS3）" >&2; exit 2 ;;
esac
case "$POSE_SOURCE" in
  pseudo|vision) ;;
  *) echo "未知位姿來源：$POSE_SOURCE（只支援 pseudo 或 vision）" >&2; exit 2 ;;
esac
if [ "$SCENARIO" = "GS3" ] && [ "$POSE_SOURCE" = "vision" ]; then
  echo "GS3 第一輪只支援 pseudo 位姿（vision 模式尚未驗證）" >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_arena.world"
if [ "$SCENARIO" = "GS3" ]; then
  WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_sapf.world"
elif [ "$POSE_SOURCE" = "vision" ]; then
  WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_arena_vision.world"
fi
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
OUTPUT_DIR="$REPO_ROOT/outputs/gazebo"
STAMP="$(date +%Y%m%d_%H%M%S)"
TRACE="$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.jsonl"
REPORT="$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.eval.json"
TMP_DIR="$(mktemp -d)"
WORLD_TMP="$TMP_DIR/${SCENARIO}_${FILTER_NAME}.world"
PIDS=()
SPAWN_POSE="${VGR_SPAWN_POSE:-0.5 0 0 0 0 0}"

mkdir -p "$OUTPUT_DIR"

cleanup() {
  local pid i
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2> /dev/null; then
      kill "$pid" 2> /dev/null || true
      # ign gazebo 可能無視 SIGTERM（實測卡死過 10 小時）；等 10 秒就
      # 升級 SIGKILL，絕不讓 cleanup 的 wait 卡死整條 pipeline。
      for i in 1 2 3 4 5 6 7 8 9 10; do
        kill -0 "$pid" 2> /dev/null || break
        sleep 1
      done
      if kill -0 "$pid" 2> /dev/null; then
        kill -9 "$pid" 2> /dev/null || true
      fi
      wait "$pid" 2> /dev/null || true
    fi
  done
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

fail_with_logs() {
  echo "GS_FAIL scenario=$SCENARIO filter=$FILTER_NAME"
  echo "trace=$TRACE"
  echo "report=$REPORT"
  exit 1
}

command -v ign > /dev/null 2>&1 || { echo "找不到 ign CLI" >&2; fail_with_logs; }
command -v ros2 > /dev/null 2>&1 || { echo "找不到 ros2 CLI" >&2; fail_with_logs; }

# 使用臨時 world 注入 robot 起點，避免 runtime spawn 的時序不穩定。
python3 - "$WORLD_SRC" "$WORLD_TMP" "$SPAWN_POSE" <<'PY'
import sys
import xml.etree.ElementTree as ET

src, dst, spawn_pose = sys.argv[1], sys.argv[2], sys.argv[3]
tree = ET.parse(src)
root = tree.getroot()
world = root.find("world")
if world is None:
    raise SystemExit("world element not found")

include = ET.SubElement(world, "include")
ET.SubElement(include, "uri").text = "model://vgr_diff_drive"
ET.SubElement(include, "pose").text = spawn_pose

ET.indent(root, space="  ")
tree.write(dst, encoding="unicode", xml_declaration=False)
PY

export IGN_GAZEBO_RESOURCE_PATH="$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"

# 讓 Gazebo/Ignition runtime 檔案寫入臨時 HOME，降低 CI/sandbox 權限問題。
mkdir -p "$TMP_DIR/home"
export HOME="$TMP_DIR/home"

cd "$REPO_ROOT"
rm -f "$TRACE" "$REPORT"

GAZEBO_ARGS=(-s -r)
if [ "$POSE_SOURCE" = "vision" ]; then
  GAZEBO_ARGS+=(--headless-rendering)
fi
GAZEBO_ARGS+=("$WORLD_TMP")

ign gazebo "${GAZEBO_ARGS[@]}" > "$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.gazebo.log" 2>&1 &
PIDS+=("$!")
sleep 6

BRIDGE_TOPICS=(
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
  '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
  '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist'
)
if [ "$POSE_SOURCE" = "vision" ]; then
  BRIDGE_TOPICS+=('/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image')
fi

ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_TOPICS[@]}" \
  > "$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.bridge.log" 2>&1 &
PIDS+=("$!")
sleep 3

if [ "$POSE_SOURCE" = "vision" ]; then
  python3 -m gazebo_sim.nodes.aruco_detector \
    --ros-args -p use_sim_time:=true \
    > "$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.aruco.log" 2>&1 &
else
  python3 -m gazebo_sim.nodes.pseudo_aruco \
    --ros-args -p use_sim_time:=true \
    > "$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.aruco.log" 2>&1 &
fi
PIDS+=("$!")

GATE_EXTRA_ARGS=()
case "$SCENARIO" in
  GS1)
    # 吸引目標放在禁止穿越的東牆上（與 safety_sim S1 world goal 同源）。
    GATE_EXTRA_ARGS+=(-p fixed_goal_enabled:=true -p goal_x:=4.0 -p goal_y:=0.0)
    ;;
  GS2)
    GATE_EXTRA_ARGS+=(-p fixed_goal_enabled:=true -p goal_x:=3.0 -p goal_y:=0.0)
    ;;
  GS3)
    # 固定 goal 與圓柱靜態 map：與 vgr_sapf.world 的幾何同源（SAPF_OBSTACLE）。
    # cbf_buffer_m=0.05：與 SAPF d_safe=0.28 對齊的公平設定（margin 0.28）；
    # gate 只在 filter_name=cbf 時使用，其他 filter 忽略。
    GATE_EXTRA_ARGS+=(
      -p fixed_goal_enabled:=true
      -p goal_x:=3.2
      -p goal_y:=0.0
      -p 'obstacles_json:="[{\"type\":\"box\",\"x\":2.0,\"y\":0.0,\"size_x\":0.4,\"size_y\":0.6}]"'
      -p cbf_buffer_m:=0.05
    )
    ;;
esac

python3 -m gazebo_sim.nodes.safety_gate \
  --ros-args -p use_sim_time:=true -p filter_name:="$FILTER_NAME" \
  "${GATE_EXTRA_ARGS[@]}" \
  > "$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.gate.log" 2>&1 &
PIDS+=("$!")

python3 -m gazebo_sim.nodes.trace_recorder \
  --ros-args -p use_sim_time:=true -p output_path:="$TRACE" \
  > "$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.recorder.log" 2>&1 &
PIDS+=("$!")
sleep 3

python3 -m gazebo_sim.nodes.scripted_nav \
  --ros-args -p use_sim_time:=true -p profile:="$PROFILE" \
  > "$OUTPUT_DIR/${SCENARIO}_${FILTER_NAME}_${STAMP}.nav.log" 2>&1 &
PIDS+=("$!")

echo "[phase] $SCENARIO running ${RUN_BEFORE_FAULT_S}s"
sleep "$RUN_BEFORE_FAULT_S"

if [ "$SCENARIO" = "GS2" ]; then
  if [ "$POSE_SOURCE" = "vision" ]; then
    echo "[phase] GS2 vision 模式跳過 marker dropout service；marker 丟失由視野自然發生"
  else
    echo "[phase] GS2 注入 marker dropout"
    # fault_t0 由 evaluate_gs_trace.py 從 trace 的 pose_age 持續上升段回推；
    # 這比從 shell 讀 /clock 少一個 ROS CLI 競態，也符合 G2 簡化驗收方式。
    timeout 10 ros2 service call /aruco/set_dropout std_srvs/srv/SetBool '{data: true}' > /dev/null
  fi
  echo "[phase] GS2 dropout 後 ${RUN_AFTER_FAULT_S}s"
  sleep "$RUN_AFTER_FAULT_S"
fi

# 先停止節點，確保 recorder flush 完 JSONL，再做離線評估。
cleanup
trap - EXIT

if [ ! -s "$TRACE" ]; then
  echo "trace 不存在或為空：$TRACE" >&2
  fail_with_logs
fi

# recorder 只記 topic event；在離線評估前補 metadata，讓 JSON 報告保留實際 filter。
python3 - "$TRACE" "$SCENARIO" "$FILTER_NAME" <<'PY'
import json
import sys

trace, scenario, filter_name = sys.argv[1:4]
with open(trace, "a", encoding="utf-8") as fh:
    fh.write(json.dumps({
        "topic": "metadata",
        "scenario_name": scenario,
        "filter_name": filter_name,
    }, sort_keys=True) + "\n")
PY

set +e
python3 -m gazebo_sim.evaluate_gs_trace "$TRACE" "$SCENARIO" --output "$REPORT"
EVAL_STATUS="$?"
set -e

python3 - "$REPORT" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
metrics = report["metrics"]
reasons = "; ".join(report["reasons"]) if report["reasons"] else "-"
print(
    "metrics "
    f"min_clearance={metrics['min_clearance']:.3f} "
    f"max_speed={metrics['max_speed_mps']:.3f} "
    f"time_to_stop={metrics['time_to_stop_after_fault_s']} "
    f"intervention_ratio={metrics['intervention_ratio']:.3f} "
    f"cmd_distortion={metrics['cmd_distortion']:.6f} "
    f"reasons={reasons}"
)
PY

echo "trace=$TRACE"
echo "report=$REPORT"
if [ "$EVAL_STATUS" -eq 0 ]; then
  echo "GS_PASS scenario=$SCENARIO filter=$FILTER_NAME"
  exit 0
fi

echo "GS_FAIL scenario=$SCENARIO filter=$FILTER_NAME"
exit 1
