#!/bin/bash
# 舊 RPP 停車基準：global planner 看不到箱，local costmap 才收視覺點雲。
# 真正的局部規劃器比較請使用 run_local_detour_compare.sh（SAPF vs DWB）。
#
# 場景：vgr_nav2.world + 不含箱子的 hidden map。
# Arm A（nav2）：盲 NavFn 路徑 + RPP；只驗證跟隨器能否停在障礙前。
# Arm B（sapf）：同一視覺箱幾何 + safety_gate(safe_apf_new)。
#
# RPP 不會自行選擇繞行側，因此本 runner 不用於局部繞行優劣結論。
#
# 用法：
#   bash gazebo_sim/scripts/run_controller_compare.sh --arm nav2 --out /tmp/cmp_nav2
#   bash gazebo_sim/scripts/run_controller_compare.sh --arm sapf --out /tmp/cmp_sapf
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_nav2.world"
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/config/nav2_params.yaml}"
ARM=""
OUT_DIR=""
START_X=0.7
GOAL_X=3.5
DROPOUT_AT=""
DROPOUT_AFTER_X=""
VISUAL_MAX_RANGE="${VISUAL_MAX_RANGE:-3.0}"
NAV2_TIMEOUT_S="${NAV2_TIMEOUT_S:-60}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm) ARM="$2"; shift 2 ;;
    --out) OUT_DIR="$2"; shift 2 ;;
    --dropout-at) DROPOUT_AT="$2"; shift 2 ;;
    --dropout-after-x) DROPOUT_AFTER_X="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$ARM" && -n "$OUT_DIR" ]] || { echo "need --arm nav2|sapf --out DIR" >&2; exit 2; }
[[ "$ARM" == "nav2" || "$ARM" == "sapf" ]] || { echo "arm must be nav2 or sapf" >&2; exit 2; }
mkdir -p "$OUT_DIR"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 100 + 100))}"
export IGN_PARTITION="vgr_cmp_$$"
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="$REPO_ROOT:$REPO_ROOT/ros2_ws/src/vgr_core:$REPO_ROOT/ros2_ws/src/vgr_safety_gate${PYTHONPATH:+:$PYTHONPATH}"
export IGN_GAZEBO_RESOURCE_PATH="$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

TMP_DIR="$(mktemp -d)"
WORLD_TMP="$TMP_DIR/world.sdf"
STAMP="$(date +%Y%m%d_%H%M%S)"
TRACE="$OUT_DIR/${ARM}_${STAMP}.jsonl"
PIDS=()

cleanup() {
  set +e
  for pid in "${PIDS[@]:-}"; do kill -- "-$pid" 2>/dev/null; done
  sleep 2
  for pid in "${PIDS[@]:-}"; do kill -KILL -- "-$pid" 2>/dev/null; done
  for pid in "${PIDS[@]:-}"; do wait "$pid" 2>/dev/null; done
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT INT TERM

cd "$REPO_ROOT"

# 產生 Nav2 資產（world + 隱藏 map + 障礙 marker）
python3 -m gazebo_sim.generators.generate_nav2_assets >/dev/null
python3 - "$WORLD_SRC" "$WORLD_TMP" <<PY
import sys
import xml.etree.ElementTree as ET
tree = ET.parse(sys.argv[1]); root = tree.getroot(); world = root.find("world")
inc = ET.SubElement(world, "include")
ET.SubElement(inc, "uri").text = "model://vgr_diff_drive"
ET.SubElement(inc, "pose").text = "$START_X 0 0 0 0 0"
ET.indent(root, space="  ")
tree.write(sys.argv[2], encoding="unicode")
PY

# colcon build（與 run_nav2_scenario 一致）
colcon --log-base "$TMP_DIR/log" build --base-paths ros2_ws/src --symlink-install \
  --build-base "$TMP_DIR/build" --install-base "$TMP_DIR/install" >/dev/null
set +u
source "$TMP_DIR/install/setup.bash"
set -u

setsid ign gazebo -s -r "$WORLD_TMP" >"$OUT_DIR/${ARM}_${STAMP}.gazebo.log" 2>&1 &
PIDS+=("$!")
sleep 6

setsid ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
  '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
  '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
  '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
  --ros-args -r /sim/true_pose:=/sim/true_pose_raw \
  >"$OUT_DIR/${ARM}_${STAMP}.bridge.log" 2>&1 &
PIDS+=("$!")
sleep 3

ARUCO_EXTRA=()
if [ -n "$DROPOUT_AFTER_X" ]; then
  ARUCO_EXTRA+=(-p "dropout_after_x:=$DROPOUT_AFTER_X")
fi
setsid python3 -m gazebo_sim.nodes.pseudo_aruco --ros-args -p use_sim_time:=true \
  -r /sim/true_pose:=/sim/true_pose_raw "${ARUCO_EXTRA[@]}" \
  >"$OUT_DIR/${ARM}_${STAMP}.aruco.log" 2>&1 &
PIDS+=("$!")

# 權威 marker 幾何；visual_obstacle_scan 只在感測到後發布。
OBSTACLES_JSON='[{"type":"box","x":2.0,"y":0.0,"size_x":0.4,"size_y":0.6}]'
if [ "$ARM" = "sapf" ]; then
  # visual_obstacle_scan 直接發布同一 marker detection 的 Box2D。
  setsid python3 -m gazebo_sim.nodes.safety_gate --ros-args -p use_sim_time:=true \
    -p filter_name:=safe_apf_new -p fixed_goal_enabled:=true \
    -p "goal_x:=$GOAL_X" -p goal_y:=0.0 \
    >"$OUT_DIR/${ARM}_${STAMP}.gate.log" 2>&1 &
  PIDS+=("$!")
else
  setsid python3 -m gazebo_sim.nodes.safety_gate --ros-args -p use_sim_time:=true \
    -p filter_name:=passthrough \
    >"$OUT_DIR/${ARM}_${STAMP}.gate.log" 2>&1 &
  PIDS+=("$!")
fi

setsid python3 -m gazebo_sim.nodes.trace_recorder --ros-args -p use_sim_time:=true \
  -r /sim/true_pose:=/sim/true_pose_raw \
  -p "output_path:=$TRACE" >"$OUT_DIR/${ARM}_${STAMP}.recorder.log" 2>&1 &
PIDS+=("$!")

# 同一視覺節點：PointCloud2 給 Nav2；Box2D 量測給 SAPF gate。
setsid python3 -m gazebo_sim.nodes.visual_obstacle_scan --ros-args \
  -p use_sim_time:=true -p "obstacles_json:='$OBSTACLES_JSON'" \
  -p pose_topic:=/sim/true_pose_raw -p fov_rad:=2.0 \
  -p max_range:="$VISUAL_MAX_RANGE" \
  >"$OUT_DIR/${ARM}_${STAMP}.visob.log" 2>&1 &
PIDS+=("$!")

# 隱藏 map；預設 params 只讓 local costmap 收視覺點雲，實驗可用 NAV2_PARAMS_FILE 覆寫。
HIDDEN_MAP="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/maps/vgr_nav2_hidden.yaml"
if [ "$ARM" = "nav2" ]; then
  export VGR_NAV2_USE_CONTROLLER=1
  setsid ros2 launch vgr_nav2_bringup navigation.launch.py \
    params_file:="$NAV2_PARAMS_FILE" \
    odom_mode:=ground_truth map:="$HIDDEN_MAP" use_controller:=true \
    >"$OUT_DIR/${ARM}_${STAMP}.nav2.log" 2>&1 &
  PIDS+=("$!")
  sleep 10
  # 診斷：Nav2 costmap 實際消費的 odom-frame 點雲是否發布。
  ( sleep 15; timeout 5 ros2 topic echo --once \
    /visual_obstacle_points sensor_msgs/msg/PointCloud2 --field width \
    > "$OUT_DIR/${ARM}_${STAMP}.points_diag.log" 2>&1 ) &
  DIAG_PID=$!
  PIDS+=("$DIAG_PID")
  if [ -n "$DROPOUT_AT" ]; then
    sleep "$DROPOUT_AT"
    timeout 10 ros2 service call /aruco/set_dropout std_srvs/srv/SetBool '{data: true}' >/dev/null 2>&1
  fi
  set +e
  python3 -m nav2_integration.acceptance --start-x "$START_X" --start-y 0.0 \
    --goal-x "$GOAL_X" --goal-y 0.0 --goal-yaw 0.0 \
    --timeout-s "$NAV2_TIMEOUT_S" --report "$OUT_DIR/${ARM}_${STAMP}.eval.json"
  STATUS=$?
  set -e
else
  export VGR_NAV2_USE_CONTROLLER=0
  setsid ros2 launch vgr_nav2_bringup navigation.launch.py \
    params_file:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/config/nav2_params.yaml" \
    odom_mode:=ground_truth map:="$HIDDEN_MAP" use_controller:=false \
    >"$OUT_DIR/${ARM}_${STAMP}.nav2.log" 2>&1 &
  PIDS+=("$!")
  sleep 10
  setsid python3 -m nav2_integration.path_to_plan --ros-args \
    -p use_sim_time:=true -p "start_x:=$START_X" -p start_y:=0.0 \
    -p "goal_x:=$GOAL_X" -p goal_y:=0.0 -p plan_topic:=/plan -p rate_hz:=1.0 \
    >"$OUT_DIR/${ARM}_${STAMP}.plan.log" 2>&1 &
  PIDS+=("$!")
  sleep 4
  if [ -n "$DROPOUT_AT" ]; then
    sleep "$DROPOUT_AT"
    timeout 10 ros2 service call /aruco/set_dropout std_srvs/srv/SetBool '{data: true}' >/dev/null 2>&1
  fi
  # 跑固定時間（SAPF 自己控制運動與停止）
  timeout 60 ros2 topic echo --full-length /safety_gate/status std_msgs/msg/String \
    >"$OUT_DIR/${ARM}_${STAMP}.status.log" 2>&1 &
  PIDS+=("$!")
  sleep 60
  STATUS=0
fi

cleanup
trap - EXIT

# 評估：到達 goal？最小淨空（world 幾何，含箱）？
export PYTHONPATH="$REPO_ROOT:$REPO_ROOT/ros2_ws/src/vgr_core:$REPO_ROOT/ros2_ws/src/vgr_safety_gate"
python3 - "$TRACE" "$GOAL_X" <<'PY'
import json
import sys

from gazebo_sim.evaluate_local_detour import evaluate_detour_trace

trace_path, goal_x = sys.argv[1], float(sys.argv[2])
result = evaluate_detour_trace(
    trace_path,
    box={"x": 2.0, "y": 0.0, "size_x": 0.4, "size_y": 0.6},
    goal=(goal_x, 0.0),
)
print(json.dumps(result, indent=1, sort_keys=True))
PY
echo "compare done: arm=$ARM trace=$TRACE"