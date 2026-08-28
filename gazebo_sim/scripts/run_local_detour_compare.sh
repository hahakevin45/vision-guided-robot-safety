#!/bin/bash
# 純局部繞行比較：global plan 永遠穿箱，只有局部層看得到視覺障礙。
#   sapf: 穿箱直線 /plan + safety_gate(safe_apf_new)
#   dwb : 隱藏障礙的 NavFn 直線 plan + 完整 DWB controller
# 兩臂共用 visual_obstacle_scan；global costmap 不訂閱障礙。
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
ARM=""; OUT_DIR=""; RUN_SECONDS="${RUN_SECONDS:-90}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm) ARM="$2"; shift 2 ;;
    --out) OUT_DIR="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[[ "$ARM" =~ ^(sapf|dwb)$ ]] || { echo "arm must be sapf|dwb" >&2; exit 2; }
mkdir -p "$OUT_DIR"
if [[ "${DRY_RUN:-}" == "YES" ]]; then echo "DRY_RUN: arm=$ARM out=$OUT_DIR"; exit 0; fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 100 + 100))}"
export IGN_PARTITION="vgr_detour_$$"
set +u; source /opt/ros/humble/setup.bash; source "$REPO_ROOT/ros2_ws/install/setup.bash"; set -u
export PYTHONPATH="$REPO_ROOT:$REPO_ROOT/ros2_ws/src/vgr_core:$REPO_ROOT/ros2_ws/src/vgr_safety_gate${PYTHONPATH:+:$PYTHONPATH}"
export IGN_GAZEBO_RESOURCE_PATH="$MODEL_ROOT"
TMP_DIR="$(mktemp -d)"; STAMP="$(date +%Y%m%d_%H%M%S)"
TRACE="$OUT_DIR/${ARM}_${STAMP}.jsonl"; PIDS=()
cleanup() {
  set +e
  for pid in "${PIDS[@]:-}"; do
    kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
  done
  for pid in "${PIDS[@]:-}"; do wait "$pid" 2>/dev/null; done
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT INT TERM
cd "$REPO_ROOT"

python3 -m gazebo_sim.generators.generate_nav2_assets >/dev/null
# 摩擦注入（可選，與 R3 harness 同構）：LEFT_WHEEL_MU/RIGHT_WHEEL_MU
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
  export IGN_GAZEBO_RESOURCE_PATH="$TMP_DIR/models:$MODEL_ROOT"
fi
python3 - gazebo_sim/worlds/vgr_nav2.world "$TMP_DIR/world.sdf" <<PY
import sys, xml.etree.ElementTree as ET
tree = ET.parse(sys.argv[1]); root = tree.getroot(); world = root.find("world")
inc = ET.SubElement(world, "include")
ET.SubElement(inc, "uri").text = "model://vgr_diff_drive"
ET.SubElement(inc, "pose").text = "0.7 0.05 0 0 0 0"
ET.indent(root, space="  "); tree.write(sys.argv[2], encoding="unicode")
PY

setsid ign gazebo -s -r "$TMP_DIR/world.sdf" >"$OUT_DIR/${ARM}_${STAMP}.gazebo.log" 2>&1 &
PIDS+=("$!"); sleep 6
setsid ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
  '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
  '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
  --ros-args -r /sim/true_pose:=/sim/true_pose_raw >"$OUT_DIR/${ARM}_${STAMP}.bridge.log" 2>&1 &
PIDS+=("$!"); sleep 3
# 定位與兩臂共用的視覺障礙來源。
setsid python3 -m gazebo_sim.nodes.pseudo_aruco --ros-args -p use_sim_time:=true \
  -r /sim/true_pose:=/sim/true_pose_raw >"$OUT_DIR/${ARM}_${STAMP}.aruco.log" 2>&1 &
PIDS+=("$!")
OBSTACLES_JSON='[{"type":"box","x":2.0,"y":0.0,"size_x":0.4,"size_y":0.6}]'
setsid python3 -m gazebo_sim.nodes.visual_obstacle_scan --ros-args \
  -p use_sim_time:=true -p "obstacles_json:='$OBSTACLES_JSON'" \
  -p pose_topic:=/sim/true_pose_raw -p fov_rad:=2.0 \
  >"$OUT_DIR/${ARM}_${STAMP}.visob.log" 2>&1 &
PIDS+=("$!")

# 在任何 planner/controller 動作前開始記錄，避免漏掉起步段。
setsid python3 -m gazebo_sim.nodes.trace_recorder --ros-args -p use_sim_time:=true \
  -r /sim/true_pose:=/sim/true_pose_raw -p "output_path:=$TRACE" \
  >"$OUT_DIR/${ARM}_${STAMP}.recorder.log" 2>&1 &
PIDS+=("$!")

HIDDEN_MAP="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/maps/vgr_nav2_hidden.yaml"
if [[ "$ARM" == "sapf" ]]; then
  setsid python3 -m gazebo_sim.nodes.safety_gate --ros-args -p use_sim_time:=true \
    -p filter_name:=safe_apf_new >"$OUT_DIR/${ARM}_${STAMP}.gate.log" 2>&1 &
  PIDS+=("$!")
  setsid python3 -m nav2_integration.straight_plan_publisher --ros-args \
    -p use_sim_time:=true -p start_x:=0.7 -p goal_x:=3.5 \
    >"$OUT_DIR/${ARM}_${STAMP}.plan.log" 2>&1 &
  PIDS+=("$!")
  sleep "$RUN_SECONDS"
else
  setsid python3 -m gazebo_sim.nodes.safety_gate --ros-args -p use_sim_time:=true \
    -p filter_name:=passthrough >"$OUT_DIR/${ARM}_${STAMP}.gate.log" 2>&1 &
  PIDS+=("$!")
  export VGR_NAV2_USE_CONTROLLER=1
  setsid ros2 launch vgr_nav2_bringup navigation.launch.py \
    params_file:="$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup/config/nav2_params.yaml" \
    map:="$HIDDEN_MAP" odom_mode:=ground_truth use_controller:=true \
    controller_plugin:=dwb >"$OUT_DIR/${ARM}_${STAMP}.nav2.log" 2>&1 &
  PIDS+=("$!")
  sleep 12
  set +e
  python3 -m nav2_integration.acceptance \
    --start-x 0.7 --start-y 0.05 --goal-x 3.5 --goal-y 0.0 --goal-yaw 0.0 \
    --timeout-s "$RUN_SECONDS" --report "$OUT_DIR/${ARM}_${STAMP}.acceptance.json"
  ACTION_STATUS=$?
  set -e
  echo "DWB_ACCEPTANCE_EXIT=$ACTION_STATUS"
fi
cleanup; trap - EXIT
PYTHONPATH="$REPO_ROOT:$REPO_ROOT/ros2_ws/src/vgr_core" python3 - "$TRACE" <<'PY'
import json, sys
from gazebo_sim.evaluate_local_detour import evaluate_detour_trace
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
out = evaluate_detour_trace(rows, box={"x": 2.0, "y": 0.0, "size_x": 0.4, "size_y": 0.6},
                            goal=(3.5, 0.0))
print(json.dumps(out, sort_keys=True))
with open(sys.argv[1] + ".eval.json", "w") as fh:
    json.dump(out, fh, indent=2)
PY
