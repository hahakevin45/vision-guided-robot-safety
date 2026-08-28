#!/bin/bash
# Deterministic Nav2 -> safety_gate -> Gazebo obstacle-navigation acceptance.

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((RANDOM % 100 + 100))}
export IGN_PARTITION=${IGN_PARTITION:-"vgr_nav2_$$"}

if [ "${VGR_NAV2_TIMEOUT_INNER:-0}" != "1" ]; then
  VGR_NAV2_TIMEOUT_INNER=1 timeout 180 "$0" "$@"
  status=$?
  if [ "$status" -eq 124 ] || [ "$status" -eq 137 ]; then
    echo "NAV2_FAIL reason=outer_timeout"
  fi
  exit "$status"
fi

source /opt/ros/humble/setup.bash
set -euo pipefail

ODOM_MODE="${1:-ground_truth}"
POSE_SOURCE="${2:-pseudo}"
case "$ODOM_MODE" in ground_truth|wheel_odom) ;; *) echo "expected ground_truth or wheel_odom" >&2; exit 2 ;; esac
case "$POSE_SOURCE" in pseudo|vision) ;; *) echo "expected pseudo or vision" >&2; exit 2 ;; esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_nav2.world"
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
OUTPUT_DIR="$REPO_ROOT/outputs/nav2"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$OUTPUT_DIR/${ODOM_MODE}_${POSE_SOURCE}_${STAMP}.json"
TMP_DIR="$(mktemp -d)"
WORLD_TMP="$TMP_DIR/vgr_nav2.world"
PIDS=()
mkdir -p "$OUTPUT_DIR"

cleanup() {
  local pid i
  set +e
  timeout 3 ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
  for pid in "${PIDS[@]}"; do
    kill -- "-$pid" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    for i in 1 2 3 4 5; do kill -0 -- "-$pid" 2>/dev/null || break; sleep 0.2; done
    kill -9 -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

command -v ign >/dev/null || { echo "NAV2_FAIL reason=missing_ign"; exit 1; }
command -v ros2 >/dev/null || { echo "NAV2_FAIL reason=missing_ros2"; exit 1; }

cd "$REPO_ROOT"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export IGN_GAZEBO_RESOURCE_PATH="$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

python3 -m gazebo_sim.generators.generate_nav2_assets >/dev/null
colcon --log-base "$TMP_DIR/log" build --base-paths ros2_ws/src --symlink-install \
  --build-base "$TMP_DIR/build" --install-base "$TMP_DIR/install" >/dev/null
set +u
source "$TMP_DIR/install/setup.bash"
set -u

python3 - "$WORLD_SRC" "$WORLD_TMP" <<'PY'
import sys
import xml.etree.ElementTree as ET

tree = ET.parse(sys.argv[1])
root = tree.getroot()
world = root.find("world")
include = ET.SubElement(world, "include")
ET.SubElement(include, "uri").text = "model://vgr_diff_drive"
ET.SubElement(include, "pose").text = "0.7 0 0 0 0 0"
ET.indent(root, space="  ")
tree.write(sys.argv[2], encoding="unicode")
PY

GAZEBO_ARGS=(-s -r)
if [ "$POSE_SOURCE" = "vision" ]; then GAZEBO_ARGS+=(--headless-rendering); fi
setsid ign gazebo "${GAZEBO_ARGS[@]}" "$WORLD_TMP" >"$OUTPUT_DIR/${STAMP}.gazebo.log" 2>&1 &
PIDS+=("$!")
sleep 6

BRIDGE_TOPICS=(
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
  '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
  '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist'
)
if [ "$ODOM_MODE" = "wheel_odom" ]; then
  BRIDGE_TOPICS+=('/world/vgr_nav2/model/vgr_diff_drive/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model')
fi
if [ "$POSE_SOURCE" = "vision" ]; then
  BRIDGE_TOPICS+=('/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image')
fi
setsid ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_TOPICS[@]}" --ros-args \
  -r /sim/true_pose:=/sim/true_pose_raw \
  -r /world/vgr_nav2/model/vgr_diff_drive/joint_state:=/joint_states \
  >"$OUTPUT_DIR/${STAMP}.bridge.log" 2>&1 &
PIDS+=("$!")
sleep 3

if [ "$POSE_SOURCE" = "vision" ]; then
  setsid python3 -m gazebo_sim.nodes.aruco_detector --ros-args -p use_sim_time:=true \
    -p marker_map_path:="$REPO_ROOT/gazebo_sim/models/markers/nav2_marker_map.json" \
    >"$OUTPUT_DIR/${STAMP}.aruco.log" 2>&1 &
else
  setsid python3 -m gazebo_sim.nodes.pseudo_aruco --ros-args -p use_sim_time:=true \
    -r /sim/true_pose:=/sim/true_pose_raw >"$OUTPUT_DIR/${STAMP}.aruco.log" 2>&1 &
fi
PIDS+=("$!")

setsid python3 -m gazebo_sim.nodes.safety_gate --ros-args -p use_sim_time:=true \
  -p filter_name:=safe_apf >"$OUTPUT_DIR/${STAMP}.safety.log" 2>&1 &
PIDS+=("$!")

setsid ros2 launch vgr_nav2_bringup navigation.launch.py odom_mode:="$ODOM_MODE" \
  >"$OUTPUT_DIR/${STAMP}.nav2.log" 2>&1 &
PIDS+=("$!")
sleep 8

set +e
python3 -m nav2_integration.acceptance --start-x 0.7 --start-y 0.0 \
  --goal-x 3.5 --goal-y 0.0 --goal-yaw -0.5 \
  --timeout-s 90 --report "$REPORT"
ACCEPTANCE_STATUS=$?
set -e

echo "report=$REPORT"
if [ "$ACCEPTANCE_STATUS" -eq 0 ]; then
  echo "NAV2_PASS mode=$ODOM_MODE pose_source=$POSE_SOURCE"
  exit 0
fi
echo "NAV2_FAIL mode=$ODOM_MODE pose_source=$POSE_SOURCE"
exit 1
