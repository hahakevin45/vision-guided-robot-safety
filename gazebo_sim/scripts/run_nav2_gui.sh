#!/bin/bash
# Interactive Nav2 demo: Gazebo + RViz, using the same safe command path.
source /opt/ros/humble/setup.bash
set -euo pipefail

ODOM_MODE="${1:-ground_truth}"
POSE_SOURCE="${2:-pseudo}"
case "$ODOM_MODE" in ground_truth|wheel_odom) ;; *) exit 2 ;; esac
case "$POSE_SOURCE" in pseudo|vision) ;; *) exit 2 ;; esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TMP_DIR="$(mktemp -d)"
PIDS=()
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export IGN_PARTITION=${IGN_PARTITION:-"vgr_nav2_gui_$$"}
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export IGN_GAZEBO_RESOURCE_PATH="$REPO_ROOT/gazebo_sim/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

cleanup() {
  set +e
  timeout 3 ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
  for pid in "${PIDS[@]}"; do kill -- "-$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do
    kill -9 -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

cd "$REPO_ROOT"
python3 -m gazebo_sim.generators.generate_nav2_assets >/dev/null
colcon --log-base "$TMP_DIR/log" build --base-paths ros2_ws/src --symlink-install \
  --build-base "$TMP_DIR/build" --install-base "$TMP_DIR/install" >/dev/null
set +u
source "$TMP_DIR/install/setup.bash"
set -u

python3 - "$REPO_ROOT/gazebo_sim/worlds/vgr_nav2.world" "$TMP_DIR/vgr_nav2.world" <<'PY'
import sys
import xml.etree.ElementTree as ET
tree = ET.parse(sys.argv[1]); root = tree.getroot(); world = root.find("world")
include = ET.SubElement(world, "include"); ET.SubElement(include, "uri").text = "model://vgr_diff_drive"
ET.SubElement(include, "pose").text = "0.7 0 0 0 0 0"; tree.write(sys.argv[2], encoding="unicode")
PY

setsid ign gazebo -r "$TMP_DIR/vgr_nav2.world" >/tmp/vgr_nav2_gui_gazebo.log 2>&1 & PIDS+=("$!")
sleep 8
BRIDGE_TOPICS=(
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
  '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
  '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist'
  '/world/vgr_nav2/model/vgr_diff_drive/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
)
if [ "$POSE_SOURCE" = "vision" ]; then BRIDGE_TOPICS+=('/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'); fi
setsid ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_TOPICS[@]}" --ros-args \
  -r /sim/true_pose:=/sim/true_pose_raw \
  -r /world/vgr_nav2/model/vgr_diff_drive/joint_state:=/joint_states \
  >/tmp/vgr_nav2_gui_bridge.log 2>&1 & PIDS+=("$!")
sleep 3
if [ "$POSE_SOURCE" = "vision" ]; then
  setsid python3 -m gazebo_sim.nodes.aruco_detector --ros-args -p use_sim_time:=true \
    -p marker_map_path:="$REPO_ROOT/gazebo_sim/models/markers/nav2_marker_map.json" \
    >/tmp/vgr_nav2_gui_aruco.log 2>&1 &
else
  setsid python3 -m gazebo_sim.nodes.pseudo_aruco --ros-args -p use_sim_time:=true \
    -r /sim/true_pose:=/sim/true_pose_raw >/tmp/vgr_nav2_gui_aruco.log 2>&1 &
fi
PIDS+=("$!")
setsid python3 -m gazebo_sim.nodes.safety_gate --ros-args -p use_sim_time:=true \
  -p filter_name:=safe_apf >/tmp/vgr_nav2_gui_safety.log 2>&1 & PIDS+=("$!")
setsid ros2 launch vgr_nav2_bringup navigation.launch.py odom_mode:="$ODOM_MODE" \
  >/tmp/vgr_nav2_gui_nav2.log 2>&1 & PIDS+=("$!")
sleep 5
setsid rviz2 -d "$TMP_DIR/install/vgr_nav2_bringup/share/vgr_nav2_bringup/rviz/nav2.rviz" & PIDS+=("$!")
echo "Use RViz Nav2 Goal to send a target; all motion goes through /cmd_vel_safe."
wait "${PIDS[-1]}"
