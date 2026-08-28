#!/bin/bash
# G4：重開機修復 NVIDIA/EGL 後的一鍵 vision 整合煙霧測試。
# 這支腳本需要真 Gazebo headless rendering；目前本機驅動錯位時不要執行。

# 隔離每次 smoke 的 ROS DDS domain 與 gz-transport partition。
# 實測跨執行殘留程序會污染 DDS/gz-transport graph，讓 G4-2 收不到 topic；
# ign gazebo 與 ign topic CLI 都繼承同一個 IGN_PARTITION 才看得到彼此。
export ROS_DOMAIN_ID=$((RANDOM % 100 + 100))
export IGN_PARTITION="g4smoke_$$"

if [ "${G4_SMOKE_TIMEOUT_INNER:-0}" != "1" ]; then
  G4_SMOKE_TIMEOUT_INNER=1 timeout 180 "$0" "$@"
  status="$?"
  if [ "$status" -eq 124 ] || [ "$status" -eq 137 ]; then
    echo "[G4-timeout] FAIL 全程 timeout 保底觸發"
    exit 1
  fi
  exit "$status"
fi

# 注意：ROS setup 會讀到未設定變數；必須在 set -u 之前 source。
source /opt/ros/humble/setup.bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_DIR="$REPO_ROOT/gazebo_sim/scripts"
WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_arena_vision.world"
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
OUTPUT_DIR="$REPO_ROOT/outputs/gazebo"
STAMP="$(date +%Y%m%d_%H%M%S)"
TMP_DIR="$(mktemp -d)"
WORLD_TMP="$TMP_DIR/g4_vision_smoke.world"
PIDS=()

mkdir -p "$OUTPUT_DIR"

cleanup() {
  local pid exact_pid
  for pid in "${PIDS[@]}"; do
    exact_pid="$(ps -p "$pid" -o pid= 2> /dev/null | awk -v want="$pid" '$1 == want { print $1 }')"
    if [ -n "$exact_pid" ]; then
      kill "$exact_pid" 2> /dev/null || true
      wait "$exact_pid" 2> /dev/null || true
    fi
  done
  PIDS=()
}

finish_cleanup() {
  cleanup
  rm -rf "$TMP_DIR"
}
trap finish_cleanup EXIT

fail() {
  local stage="$1"
  local message="$2"
  echo "$stage FAIL $message"
  exit 1
}

ok() {
  local stage="$1"
  echo "$stage OK"
}

command -v ign > /dev/null 2>&1 || fail "[G4-preflight]" "找不到 ign CLI"
command -v ros2 > /dev/null 2>&1 || fail "[G4-preflight]" "找不到 ros2 CLI"
command -v timeout > /dev/null 2>&1 || fail "[G4-preflight]" "找不到 timeout CLI"

python3 - "$WORLD_SRC" "$WORLD_TMP" <<'PY'
import sys
import xml.etree.ElementTree as ET

src, dst = sys.argv[1], sys.argv[2]
tree = ET.parse(src)
root = tree.getroot()
world = root.find("world")
if world is None:
    raise SystemExit("world element not found")

include = ET.SubElement(world, "include")
ET.SubElement(include, "uri").text = "model://vgr_diff_drive"
# 起點朝東牆，距東牆約 1.5m；G4-2 在近距離驗證定位精度。
ET.SubElement(include, "pose").text = "2.5 0 0 0 0 0"

ET.indent(root, space="  ")
tree.write(dst, encoding="unicode", xml_declaration=False)
PY

export IGN_GAZEBO_RESOURCE_PATH="$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$TMP_DIR/home"
export HOME="$TMP_DIR/home"

cd "$REPO_ROOT"

start_gazebo() {
  ign gazebo -s -r --headless-rendering "$WORLD_TMP" \
    > "$OUTPUT_DIR/G4_vision_smoke_${STAMP}.gazebo.log" 2>&1 &
  PIDS+=("$!")
  sleep 8
}

start_bridge() {
  ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
    '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
    '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
    '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image' \
    > "$OUTPUT_DIR/G4_vision_smoke_${STAMP}.bridge.log" 2>&1 &
  PIDS+=("$!")
  sleep 3
}

start_detector() {
  python3 -m gazebo_sim.nodes.aruco_detector \
    --ros-args -p use_sim_time:=true \
    > "$OUTPUT_DIR/G4_vision_smoke_${STAMP}.aruco.log" 2>&1 &
  PIDS+=("$!")
  sleep 3
}

echo "[G4-1] 渲染煙霧：確認 /camera/image_raw 影像頻率 > 5 Hz"
start_gazebo
HZ_LOG="$TMP_DIR/camera_hz.log"
set +e
timeout 8 ign topic -hz -t /camera/image_raw > "$HZ_LOG" 2>&1
set -e
python3 - "$HZ_LOG" <<'PY' || fail "[G4-1]" "ign topic 未量到 > 5 Hz 的 /camera/image_raw；log=$HZ_LOG gazebo_log=$OUTPUT_DIR/G4_vision_smoke_${STAMP}.gazebo.log"
import re
import sys

text = open(sys.argv[1], encoding="utf-8", errors="ignore").read()
values = [float(match) for match in re.findall(r"[-+]?(?:\d+\.\d+|\d+)", text)]
if not values or max(values) <= 5.0:
    raise SystemExit(1)
PY
ok "[G4-1]"

echo "[G4-2] 偵測煙霧：bridge + aruco_detector，/aruco/pose 對 ground truth 誤差 < 0.15 m"
start_bridge
start_detector
POSE_OUT="$TMP_DIR/aruco_pose.txt"
GT_OUT="$TMP_DIR/true_pose.txt"
timeout 12 ros2 topic echo --once /aruco/pose --field pose.position > "$POSE_OUT" \
  || fail "[G4-2]" "未收到 /aruco/pose；log=$OUTPUT_DIR/G4_vision_smoke_${STAMP}.aruco.log bridge_log=$OUTPUT_DIR/G4_vision_smoke_${STAMP}.bridge.log"
timeout 12 ros2 topic echo --once /sim/true_pose --field pose.pose.position > "$GT_OUT" \
  || fail "[G4-2]" "未收到 /sim/true_pose；log=$OUTPUT_DIR/G4_vision_smoke_${STAMP}.bridge.log gazebo_log=$OUTPUT_DIR/G4_vision_smoke_${STAMP}.gazebo.log"
python3 - "$POSE_OUT" "$GT_OUT" <<'PY' || fail "[G4-2]" "/aruco/pose 與 ground truth 的 x,y 誤差 >= 0.15 m；pose=$POSE_OUT gt=$GT_OUT aruco_log=$OUTPUT_DIR/G4_vision_smoke_${STAMP}.aruco.log"
import math
import re
import sys

def xy(path: str) -> tuple[float, float]:
    text = open(path, encoding="utf-8", errors="ignore").read()
    found = dict((name, float(value)) for name, value in re.findall(r"\b([xyz]):\s*([-+0-9.eE]+)", text))
    return found["x"], found["y"]

px, py = xy(sys.argv[1])
gx, gy = xy(sys.argv[2])
if math.hypot(px - gx, py - gy) >= 0.15:
    raise SystemExit(1)
PY
ok "[G4-2]"

# 注意：四面牆都有 marker，原地旋轉任何角度仍會看到某面牆（實測轉 180 度
# 看到西牆 ID 2,3 繼續定位）。自然丟失的正確做法是貼近東牆：距牆 < 0.4 m
# 時 marker（y=±1/3）超出 hfov/2 ≈ 34 度而出視野。
echo "[G4-3] 自然丟失：貼近東牆使 marker 出視野，確認 /aruco/pose 3 秒無新訊息"
timeout 8.2 ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.0}}' > /dev/null 2>&1 || true
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' > /dev/null 2>&1 || true
sleep 1
set +e
timeout 3 ros2 topic echo --once /aruco/pose > "$TMP_DIR/lost_pose.txt" 2>&1
POSE_STATUS="$?"
set -e
if [ "$POSE_STATUS" -eq 0 ]; then
  fail "[G4-3]" "貼近東牆後 /aruco/pose 仍有新訊息；log=$TMP_DIR/lost_pose.txt aruco_log=$OUTPUT_DIR/G4_vision_smoke_${STAMP}.aruco.log"
fi
ok "[G4-3]"

cleanup

echo "[G4-4] 端到端：vision 模式跑 GS2 nav profile + clamp_watchdog 20 秒並評估不碰撞"
# GS2 vision 起點同樣在 x=2.5：車向東牆前進，靠近後 marker 出視野，
# 位姿自然丟失，安全層應停車。
export VGR_SPAWN_POSE="2.5 0 0 0 0 0"
timeout 80 "$SCRIPT_DIR/run_gs_scenario.sh" GS2 clamp_watchdog vision \
  > "$OUTPUT_DIR/G4_vision_smoke_${STAMP}.gs2.log" 2>&1 \
  || fail "[G4-4]" "GS2 vision clamp_watchdog 評估失敗，詳見 outputs/gazebo/G4_vision_smoke_${STAMP}.gs2.log"
ok "[G4-4]"

echo "G4_SMOKE_OK"
exit 0
