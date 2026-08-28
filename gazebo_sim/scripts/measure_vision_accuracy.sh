#!/bin/bash
# 量測真 Gazebo render 下 ArUco 視覺定位誤差，輸出給 safety_sim sensor 參數回填。

# 隔離每次量測的 ROS DDS domain 與 gz-transport partition。
# 跨執行殘留程序會污染 DDS/gz-transport graph；ign gazebo 與 bridge 必須
# 繼承同一個 IGN_PARTITION 才看得到同一輪 topic。
export ROS_DOMAIN_ID=$((RANDOM % 100 + 100))
export IGN_PARTITION="g4measure_$$"

# 注意：ROS setup 會讀到未設定變數；必須在 set -u 之前 source。
source /opt/ros/humble/setup.bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_arena_vision.world"
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
OUTPUT_DIR="$REPO_ROOT/outputs/gazebo"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$OUTPUT_DIR/vision_accuracy_${STAMP}.md"
TMP_DIR="$(mktemp -d)"
PIDS=()

# spawn x: 3.0 2.5 2.0 1.5 1.0 0.5，對應距東牆約 1.0~3.5m。
SPAWN_XS=(3.0 2.5 2.0 1.5 1.0 0.5)

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

command -v ign > /dev/null 2>&1 || { echo "找不到 ign CLI" >&2; exit 2; }
command -v ros2 > /dev/null 2>&1 || { echo "找不到 ros2 CLI" >&2; exit 2; }

export IGN_GAZEBO_RESOURCE_PATH="$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$TMP_DIR/home"
export HOME="$TMP_DIR/home"

cd "$REPO_ROOT"

{
  echo "# Vision Accuracy ${STAMP}"
  echo
  echo "| 距離 m | 偵測 ID | 位姿誤差 m |"
  echo "|---:|---|---:|"
} > "$REPORT"

make_world() {
  local spawn_x="$1"
  local world_tmp="$2"
  python3 - "$WORLD_SRC" "$world_tmp" "$spawn_x" <<'PY'
import sys
import xml.etree.ElementTree as ET

src, dst, spawn_x = sys.argv[1], sys.argv[2], sys.argv[3]
tree = ET.parse(src)
root = tree.getroot()
world = root.find("world")
if world is None:
    raise SystemExit("world element not found")

include = ET.SubElement(world, "include")
ET.SubElement(include, "uri").text = "model://vgr_diff_drive"
ET.SubElement(include, "pose").text = f"{spawn_x} 0 0 0 0 0"

ET.indent(root, space="  ")
tree.write(dst, encoding="unicode", xml_declaration=False)
PY
}

measure_once() {
  local spawn_x="$1"
  local distance_m="$2"

  python3 - "$spawn_x" "$distance_m" <<'PY'
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

from gazebo_sim.nodes.aruco_detector import (
    ArucoWorldLocalizer,
    DEFAULT_CAMERA_INFO_PATH,
    DEFAULT_CAMERA_POSE_ON_ROBOT,
    DEFAULT_MARKER_MAP_PATH,
    load_json,
)

spawn_x = float(sys.argv[1])
distance_m = float(sys.argv[2])
frame: Image | None = None

rclpy.init()
node = rclpy.create_node("vision_accuracy_probe")

def on_image(msg: Image) -> None:
    global frame
    frame = msg

node.create_subscription(Image, "/camera/image_raw", on_image, 10)
deadline = time.monotonic() + 12.0
while frame is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

try:
    if frame is None:
        print(f"| {distance_m:.1f} | frame_timeout | NA |")
        raise SystemExit(0)

    data = np.frombuffer(frame.data, dtype=np.uint8)
    rows = data.reshape((frame.height, frame.step))
    if frame.encoding == "rgb8":
        image_rgb = rows[:, : frame.width * 3].reshape((frame.height, frame.width, 3))
        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    elif frame.encoding == "bgr8":
        image_bgr = rows[:, : frame.width * 3].reshape((frame.height, frame.width, 3))
    else:
        print(f"| {distance_m:.1f} | encoding:{frame.encoding} | NA |")
        raise SystemExit(0)

    localizer = ArucoWorldLocalizer(
        load_json(DEFAULT_MARKER_MAP_PATH),
        load_json(DEFAULT_CAMERA_INFO_PATH),
        DEFAULT_CAMERA_POSE_ON_ROBOT,
    )
    corners, ids = localizer._detect_marker_corners(image_bgr)
    detected = ",".join(str(int(marker_id[0])) for marker_id in ids) if ids is not None else "-"
    pose = localizer.locate(image_bgr)
    if pose is None:
        print(f"| {distance_m:.1f} | {detected} | NA |")
    else:
        error_m = math.hypot(pose.x - spawn_x, pose.y)
        print(f"| {distance_m:.1f} | {detected} | {error_m:.3f} |")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

for spawn_x in "${SPAWN_XS[@]}"; do
  WORLD_TMP="$TMP_DIR/vision_accuracy_${spawn_x}.world"
  make_world "$spawn_x" "$WORLD_TMP"

  ign gazebo -s -r --headless-rendering "$WORLD_TMP" \
    > "$OUTPUT_DIR/vision_accuracy_${STAMP}_${spawn_x}.gazebo.log" 2>&1 &
  PIDS+=("$!")
  sleep 8

  ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
    '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image' \
    > "$OUTPUT_DIR/vision_accuracy_${STAMP}_${spawn_x}.bridge.log" 2>&1 &
  PIDS+=("$!")
  sleep 3

  distance_m="$(python3 - "$spawn_x" <<'PY'
import sys
print(f"{4.0 - float(sys.argv[1]):.1f}")
PY
)"
  measure_once "$spawn_x" "$distance_m" >> "$REPORT"

  cleanup
  sleep 1
done

echo "report=$REPORT"
