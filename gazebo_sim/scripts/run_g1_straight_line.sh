#!/bin/bash
# G1：Gazebo Fortress 無頭直行驗收。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_arena.world"
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
TMP_DIR="$(mktemp -d)"
WORLD_TMP="$TMP_DIR/vgr_arena_g1.world"
SERVER_LOG="$TMP_DIR/ign_gazebo_server.log"
SERVER_PID=""

cleanup() {
  if [ -n "$SERVER_PID" ] && kill -0 "$SERVER_PID" 2> /dev/null; then
    kill "$SERVER_PID" 2> /dev/null || true
    wait "$SERVER_PID" 2> /dev/null || true
  fi
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

fail() {
  echo "G1_FAIL"
  if [ -s "$SERVER_LOG" ]; then
    tail -80 "$SERVER_LOG" >&2 || true
  fi
  exit 1
}

if ! command -v ign > /dev/null 2>&1; then
  echo "找不到 ign CLI" >&2
  fail
fi

# 使用 world <include> + IGN_GAZEBO_RESOURCE_PATH：
# 起點與模型 URI 在啟動前就固定，避免 runtime create service 的時序不確定性。
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
uri = ET.SubElement(include, "uri")
uri.text = "model://vgr_diff_drive"
pose = ET.SubElement(include, "pose")
pose.text = "0.5 0 0 0 0 0"

ET.indent(root, space="  ")
tree.write(dst, encoding="unicode", xml_declaration=False)
PY

export IGN_GAZEBO_RESOURCE_PATH="$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

# 讓 Gazebo/Ignition runtime 檔案寫到臨時目錄；CI 或 sandbox 常不允許寫入真實 HOME。
mkdir -p "$TMP_DIR/home"
export HOME="$TMP_DIR/home"

# 保留字面指令供驗收檢查：ign gazebo -s -r
timeout 120 ign gazebo -s -r "$WORLD_TMP" > "$SERVER_LOG" 2>&1 &
SERVER_PID="$!"

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + 30))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ! kill -0 "$SERVER_PID" 2> /dev/null; then
      echo "Gazebo server 提早結束" >&2
      fail
    fi
    if ign topic -l 2> /dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    sleep 0.5
  done
  echo "等待 topic 逾時：$topic" >&2
  fail
}

read_pose_x() {
  local sample="$TMP_DIR/true_pose_$(date +%s%N).json"
  timeout 15 ign topic -e --json-output -t /sim/true_pose -n 1 > "$sample" 2> /dev/null || {
    echo "讀取 /sim/true_pose 逾時" >&2
    fail
  }
  python3 - "$sample" <<'PY'
import json
import re
import sys

text = open(sys.argv[1], encoding="utf-8").read()

def find_x(value):
    if isinstance(value, dict):
        if "position" in value and isinstance(value["position"], dict) and "x" in value["position"]:
            return float(value["position"]["x"])
        for child in value.values():
            found = find_x(child)
            if found is not None:
                return found
    elif isinstance(value, list):
        for child in value:
            found = find_x(child)
            if found is not None:
                return found
    return None

for line in text.splitlines():
    line = line.strip()
    if not line:
        continue
    try:
        parsed = json.loads(line)
    except json.JSONDecodeError:
        continue
    found = find_x(parsed)
    if found is not None:
        print(found)
        raise SystemExit(0)

match = re.search(r"position\s*\{[^{}]*\bx:\s*([-+0-9.eE]+)", text, re.S)
if match:
    print(float(match.group(1)))
    raise SystemExit(0)

raise SystemExit("pose x not found")
PY
}

publish_twist() {
  local linear_x="$1"
  local duration_s="$2"
  # ign topic 的 -d/--duration 只作用於 echo，對 publish 無效（實測發一次就返回）。
  # DiffDrive 會保持最後收到的命令，所以發一次後 sleep 等待即可。
  timeout 20 ign topic -t /cmd_vel_safe -m ignition.msgs.Twist \
    -p "linear: {x: ${linear_x}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" \
    > /dev/null
  sleep "$duration_s"
}

wait_for_topic "/sim/true_pose"
# /cmd_vel_safe 是 DiffDrive 的訂閱端，沒有 publisher 前不會出現在 ign topic -l，
# 不能當就緒判定；等 /sim/true_pose（OdometryPublisher 發佈端）就足夠。

START_X="$(read_pose_x)"
publish_twist "0.15" "7.0"
publish_twist "0.0" "0.5"
sleep 0.5
END_X="$(read_pose_x)"

if grep -q "Failed to load system plugin" "$SERVER_LOG"; then
  echo "Gazebo plugin 載入失敗" >&2
  fail
fi

python3 - "$START_X" "$END_X" <<'PY' || fail
import sys

start_x = float(sys.argv[1])
end_x = float(sys.argv[2])
dx = end_x - start_x
target = 0.15 * 7.0
lower = target * 0.85
upper = target * 1.15
if lower <= dx <= upper:
    print(f"G1 displacement OK: dx={dx:.3f} m")
else:
    print(f"G1 displacement out of range: dx={dx:.3f} m, expected {lower:.3f}..{upper:.3f}", file=sys.stderr)
    raise SystemExit(1)
PY

echo "G1_OK"
