#!/bin/bash
# 實體斷鏈急停演練（實車版 GS2，SG-A 第 5 步）：命令持續前進中切斷位姿，
# 驗證安全層看門狗把輪子停死。在 Pi 上執行。
#
# 前提：輪子架空、12V 可斷電、現場有人；bridge/Nav2 無殘留程序。
# 判定（2026-07-13 實測校正）：
#   - 斷鏈時刻以 pseudo-pose 節點 log 的 dropout=True 時戳為準——
#     不可用 `ros2 service call` 的呼叫時刻，Pi 上 CLI 冷啟動 >1s 會灌水。
#   - 全鏈路停止 ≤0.60s：0.4s 看門狗門檻＋安全層 20Hz 一拍＋狀態取樣一拍
#     ＋餘裕。實測 0.501s。
#   - 停止後 target 保持零 ≥2s，期間不得再動。
set -o pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/vision_guided_robot/ros2_ws/install/setup.bash"
set -u
cd "$HOME/vision_guided_robot"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" ROS2CLI_NO_DAEMON=1
export PYTHONPATH="$PWD:$PYTHONPATH"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="outputs/nav2_pi"
mkdir -p "$OUT"
PIDS=()

if [ "${VGR_WHEELS_RAISED:-}" != "YES" ]; then
  echo "dropout drill requires VGR_WHEELS_RAISED=YES" >&2
  exit 2
fi

cleanup() {
  set +e
  timeout 2 ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
  sleep 0.5
  local p
  for p in "${PIDS[@]}"; do kill -- "-$p" >/dev/null 2>&1; done
  sleep 2
  for p in "${PIDS[@]}"; do kill -9 -- "-$p" >/dev/null 2>&1; done
}
trap cleanup EXIT INT TERM

echo "[drill] 起 hardware bridge（120 cps）"
setsid env ALLOW_MOTION=true MAX_COUNTS_PER_S=120 \
  ./scripts/run_pi_hardware_bridge.sh >"$OUT/${STAMP}_drill_hw.log" 2>&1 &
PIDS+=("$!"); sleep 3
kill -0 "${PIDS[-1]}" || { echo "DRILL_FAIL bridge 起不來"; exit 1; }

echo "[drill] 起 pseudo-pose + safety_gate"
POSE_LOG="$OUT/${STAMP}_drill_pose.log"
setsid ros2 run vgr_safety_gate bench_pseudo_pose >"$POSE_LOG" 2>&1 &
PIDS+=("$!")
setsid ros2 run vgr_safety_gate safety_gate_node --ros-args \
  -p filter_name:=safe_apf -p max_v_mps:=0.03 -p max_omega_rad_s:=0.25 \
  -p nav_timeout_s:=0.2 -p 'geofence:=[-2.5,-2.5, 2.5,-2.5, 2.5,2.5, -2.5,2.5]' \
  >"$OUT/${STAMP}_drill_gate.log" 2>&1 &
PIDS+=("$!"); sleep 3
timeout 5 ros2 service type /aruco/set_dropout | grep -q SetBool \
  || { echo "DRILL_FAIL dropout service 不在"; exit 1; }

echo "[drill] 錄 /hardware/status（--full-length，String JSON 會被預設截斷）"
setsid bash -c 'ros2 topic echo --full-length /hardware/status | stdbuf -oL grep -E "target_cps" | while IFS= read -r line; do printf "%s %s\n" "$(date +%s.%N)" "$line"; done' \
  >"$OUT/${STAMP}_drill_status.log" 2>&1 &
PIDS+=("$!")

echo "[drill] 前進命令 0.02 m/s @20Hz，共 12s"
setsid timeout 12 ros2 topic pub -r 20 /cmd_vel_nav geometry_msgs/msg/Twist \
  '{linear: {x: 0.02}, angular: {z: 0.0}}' >/dev/null 2>&1 &
PIDS+=("$!")

sleep 4
echo "[drill] 注入 dropout"
timeout 8 ros2 service call /aruco/set_dropout std_srvs/srv/SetBool '{data: true}' >/dev/null \
  || { echo "DRILL_FAIL dropout 呼叫失敗"; exit 1; }
sleep 4

cleanup
trap - EXIT

python3 - "$OUT/${STAMP}_drill_status.log" "$POSE_LOG" <<'PY'
import re
import sys

status_path, pose_log_path = sys.argv[1], sys.argv[2]
m = re.search(r"\[WARN\] \[([\d.]+)\].*dropout=True", open(pose_log_path).read())
if not m:
    print("DRILL_FAIL pseudo-pose log 沒有 dropout=True 時戳"); sys.exit(1)
t0 = float(m.group(1))

samples = []
for ln in open(status_path).read().splitlines():
    ts = re.match(r"([\d.]+) ", ln)
    if not ts:
        continue
    pairs = dict(re.findall(r'"?(left|right)_target_cps"?\s*[:=]\s*(-?\d+)', ln))
    if len(pairs) == 2:
        samples.append((float(ts.group(1)), int(pairs["left"]), int(pairs["right"])))

before = [s for s in samples if s[0] < t0]
after = [s for s in samples if s[0] >= t0]
moving = [s for s in before if s[1] or s[2]]
if not moving:
    print("DRILL_FAIL 斷鏈前輪子沒在動，演練無效"); sys.exit(1)
stop = next((s for s in after if s[1] == 0 and s[2] == 0), None)
if stop is None:
    print("DRILL_FAIL 斷鏈後 target 沒歸零！"); sys.exit(1)
latency = stop[0] - t0
tail = [s for s in after if s[0] >= stop[0]]
nonzero_after = [s for s in tail if s[1] or s[2]]
hold = tail[-1][0] - stop[0] if tail else 0.0
print(f"斷鏈前運轉样本 {len(moving)}（最後 target {moving[-1][1]}/{moving[-1][2]} cps）")
print(f"急停延遲 {latency:.3f}s（門檻 0.60 = 0.4 看門狗＋兩拍 20Hz 取樣＋餘裕）")
print(f"停止後觀察 {hold:.1f}s，再動次數 {len(nonzero_after)}")
ok = latency <= 0.60 and not nonzero_after and hold >= 2.0
print("DRILL_PASS" if ok else "DRILL_FAIL 未滿足（≤0.60s 停、保持零 ≥2s）")
sys.exit(0 if ok else 1)
PY
rc=$?
if fuser /dev/ttyACM0 >/dev/null 2>&1; then
  echo "警告：serial 仍被占用"; rc=1
fi
echo "status log: $OUT/${STAMP}_drill_status.log"
exit $rc
