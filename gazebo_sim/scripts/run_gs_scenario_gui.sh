#!/bin/bash
# GUI 版情境展示：與 run_gs_scenario.sh 同一套 stack，但開 Gazebo 視窗，
# 且結束時只收掉節點與 bridge，視窗留著讓人檢視最終狀態。
# 用法：./run_gs_scenario_gui.sh <GS1|GS2> <filter_name> [pseudo|vision]
source /opt/ros/humble/setup.bash
set -uo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
  echo "用法: run_gs_scenario_gui.sh <GS1|GS2> <filter_name> [pseudo|vision]" >&2
  exit 2
fi

GS="$1"
FILTER="$2"
POSE_SOURCE="${3:-pseudo}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
export IGN_GAZEBO_RESOURCE_PATH="$REPO/gazebo_sim/models"
export PYTHONPATH="$REPO:${PYTHONPATH:-}"
WORLD_SRC="$REPO/gazebo_sim/worlds/vgr_arena.world"

case "$GS" in
  GS1) PROFILE="gs1_wall_rush" ;;
  GS2) PROFILE="gs2_blackout" ;;
  GS3) PROFILE="gs3_sapf_single_obstacle" ;;
  *) echo "unknown scenario: $GS"; exit 2 ;;
esac
case "$POSE_SOURCE" in
  pseudo|vision) ;;
  *) echo "unknown pose_source: $POSE_SOURCE (expected pseudo or vision)"; exit 2 ;;
esac
if [ "$GS" = "GS3" ] && [ "$POSE_SOURCE" = "vision" ]; then
  echo "GS3 第一輪只支援 pseudo 位姿" >&2
  exit 2
fi
if [ "$GS" = "GS3" ]; then
  WORLD_SRC="$REPO/gazebo_sim/worlds/vgr_sapf.world"
elif [ "$POSE_SOURCE" = "vision" ]; then
  WORLD_SRC="$REPO/gazebo_sim/worlds/vgr_arena_vision.world"
fi

# 產生機器人已就位的臨時 world（起點 0.5, 0）。
TMP_WORLD="$(mktemp --suffix=.world)"
python3 - "$WORLD_SRC" "$TMP_WORLD" <<'EOF'
import pathlib, sys
w = pathlib.Path(sys.argv[1]).read_text()
inc = '''    <include>
      <uri>model://vgr_diff_drive</uri>
      <pose>0.5 0 0 0 0 0</pose>
    </include>
  </world>'''
pathlib.Path(sys.argv[2]).write_text(w.replace("  </world>", inc))
EOF

NODE_PIDS=()
cleanup_nodes() { for p in "${NODE_PIDS[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup_nodes EXIT

echo "[gui] 啟動 Gazebo 視窗（GUI + server）..."
ign gazebo -r "$TMP_WORLD" > /tmp/gs_gui_gazebo.log 2>&1 &
GZ_PID=$!
sleep 12   # GUI 啟動比 headless 慢

BRIDGE_TOPICS=(
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
  '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
  '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist'
)
if [ "$POSE_SOURCE" = "vision" ]; then
  BRIDGE_TOPICS+=('/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image')
fi

ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_TOPICS[@]}" \
  > /tmp/gs_gui_bridge.log 2>&1 &
NODE_PIDS+=($!)
sleep 3

if [ "$POSE_SOURCE" = "vision" ]; then
  python3 -m gazebo_sim.nodes.aruco_detector --ros-args -p use_sim_time:=true > /tmp/gs_gui_aruco.log 2>&1 &
else
  python3 -m gazebo_sim.nodes.pseudo_aruco   --ros-args -p use_sim_time:=true > /tmp/gs_gui_aruco.log 2>&1 &
fi
NODE_PIDS+=($!)
GATE_EXTRA_ARGS=()
case "$GS" in
  GS1) GATE_EXTRA_ARGS+=(-p fixed_goal_enabled:=true -p goal_x:=4.0 -p goal_y:=0.0) ;;
  GS2) GATE_EXTRA_ARGS+=(-p fixed_goal_enabled:=true -p goal_x:=3.0 -p goal_y:=0.0) ;;
  GS3)
    GATE_EXTRA_ARGS+=(
      -p fixed_goal_enabled:=true
      -p goal_x:=3.2
      -p goal_y:=0.0
      -p 'obstacles_json:="[{\"x\":2.0,\"y\":0.0,\"radius\":0.2}]"'
    )
    ;;
esac
python3 -m gazebo_sim.nodes.safety_gate    --ros-args -p use_sim_time:=true -p filter_name:="$FILTER" "${GATE_EXTRA_ARGS[@]}" > /tmp/gs_gui_gate.log 2>&1 &
NODE_PIDS+=($!)
sleep 3
python3 -m gazebo_sim.nodes.scripted_nav   --ros-args -p use_sim_time:=true -p profile:="$PROFILE" > /tmp/gs_gui_nav.log 2>&1 &
NODE_PIDS+=($!)

if [ "$GS" = "GS2" ]; then
  echo "[gui] 正常行駛 10 秒（看車往 +x 前進）..."
  sleep 10
  if [ "$POSE_SOURCE" = "vision" ]; then
    echo "[gui] GS2 vision 模式跳過 marker dropout service；marker 丟失由視野自然發生"
  else
    echo "[gui] >>> 注入 marker dropout（位姿凍結，安全層應在 ~0.5 秒後停車）<<<"
    timeout 10 ros2 service call /aruco/set_dropout std_srvs/srv/SetBool '{data: true}' >/dev/null 2>&1 || true
  fi
  sleep 10
elif [ "$GS" = "GS3" ]; then
  echo "[gui] GS3：車繞過中央圓柱前往 goal (3.2, 0)，45 秒..."
  sleep 45
else
  echo "[gui] GS1：Nav 全速直衝右側牆，觀察 filter 行為，35 秒..."
  sleep 35
fi

cleanup_nodes
trap - EXIT
echo "[gui] 情境結束。節點已收掉，Gazebo 視窗留著給你檢視；關閉視窗即結束。"
wait "$GZ_PID" 2>/dev/null || true
rm -f "$TMP_WORLD"
