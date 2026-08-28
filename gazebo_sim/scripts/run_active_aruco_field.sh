#!/bin/bash
# Active ArUco field safety experiment runner (lean, single-repeat).
#
# Three arms, each exactly one repeat:
#   controlled_adaptive   injected vision dropout window + drift-aware SAPF
#   controlled_fixed_028  same injected dropout + fixed 0.28 m safe radius
#   natural_adaptive      no injection; rendered-marker visibility only
#
# DRY_RUN=YES writes locked manifests and launches nothing. Runtime reuses the
# proven R3/G4 Gazebo patterns: isolated robot model with left-wheel friction
# mu=0.03 and camera, robot inserted into the active world, per-run HOME /
# ROS domain / gz partition, setsid process groups with a cleanup trap,
# headless rendering, ros_gz image+state bridge, recorder, vision gate, safety
# gate, dropout scheduler (controlled arms only), detector on /aruco/pose_raw,
# bounded readiness wait, and nominal launched last.
#
# Usage:
#   DRY_RUN=YES bash gazebo_sim/scripts/run_active_aruco_field.sh --all --out /tmp/f
#   bash gazebo_sim/scripts/run_active_aruco_field.sh --arm controlled_adaptive --out /tmp/f
#   bash gazebo_sim/scripts/run_active_aruco_field.sh --arm natural_adaptive --smoke --out /tmp/f
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_ROOT="$REPO_ROOT/gazebo_sim/models"
WORLD_SRC="$REPO_ROOT/gazebo_sim/worlds/vgr_field_active.world"
FIELD_MARKER_MAP="$REPO_ROOT/config/field_marker_map.json"
CAMERA_INFO_JSON="$REPO_ROOT/gazebo_sim/models/vgr_diff_drive/camera_info.json"

# --- locked synthetic portfolio parameters ---
START_X=1.8
START_Y=0.5
START_YAW=3.1415926536
GOAL_X=0.4
GOAL_Y=0.5
ROBOT_RADIUS_M=0.23
WALL_THICKNESS_M=0.05
LEFT_WHEEL_MU=0.03
BLIND_MAX_DIST_M=100.0
BLIND_MAX_S=300.0
TIMEOUT_SIM_S=90.0
DROPOUT_X=1.40
RESUME_X=0.80
WALLS_X=(0.0 2.5 2.4 0.2)
WALLS_Y=(-0.7 -0.6 1.9 1.8)
ARM_ORDER=(controlled_adaptive controlled_fixed_028 natural_adaptive)

ARM=""
REPEAT="1"
OUT_DIR=""
ALL=0
SMOKE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm) ARM="$2"; shift 2 ;;
    --repeat) REPEAT="$2"; shift 2 ;;
    --out) OUT_DIR="$2"; shift 2 ;;
    --all) ALL=1; shift ;;
    --smoke) SMOKE=1; shift ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

valid_arm() {
  local a
  for a in "${ARM_ORDER[@]}"; do
    [[ "$a" == "$1" ]] && return 0
  done
  return 1
}

arm_index() {
  local i
  for i in "${!ARM_ORDER[@]}"; do
    if [[ "${ARM_ORDER[$i]}" == "$1" ]]; then echo "$i"; return 0; fi
  done
  return 1
}

if [[ "$REPEAT" != "1" ]]; then echo "repeat must be 1" >&2; exit 2; fi
[[ -n "$OUT_DIR" ]] || { echo "need --out DIR" >&2; exit 2; }
if [[ "$ALL" == "1" ]]; then
  if [[ -n "$ARM" ]]; then echo "--all and --arm are mutually exclusive" >&2; exit 2; fi
else
  [[ -n "$ARM" ]] || { echo "need --arm" >&2; exit 2; }
  if ! valid_arm "$ARM"; then echo "unknown arm: $ARM" >&2; exit 2; fi
fi
mkdir -p "$OUT_DIR"

write_manifest() {
  local arm="$1" run_dir="$2"
  python3 - "$arm" "$run_dir" <<'PY'
import json
import pathlib
import sys

arm, run_dir = sys.argv[1], sys.argv[2]
if arm == "controlled_fixed_028":
    filter_cfg = {
        "name": "safe_apf_new",
        "ignore_pose_drift": True,
        "fixed_d_safe_m": 0.28,
    }
else:  # controlled_adaptive and natural_adaptive share the drift-aware field
    filter_cfg = {
        "name": "safe_apf_new",
        "ignore_pose_drift": False,
        "fixed_d_safe_m": None,
    }
manifest = {
    "arm": arm,
    "repeat": 1,
    "start_pose": {"x": 1.8, "y": 0.5, "yaw": 3.1415926536},
    "goal": {"x": 0.4, "y": 0.5},
    "walls": [[0.0, -0.7], [2.5, -0.6], [2.4, 1.9], [0.2, 1.8]],
    "wall_thickness_m": 0.05,
    "robot_radius_m": 0.23,
    "left_wheel_mu": 0.03,
    "timeout_sim_s": 90.0,
    "runtime_failures": [],
    "dropout": {
        "enabled": arm != "natural_adaptive",
        "dropout_x": 1.40,
        "resume_x": 0.80,
    },
    "filter": filter_cfg,
    "blind_max_dist_m": 100.0,
    "blind_max_s": 300.0,
}
pathlib.Path(run_dir).mkdir(parents=True, exist_ok=True)
pathlib.Path(run_dir, "manifest.json").write_text(
    json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
PY
}

if [[ "${DRY_RUN:-NO}" == "YES" ]]; then
  if [[ "$ALL" == "1" ]]; then
    for i in "${!ARM_ORDER[@]}"; do
      idx="$(printf "%02d" "$i")"
      write_manifest "${ARM_ORDER[$i]}" "$OUT_DIR/${idx}_${ARM_ORDER[$i]}"
    done
  else
    idx="$(printf "%02d" "$(arm_index "$ARM")")"
    write_manifest "$ARM" "$OUT_DIR/${idx}_${ARM}"
  fi
  echo "DRY_RUN_OK"
  exit 0
fi

# ROS setup reads unset variables; toggle strict off around the source.
set +euo pipefail
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -euo pipefail

command -v ign > /dev/null 2>&1 || { echo "missing ign CLI" >&2; exit 1; }
command -v ros2 > /dev/null 2>&1 || { echo "missing ros2 CLI" >&2; exit 1; }
command -v setsid > /dev/null 2>&1 || { echo "missing setsid" >&2; exit 1; }

export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"

run_one() {
  local arm="$1" idx="$2"
  local run_dir="$OUT_DIR/${idx}_${arm}"
  local run_log="$run_dir/run.log"
  local trace="$run_dir/trace.jsonl"
  local eval_out="$run_dir/eval.json"
  local PIDS=()

  mkdir -p "$run_dir/home" "$run_dir/models/vgr_diff_drive"
  write_manifest "$arm" "$run_dir"

  # isolated robot model with left-wheel friction mu=0.03 (regenerates camera too)
  cp "$MODEL_ROOT/vgr_diff_drive/model.config" "$run_dir/models/vgr_diff_drive/"
  PYTHONPATH="$REPO_ROOT:$REPO_ROOT/ros2_ws/src/vgr_core" python3 - \
    "$LEFT_WHEEL_MU" "$run_dir/models/vgr_diff_drive/model.sdf" <<'PY'
import sys
from gazebo_sim.generators.generate_robot_sdf import build_robot_sdf
from vgr_core.motion import DiffDriveParams
open(sys.argv[2], "w", encoding="utf-8").write(
    build_robot_sdf(DiffDriveParams(), left_wheel_mu=float(sys.argv[1])))
PY

  # insert robot into the active world at the locked start pose
  python3 - "$WORLD_SRC" "$run_dir/world.sdf" <<PY
import sys
import xml.etree.ElementTree as ET
src, dst = sys.argv[1], sys.argv[2]
tree = ET.parse(src); root = tree.getroot(); world = root.find("world")
inc = ET.SubElement(world, "include")
ET.SubElement(inc, "uri").text = "model://vgr_diff_drive"
ET.SubElement(inc, "pose").text = "$START_X $START_Y 0 0 0 $START_YAW"
ET.indent(root, space="  ")
tree.write(dst, encoding="unicode", xml_declaration=False)
PY

  # per-run isolation
  export HOME="$run_dir/home"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + RANDOM % 100))}"
  export IGN_PARTITION="activefield_${idx}_${arm}_$$"
  export IGN_GAZEBO_RESOURCE_PATH="$run_dir/models:$MODEL_ROOT${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
  cd "$REPO_ROOT"

  cleanup() {
    # Graceful TERM first (recorder flushes JSONL continuously on each event).
    for pid in "${PIDS[@]:-}"; do
      kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
    done
    # Bounded grace window (~5s), then hard KILL any stragglers. Never block
    # indefinitely on a TERM-ignoring process.
    local deadline=$((SECONDS + 5))
    local alive=1
    while (( SECONDS < deadline )); do
      alive=0
      for pid in "${PIDS[@]:-}"; do
        if kill -0 -- "-$pid" 2>/dev/null; then alive=1; break; fi
      done
      if (( alive == 0 )); then break; fi
      sleep 0.2
    done
    if (( alive == 1 )); then
      for pid in "${PIDS[@]:-}"; do
        kill -9 -- "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null || true
      done
    fi
    for pid in "${PIDS[@]:-}"; do
      wait "$pid" 2>/dev/null || true
    done
  }
  trap cleanup EXIT INT TERM

  {
    echo "[phase] arm=$arm start"
    echo "domain=$ROS_DOMAIN_ID partition=$IGN_PARTITION"
  } >> "$run_log"

  # headless rendering server with the active world
  setsid ign gazebo -s -r --headless-rendering "$run_dir/world.sdf" \
    > "$run_dir/gazebo.log" 2>&1 &
  PIDS+=("$!")
  sleep 6

  # state/image bridge: sim clock, ground truth, odom, camera image.
  # Command bridging is deliberately deferred until after READY (and omitted
  # entirely in --smoke) so no motion can occur before readiness or in smoke.
  setsid ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
    '/sim/true_pose@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
    '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image' \
    > "$run_dir/bridge.log" 2>&1 &
  PIDS+=("$!")
  sleep 3

  # recorder starts before any motion
  setsid python3 -m gazebo_sim.nodes.trace_recorder \
    --ros-args -p use_sim_time:=true -p output_path:="$trace" \
    > "$run_dir/recorder.log" 2>&1 &
  PIDS+=("$!")

  # vision gate present in every arm; transparent when open
  setsid python3 -m vgr_safety_gate.vision_gate \
    --ros-args -p use_sim_time:=true \
    -p in_topic:=/aruco/pose_raw -p out_topic:=/aruco/pose \
    > "$run_dir/vision_gate.log" 2>&1 &
  PIDS+=("$!")

  # dropout scheduler: enabled for controlled arms, disabled for natural
  local DROPOUT_ENABLED="false"
  if [[ "$arm" == "controlled_adaptive" || "$arm" == "controlled_fixed_028" ]]; then
    DROPOUT_ENABLED="true"
  fi
  setsid python3 -m gazebo_sim.nodes.field_dropout_controller \
    --ros-args -p use_sim_time:=true -p enabled:="$DROPOUT_ENABLED" \
    -p dropout_x:="$DROPOUT_X" -p resume_x:="$RESUME_X" \
    > "$run_dir/dropout.log" 2>&1 &
  PIDS+=("$!")

  # safety gate: adaptive default vs fixed 0.28 for the fixed arm
  local GATE_EXTRA=()
  if [[ "$arm" == "controlled_fixed_028" ]]; then
    GATE_EXTRA+=(-p filter_kwargs_ignore_pose_drift:=true)
    GATE_EXTRA+=(-p filter_kwargs_fixed_d_safe_m:=0.28)
  fi
  setsid python3 -m gazebo_sim.nodes.safety_gate \
    --ros-args -p use_sim_time:=true -p filter_name:=safe_apf_new \
    -p fixed_goal_enabled:=true -p goal_x:="$GOAL_X" -p goal_y:="$GOAL_Y" \
    -p "blind_max_dist_m:=$BLIND_MAX_DIST_M" -p "blind_max_s:=$BLIND_MAX_S" \
    -p "geofence:=[0.0,-0.7, 2.5,-0.6, 2.4,1.9, 0.2,1.8]" \
    "${GATE_EXTRA[@]}" \
    > "$run_dir/gate.log" 2>&1 &
  PIDS+=("$!")

  # detector publishes accepted poses on /aruco/pose_raw from the field map
  setsid python3 -m gazebo_sim.nodes.aruco_detector \
    --ros-args -p use_sim_time:=true -p marker_map_path:="$FIELD_MARKER_MAP" \
    -p camera_info_path:="$CAMERA_INFO_JSON" -p pose_topic:=/aruco/pose_raw \
    > "$run_dir/detector.log" 2>&1 &
  PIDS+=("$!")
  sleep 3

  # nominal controller launched last
  setsid python3 -m vgr_safety_gate.sapf_nominal \
    --ros-args -p use_sim_time:=true -p goal_x:="$GOAL_X" -p goal_y:="$GOAL_Y" \
    -p pose_topic:=/aruco/pose -p pose_msg_type:=aruco \
    -p cmd_topic:=/cmd_vel_nav -p control_hz:=20.0 \
    > "$run_dir/nominal.log" 2>&1 &
  PIDS+=("$!")

  # bounded wait for clock, image, marker-5 fix, gate and nav heartbeats
  if ! python3 - "$run_dir/ready.log" <<'PY'
import json
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist as RosTwist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

log = sys.argv[1]


class Ready(Node):
    def __init__(self):
        super().__init__("active_ready")
        self.sim_t = None
        self.image = False
        self.raw = False
        self.marker5 = False
        self.status = False
        self.nav = False
        self.create_subscription(Odometry, "/sim/true_pose", self._sim, 10)
        self.create_subscription(Image, "/camera/image_raw", self._img,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/aruco/pose_raw", self._raw, 10)
        self.create_subscription(String, "/aruco/marker_ids", self._ids, 10)
        self.create_subscription(String, "/safety_gate/status", self._st, 10)
        self.create_subscription(RosTwist, "/cmd_vel_nav", self._nav, 10)

    def _sim(self, m):
        self.sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _img(self, m):
        self.image = True

    def _raw(self, m):
        self.raw = True

    def _ids(self, m):
        try:
            if 5 in json.loads(m.data).get("ids", []):
                self.marker5 = True
        except Exception:
            pass

    def _st(self, m):
        self.status = True

    def _nav(self, m):
        self.nav = True


rclpy.init()
n = Ready()
deadline = time.monotonic() + 45.0
t0 = time.monotonic()
ok = False
while time.monotonic() < deadline:
    rclpy.spin_once(n, timeout_sec=0.2)
    if n.image and n.raw and n.marker5 and n.status and n.nav and n.sim_t is not None:
        ok = True
        break
with open(log, "a", encoding="utf-8") as f:
    f.write("READY elapsed=%.1fs sim=%s image=%s raw=%s marker5=%s status=%s nav=%s\n"
            % (time.monotonic() - t0, n.sim_t, n.image, n.raw, n.marker5,
               n.status, n.nav))
n.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PY
  then
    echo "[ready] not ready arm=$arm" >&2
    { echo "[phase] NOT_READY arm=$arm"; } >> "$run_log"
    cleanup
    trap - EXIT
    return 1
  fi
  { echo "[phase] READY arm=$arm"; } >> "$run_log"

  # Normal mode only: bridge the safety command to the robot now that the stack
  # is ready. --smoke never bridges command, so the robot stays stationary.
  if [[ "$SMOKE" != "1" ]]; then
    setsid ros2 run ros_gz_bridge parameter_bridge \
      '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
      > "$run_dir/cmd_bridge.log" 2>&1 &
    PIDS+=("$!")
    sleep 2
  fi

  local RUN_S=90
  if [[ "$SMOKE" == "1" ]]; then
    RUN_S=10
  fi
  # smoke records a stationary stack; normal records 90 sim seconds with a
  # bounded wall cap (sim runs at ~real time, +30 s boot slack).
  { echo "[phase] running ${RUN_S}s arm=$arm"; } >> "$run_log"
  timeout "$((RUN_S + 30))" bash -c "sleep $RUN_S" || true

  # stop processes (recorder flushes JSONL on shutdown) before evaluation
  cleanup
  trap - EXIT
  { echo "[phase] stopped arm=$arm"; } >> "$run_log"

  if [[ ! -s "$trace" ]]; then
    echo "[trace] empty trace arm=$arm" >&2
    return 1
  fi

  if python3 -c "import gazebo_sim.evaluate_active_aruco_field" 2>/dev/null; then
    set +e
    python3 -m gazebo_sim.evaluate_active_aruco_field \
      --trace "$trace" --manifest "$run_dir/manifest.json" --output "$eval_out" \
      > "$run_dir/eval.log" 2>&1
    local EVAL_STATUS=$?
    set -e
    if [[ "$EVAL_STATUS" -ne 0 ]]; then
      echo "EVAL_FAIL arm=$arm" >&2
      return 1
    fi
    { echo "[phase] evaluated arm=$arm"; } >> "$run_log"
  else
    echo "[evaluator] CLI unavailable; trace=$trace" >&2
  fi
}

if [[ "$ALL" == "1" ]]; then
  BATCH_STATUS=0
  for i in "${!ARM_ORDER[@]}"; do
    idx="$(printf "%02d" "$i")"
    if ! run_one "${ARM_ORDER[$i]}" "$idx"; then
      echo "RUN_FAIL ${ARM_ORDER[$i]}"
      BATCH_STATUS=1
    fi
  done
  if [[ "$BATCH_STATUS" -ne 0 ]]; then
    echo "ACTIVE_FIELD_ALL_FAILED"
    exit "$BATCH_STATUS"
  fi
  echo "ACTIVE_FIELD_ALL_DONE"
  exit 0
fi

idx="$(printf "%02d" "$(arm_index "$ARM")")"
run_one "$ARM" "$idx" || { echo "RUN_FAIL arm=$ARM"; exit 1; }
echo "ACTIVE_FIELD_DONE arm=$ARM"
exit 0
