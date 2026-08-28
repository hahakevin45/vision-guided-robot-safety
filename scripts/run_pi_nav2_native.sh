#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${VGR_NAV2_WS:-$HOME/nav2_jazzy_ws}"
OVERLAY="$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS2CLI_NO_DAEMON=1

[[ -f "$OVERLAY" ]] || {
  echo "native Nav2 overlay missing: $OVERLAY" >&2
  exit 3
}

set +u
source /opt/ros/jazzy/setup.bash
source "$OVERLAY"
set -u

for package in nav2_map_server nav2_planner nav2_controller nav2_behaviors \
  nav2_bt_navigator nav2_lifecycle_manager vgr_nav2_bringup; do
  prefix="$(ros2 pkg prefix "$package")"
  [[ "$prefix" == "$WORKSPACE/install/"* ]] || {
    echo "native overlay package missing: $package=$prefix" >&2
    exit 4
  }
done

cd "$REPO_ROOT"
exec ros2 launch vgr_nav2_bringup real_bench.launch.py "$@"
