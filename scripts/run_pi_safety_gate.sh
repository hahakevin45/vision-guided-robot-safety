#!/bin/bash
# Helper script to run safety gate on Pi.
# Users can override ROS_DISTRO_SETUP to point to another ROS installation if needed.

ROS_DISTRO_SETUP=${ROS_DISTRO_SETUP:-"/opt/ros/humble/setup.bash"}

if [ -f "$ROS_DISTRO_SETUP" ]; then
    source "$ROS_DISTRO_SETUP"
else
    echo "Warning: ROS distro setup not found at $ROS_DISTRO_SETUP"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$REPO_ROOT/ros2_ws/install/setup.bash"
else
    echo "Warning: workspace setup.bash not found at $REPO_ROOT/ros2_ws/install/setup.bash"
fi

export PYTHONPATH="$REPO_ROOT:$PYTHONPATH"

exec ros2 launch vgr_safety_gate safety_gate.launch.py "$@"
