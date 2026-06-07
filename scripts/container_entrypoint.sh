#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/workspace"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${REPO_ROOT}"

if [ "${OPENAMR_REBUILD_ON_START:-0}" = "1" ]; then
  bash scripts/build_frontend.sh
  bash scripts/sync_frontend_to_ros.sh
  bash scripts/build_ros.sh
fi

source "${REPO_ROOT}/ros2/install/setup.bash"
exec ros2 launch openamr_ui_bringup ui.launch.py
