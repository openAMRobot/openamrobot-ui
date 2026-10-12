#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/workspace"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# ROS and colcon setup scripts read variables that may be unset (for example
# AMENT_TRACE_SETUP_FILES), so nounset is relaxed only while sourcing them.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# openamrobot_description from openamr-platform-sw (built into the image at
# the commit in /opt/openamr_platform_sw/PLATFORM_SW_COMMIT), sourced before
# the UI workspace so the Robot Description page can find it.
source "/opt/openamr_platform_sw/install/setup.bash"
set -u
cd "${REPO_ROOT}"

if [ "${OPENAMR_REBUILD_ON_START:-0}" = "1" ]; then
  bash scripts/build_frontend.sh
  bash scripts/sync_frontend_to_ros.sh
  bash scripts/build_ros.sh
fi

set +u
source "${REPO_ROOT}/ros2/install/setup.bash"
set -u
exec ros2 launch openamr_ui_bringup ui.launch.py
