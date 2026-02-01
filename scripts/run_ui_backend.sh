#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 CLI not found. ROS2 is required to run the UI backend."
  echo "If you are in Codespaces without ROS2, run only the frontend (web/) for now."
  exit 1
fi

ros2 launch openamr_ui_package ui_launch.py
