#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2"

if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon not found. ROS2 tooling is required to build ros2/ workspace."
  echo "If you are in Codespaces without ROS2, skip this step for now."
  exit 1
fi

cd "${WS_DIR}"
colcon build --symlink-install

echo "[build_ros] OK: colcon build completed"
