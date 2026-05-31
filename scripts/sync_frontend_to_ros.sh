#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEB_BUILD_DIR="${REPO_ROOT}/web/build"

ROS_APP_DIR="${REPO_ROOT}/ros2/src/openamr_ui_package/openamr_ui_package/static/app"

if [ ! -f "${WEB_BUILD_DIR}/index.html" ]; then
  echo "ERROR: web build not found: ${WEB_BUILD_DIR}"
  echo "Run: bash scripts/build_frontend.sh"
  exit 1
fi

mkdir -p "${ROS_APP_DIR}"

echo "[sync] Cleaning existing ROS package React build..."
rm -rf "${ROS_APP_DIR:?}/"*

echo "[sync] Copying full React build into ROS2 package static/app/ ..."
cp -a "${WEB_BUILD_DIR}/." "${ROS_APP_DIR}/"

echo "[sync] OK: Synced frontend build to ${ROS_APP_DIR}"
