#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEB_DIR="${REPO_ROOT}/web"
ROS_WWW_DIR="${REPO_ROOT}/ros2/openamr_ui_package/www"

cd "${WEB_DIR}"

# Install frontend dependencies
if [ -f package-lock.json ]; then
  npm ci
else
  npm install
fi

# Build the frontend
npm run build

# Detect build output folder
if [ -d "${WEB_DIR}/dist" ]; then
  BUILD_OUT="${WEB_DIR}/dist"
elif [ -d "${WEB_DIR}/build" ]; then
  BUILD_OUT="${WEB_DIR}/build"
else
  echo "ERROR: No dist/ or build/ folder found after npm run build"
  exit 1
fi

# Copy build output into ROS package
mkdir -p "${ROS_WWW_DIR}"
rm -rf "${ROS_WWW_DIR:?}/"*
cp -R "${BUILD_OUT}/." "${ROS_WWW_DIR}/"

echo "DONE: UI built and copied into ros2/openamr_ui_package/www"
