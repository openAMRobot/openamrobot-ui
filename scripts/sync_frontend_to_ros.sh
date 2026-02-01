#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEB_BUILD_DIR="${REPO_ROOT}/web/build"
WEB_STATIC_DIR="${WEB_BUILD_DIR}/static"

ROS_STATIC_DIR="${REPO_ROOT}/ros2/openamr_ui_package/openamr_ui_package/static"
ROS_JS_DIR="${ROS_STATIC_DIR}/js"
ROS_CSS_DIR="${ROS_STATIC_DIR}/css"
ROS_MEDIA_DIR="${ROS_STATIC_DIR}/media"

if [ ! -d "${WEB_STATIC_DIR}" ]; then
  echo "ERROR: web build static folder not found: ${WEB_STATIC_DIR}"
  echo "Run: bash scripts/build_frontend.sh"
  exit 1
fi

mkdir -p "${ROS_JS_DIR}" "${ROS_CSS_DIR}" "${ROS_MEDIA_DIR}"

echo "[sync] Cleaning existing static subfolders (js/css/media only)..."
rm -rf "${ROS_JS_DIR:?}/"* "${ROS_CSS_DIR:?}/"* "${ROS_MEDIA_DIR:?}/"*

echo "[sync] Copying JS/CSS/media from web build into ROS2 package static/ ..."

# --- JS ---
# Copy all JS-related outputs first
cp -a "${WEB_STATIC_DIR}/js/." "${ROS_JS_DIR}/"

# Normalize the "main" bundle name to main.js for template stability
MAIN_JS="$(ls "${ROS_JS_DIR}/"main*.js 2>/dev/null | grep -v '\.map$' | grep -v 'LICENSE' | head -n 1 || true)"
if [ -z "${MAIN_JS}" ]; then
  echo "ERROR: Could not find main*.js in ${ROS_JS_DIR}"
  exit 1
fi
mv -f "${MAIN_JS}" "${ROS_JS_DIR}/main.js"

# --- CSS ---
cp -a "${WEB_STATIC_DIR}/css/." "${ROS_CSS_DIR}/"

MAIN_CSS="$(ls "${ROS_CSS_DIR}/"main*.css 2>/dev/null | grep -v '\.map$' | head -n 1 || true)"
if [ -z "${MAIN_CSS}" ]; then
  echo "ERROR: Could not find main*.css in ${ROS_CSS_DIR}"
  exit 1
fi
mv -f "${MAIN_CSS}" "${ROS_CSS_DIR}/main.css"

# --- MEDIA ---
if [ -d "${WEB_STATIC_DIR}/media" ]; then
  cp -a "${WEB_STATIC_DIR}/media/." "${ROS_MEDIA_DIR}/"
fi

echo "[sync] OK: Synced frontend build to ROS2 package static/"
echo "      JS : ${ROS_JS_DIR}/main.js (+ chunks)"
echo "      CSS: ${ROS_CSS_DIR}/main.css"
echo "      Media: ${ROS_MEDIA_DIR}/"
