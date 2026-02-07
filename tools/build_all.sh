#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY_SYS="/usr/bin/python3"
export PYTHONNOUSERSITE=1

# Source ROS safely: ROS setup scripts may read unset variables.
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Make sure rosdep is initialized (safe to rerun)
if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update

# System deps for ROS build + runtime (safe to rerun)
sudo apt-get update
sudo apt-get install -y \
  python3-empy \
  python3-catkin-pkg \
  python3-numpy \
  python3-flask \
  python3-colcon-common-extensions \
  python3-pip

# Resolve rosdep dependencies for packages under ros2/
rosdep install --from-paths "${REPO_ROOT}/ros2" --ignore-src -r -y

# Build React
cd "${REPO_ROOT}/web"
npm ci
npm run build

# Deploy React build into ROS package static/app
cd "${REPO_ROOT}"
APP_STATIC_DIR="${REPO_ROOT}/ros2/openamr_ui_package/openamr_ui_package/static/app"
mkdir -p "${APP_STATIC_DIR}"
rm -rf "${APP_STATIC_DIR:?}/"*
cp -R "${REPO_ROOT}/web/build/"* "${APP_STATIC_DIR}/"

# Build ROS workspace (symlink install for dev)
cd "${REPO_ROOT}"
rm -rf build install log
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE="${PY_SYS}"

