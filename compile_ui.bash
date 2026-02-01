#!/usr/bin/env bash
set -euo pipefail

# Legacy entrypoint kept for compatibility.
# Canonical build flow lives in ./scripts/
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh

echo "DONE: UI built and synced into ROS2 package static/"
