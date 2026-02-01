# Scripts

This folder contains the canonical scripts used to build and run the project.

## Frontend (React)

1) Build production bundle:
   bash scripts/build_frontend.sh

2) Sync build into ROS2 package static/ folder:
   bash scripts/sync_frontend_to_ros.sh

Recommended combined flow:
   bash scripts/build_frontend.sh
   bash scripts/sync_frontend_to_ros.sh

## ROS2 workspace

Build ROS2 workspace (requires ROS2 + colcon):
   bash scripts/build_ros.sh

Run UI backend (requires ROS2):
   bash scripts/run_ui_backend.sh
