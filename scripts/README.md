# Scripts

This folder contains the canonical scripts used to build and run the project.

## Frontend (React)

Build production bundle:

```bash
bash scripts/build_frontend.sh
```

Sync build into the ROS 2 package static app folder:

```bash
bash scripts/sync_frontend_to_ros.sh
```

Recommended combined flow:

```bash
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
```

## ROS 2 Workspace

Build ROS2 workspace (requires ROS2 + colcon):

```bash
bash scripts/build_ros.sh
```

Run UI backend (requires ROS2):

```bash
bash scripts/run_ui_backend.sh
```

## Docker Container

`container_entrypoint.sh` is used by the root `Dockerfile`. It sources ROS 2,
optionally rebuilds the frontend and ROS workspace when
`OPENAMR_REBUILD_ON_START=1` is set, sources the built workspace, and launches:

```bash
ros2 launch openamr_ui_bringup ui.launch.py
```

Normal users should start the container from the repository root:

```bash
docker compose up --build
```
