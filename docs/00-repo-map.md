# Repository map

This document explains the purpose of each top-level folder and the main entry scripts.

## Top-level folders

### ./.github/
GitHub configuration (workflows, automation). Not required to run locally, but required for CI and repo health.

### ./docs/
Project documentation.
- docs/assets/ contains images used in documentation.

### ./ros2/
ROS2 packages related to the UI (serving UI assets + UI messages/interfaces).
ROS2 build artifacts must not be committed (build/, install/, log/).

### ./scripts/
Helper scripts (bootstrap/build/run). These will be standardized in PR-02.

### ./web/
Frontend React application (the UI).
Important: web/node_modules must never be committed.

## Root scripts (legacy)

### ./compile_ui.bash
Legacy build script. Expected to be corrected in PR-02 to match the actual folder layout (web/ + ros2/...).

### ./install_openamr_deps.bash
Legacy dependency installer. Expected to be reviewed in PR-02/PR-03 (Codespaces-first approach).

### ./launch.bash
Legacy launcher. Expected to be reviewed and likely replaced by standardized scripts in PR-02.
