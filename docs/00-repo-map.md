# Repository map

This document explains the purpose of each top-level folder and the main entry scripts.

## Top-level folders

### ./.github/
GitHub configuration (workflows, repo settings, automation).  
This folder is not required to run the UI locally, but it is required to keep the repo healthy (CI, checks).

### ./docs/
Project documentation.
- docs/assets/ contains images used in documentation (screenshots, diagrams).
- The goal is that all “how to use” and “how to contribute” information lives under docs/.

### ./ros2/
ROS2 packages related to the UI.
Typical content includes:
- a package that serves the UI (e.g., Flask server + static UI build)
- messages and interfaces for UI-related communication

Important:
- ROS2 build artifacts must not be committed (build/, install/, log/ are ignored).

### ./scripts/
Helper scripts (bootstrap, build, run).
These scripts will be standardized and corrected in PR-02 so that they match the actual repo layout and provide a predictable workflow.

### ./web/
Frontend React application.
This folder is the user interface itself (browser UI).
Notes:
- web/node_modules must never be committed.
- The production build output is typically created with `npm run build`.

### ./images/
This folder is legacy and may be removed if it only contains documentation images.
Documentation images should live under docs/assets.

## Root scripts

### ./compile_ui.bash
Current purpose (legacy): build the frontend and place artifacts into the ROS2 package static directory.  
Status: expected to be updated in PR-02 to match the actual folder layout.

### ./install_openamr_deps.bash
Current purpose (legacy): install dependencies required for running the project.  
Status: expected to be reviewed and likely replaced by a clearer workflow in PR-02/PR-03 (Codespaces-first).

### ./launch.bash
Current purpose (legacy): start the UI stack.  
Status: expected to be reviewed and likely replaced by standardized scripts in PR-02.

## What changes next (PR-02)

PR-02 will standardize all scripts under ./scripts/ and ensure:
- build and run commands match the real folder layout
- each script has clear usage instructions
- root scripts become wrappers or are deprecated
