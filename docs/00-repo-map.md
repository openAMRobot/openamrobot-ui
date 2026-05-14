# Repository map

This document explains the structure of the OpenAMRobot UI repository.
Its purpose is to help **new contributors and beginners** quickly understand:
- what lives where
- which files are entrypoints
- how frontend and ROS2 backend are connected
- where to start reading the code

This is an orientation document, not an implementation guide.

--------------------------------------------------------------------
## Target Repository Structure

The `openamrobot-ui` repository is the user interface and operator interface repository for the OpenAMRobot ecosystem.

Target structure:

```text
openamrobot-ui/
├── web/
│   ├── public/
│   ├── src/
│   ├── package.json
│   └── README.md
│
├── ros2/
│   └── openamr_ui_package/
│
├── docs/
├── scripts/
├── tools/
├── README.md
├── LICENSE
├── CONTRIBUTING.md
├── SECURITY.md
├── NOTICE.md
├── AUTHORS.md
└── CHANGELOG.md
--------------------------------------------------------------------
## Recommended reading order

If you are new to the project, read documentation in the following order:

1. `docs/00-repo-map.md` (this file – overall structure)
2. `docs/README.md` (how to build and run the full system)
3. `web/README.md` (frontend architecture and development)
4. `ros2/openamr_ui_package/README.md` (ROS2 backend overview)

--------------------------------------------------------------------

## Top-level directory overview

```text
.
├── docs/          # Project documentation
├── scripts/       # Canonical helper scripts (build, sync, run)
├── web/           # React-based UI frontend
├── ros2/          # ROS2 backend packages used by the UI
├── .github/       # GitHub configuration (CI, automation)
├── compile_ui.bash
├── install_openamr_deps.bash
├── launch.bash
└── README.md

