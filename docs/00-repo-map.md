# Repository Map

This document explains the structure and architectural boundaries of the `openamrobot-ui` repository.

Its purpose is to help contributors, developers, researchers, and beginners understand:

- repository structure
- frontend and ROS 2 backend separation
- repository responsibilities
- shared interface ownership
- recommended development workflow
- where to start reading the codebase

This is an orientation and architecture document, not an implementation guide.

---

# Repository Purpose

The `openamrobot-ui` repository is responsible for:

- operator interfaces
- frontend applications
- dashboards
- visualization tools
- UI workflows
- UI-specific ROS 2 bridge/backend logic
- user interaction systems

This repository is part of the larger OpenAMRobot ecosystem.

---

# Target Repository Structure

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
├── .github/
│
├── README.md
├── LICENSE
├── CONTRIBUTING.md
├── SECURITY.md
├── NOTICE.md
├── AUTHORS.md
└── CHANGELOG.md
```

---

# Repository Boundaries

This repository SHOULD contain:

- frontend code
- React UI applications
- visualization components
- operator workflows
- UI-specific ROS 2 bridge logic
- UI build scripts
- UI documentation

This repository SHOULD NOT become the central place for:

- shared ROS 2 messages
- shared ROS 2 services
- shared ROS 2 actions
- transport protocol definitions
- general communication schemas
- robot firmware
- robot hardware files

These belong to dedicated ecosystem repositories.

---

# Shared ROS 2 Interfaces

Shared ROS 2 interface packages are NOT maintained inside this repository.

They are centralized in:

```text
openamrobot-interfaces
```

Example shared package:

```text
openamrobot-interfaces/ros2/openamr_ui_msgs
```

Shared interface packages may contain:

- ROS 2 messages (`msg`)
- ROS 2 services (`srv`)
- ROS 2 actions (`action`)

---

# Why Shared Interfaces Are Centralized

Shared interface definitions must remain reusable across:

- UI applications
- ROS 2 software
- simulation
- docking
- navigation
- communication bridges
- future fleet-management systems
- future humanoid systems

Centralizing interfaces provides:

- a single source of truth
- version consistency
- interoperability
- cleaner repository boundaries
- reduced duplication
- simulation and hardware compatibility

---

# Dependency Pattern

The UI repository should consume shared interface packages as workspace dependencies.

Example workspace:

```text
workspace/
├── src/
│   ├── openamrobot-ui/
│   └── openamrobot-interfaces/
```

The following package was migrated out of this repository:

```text
ros2/openamr_ui_msgs
```

and is now maintained in:

```text
openamrobot-interfaces
```

---

# Recommended Reading Order

If you are new to the project, read documentation in the following order:

1. `docs/00-repo-map.md`
   - overall repository structure and boundaries

2. `docs/README.md`
   - build and development overview

3. `web/README.md`
   - frontend architecture and development workflow

4. `ros2/openamr_ui_package/README.md`
   - ROS 2 UI backend overview

---

# Top-Level Directory Overview

```text
.
├── docs/          # Documentation and architecture notes
├── scripts/       # Helper scripts for development and execution
├── tools/         # Development and maintenance tooling
├── web/           # React-based frontend UI
├── ros2/          # ROS 2 UI backend and bridge packages
├── .github/       # GitHub workflows and repository configuration
│
├── compile_ui.bash
├── install_openamr_deps.bash
├── launch.bash
└── README.md
```

---

# Ecosystem Relationship

This repository interacts closely with:

| Repository | Purpose |
|---|---|
| `openamr-platform-sw` | Core ROS 2 robot software |
| `openamrobot-interfaces` | Shared ROS 2 messages/services/actions |
| `openamrobot-comm` | Communication protocols and middleware |
| `openamrobot-docs` | Ecosystem-wide documentation |
| `openamr-platform-fw` | Embedded firmware |
| `openamr-platform-hw` | Hardware, CAD, electrical, and manufacturing |

---

# Long-Term Direction

The long-term goal is to create:

- modular UI architecture
- reusable visualization systems
- scalable fleet-management interfaces
- simulation-compatible tooling
- humanoid-compatible operator interfaces
- web-based robotics control infrastructure

while maintaining clean separation between:

- UI
- interfaces
- communication
- firmware
- hardware
- robot software