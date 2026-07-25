# OpenAMRobot UI

OpenAMRobot UI is a browser dashboard for operating and monitoring an
autonomous mobile robot. It provides maps, manual driving, navigation goals,
camera views, routes, visual programs, missions, health diagnostics,
recordings, and developer tools in one interface.

Demo Mode lets you explore the complete interface without a robot or ROS
connection.

[▶ Watch the current feature tour with human narration](docs/assets/openamrobot_ui_feature_tour_with_audio.mp4)

> [!CAUTION]
> The red **E-STOP** in the dashboard is a software stop. It sends one
> zero-velocity command and asks Nav2 to cancel the active goal. It is not
> latched, safety-rated, or independent of the browser, network, rosbridge,
> and robot controller. Keep a tested physical emergency stop within reach
> whenever operating real hardware.

## Quick Start

### Try the UI in five minutes

Requirements: Docker Engine and the Docker Compose plugin.

```bash
git clone https://github.com/openAMRobot/openamrobot-ui.git
cd openamrobot-ui
docker compose up --build
```

Then:

1. Open `http://127.0.0.1:5050/`.
2. Choose **Explore without a robot** in the first-run guide.
3. If the guide does not appear, open `/config` and enable **Demo Mode**.
4. Confirm the purple Demo Mode banner and green connection indicator.
5. Follow [Lesson 00 — Your First 10 Minutes](docs/lessons/00-your-first-10-minutes.md).

Demo Mode uses browser-side sample data and does not command a real robot.

### Connect to a real robot or simulation

The robot stack and UI are separate workspaces:

```text
Robot/simulation workspace → Nav2, localization, sensors, drivers, simulator
UI workspace               → dashboard, rosbridge, camera server, relays
```

Start the robot or simulation first. Then launch the UI:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py
```

Before enabling motion:

1. Turn Demo Mode off.
2. Confirm the correct robot connection profile.
3. Check that the connection indicator is green.
4. Open **Health** and confirm the required topics are fresh.
5. Confirm the displayed map and robot pose match the physical robot.
6. Use conservative speed limits and clear the operating area.
7. Test the physical emergency stop.

## Requirements

| Use case | Required |
| --- | --- |
| Docker demo/deployment | Docker Engine and Docker Compose; Linux or WSL is recommended for host networking |
| Manual installation | Ubuntu 24.04, ROS 2 Jazzy, Python 3, `colcon`, Node.js 18+, and npm |
| Live operation | A separately running OpenAMR robot or simulation stack |
| Remote browser | Access to TCP ports `5050`, `9090`, and optionally `8080` |
| Voice Command | Anthropic API key supplied to the backend at runtime |

### Compatibility

| Component | Current target | Status |
| --- | --- | --- |
| Ubuntu | 24.04 LTS | Manual-install target |
| ROS 2 | Jazzy | Required by the documented packages and commands |
| Node.js | 18–20 | Minimum 18; documentation validation currently uses 20 |
| `openamr-platform-sw` | Matching ROS 2 Jazzy branch/commit | Release is not yet pinned; record the exact commit for deployments |
| Robot hardware | Topic-compatible OpenAMRobot platform | Hardware revision is not yet pinned; validate drivers, limits, docking, and E-stop behavior per robot |

Until platform and hardware releases are pinned, treat a known-working
platform commit and robot revision as part of each deployment's configuration.

For complete installation details, Docker Desktop networking, simulation
commands, and dependency explanations, see the
[installation and launch guide](docs/installation.md).

## Installation

Choose Docker or manual installation. Do not run both UI instances on the
same ports.

### Docker Compose

```bash
docker compose up --build
```

Useful commands:

```bash
docker compose up -d       # Run in the background
docker compose logs -f     # Follow logs
docker compose down        # Stop and keep saved backend data
```

Optional Voice Command key:

```bash
ANTHROPIC_API_KEY="your-key" docker compose up
```

Real `.env` files are excluded from Docker images. Pass secrets at runtime;
never bake them into an image.

**Success check:** the dashboard opens on port `5050`, and the logs show
`flask_app`, `rosbridge_websocket`, `map_volatile_relay`, and `nav_relays`.
The optional camera node appears only when `web_video_server` is installed.

### Manual install

Install ROS 2 Jazzy, Node.js 18+, npm, and the declared ROS dependencies.
Then:

```bash
cd ~/openamrobot-ui
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh

cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py
```

The backend requires Xacro for the Robot Description page. A complete manual
dependency command and `rosdep` workflow are in the
[manual installation guide](docs/installation.md#manual-installation).

**Success check:**

```bash
ros2 pkg list | grep openamr_ui
```

This should list `openamr_ui_bringup`, `openamr_ui_msgs`, and
`openamr_ui_package`.

## Main Pages

| Page | URL | Use it for |
| --- | --- | --- |
| Map | `/` | Map, goals, pose, joystick, docking, and waypoints |
| Routes | `/route` | Reusable waypoint sequences |
| Maps | `/maps` | Create, save, switch, rename, and organize maps |
| Programs | `/blocks` | Blockly visual programs and Voice Command |
| Scheduler | `/scheduler` | Time-triggered browser-side actions |
| Missions | `/missions` | Multi-step browser-side missions |
| Status | `/info` | Camera, telemetry, battery, and system health |
| Robot | `/robot` | URDF/Xacro model and live joint information |
| Devices | `/devices` | External-device registry and serial detection |
| Health | `/health` | Readiness, topic freshness, lifecycle, and diagnostics |
| Metrics | `/metrics` | Distance, uptime, and success statistics |
| Recordings | `/recordings` | Rosbag recording and replay |
| Events | `/events` | Filterable event history |
| Console | `/console` | `/rosout` and topic echo |
| Parameters | `/params` | Read or change Nav2 parameters |
| Fleet | `/fleet` | Robot profiles and active-robot selection |
| Config | `/config` | Connections, Demo Mode, limits, and preferences |
| Notes | `/notes` | Example plugin page |

Scheduler and Missions run in the browser tab. Keep the tab open for scheduled
actions, and do not assume an interrupted browser session resumes safely.

See [Lesson 06](docs/lessons/06-the-pages.md) for screenshots, dependencies,
and task-based page guidance.

## Architecture

![OpenAMRobot UI architecture diagram](docs/assets/openamr_ui_architecture.svg)

Default services:

| Service | Default | Purpose |
| --- | --- | --- |
| Flask UI | `http://127.0.0.1:5050` | Serves the React dashboard and REST API |
| Rosbridge | `ws://127.0.0.1:9090` | Browser-to-ROS communication |
| Web video | `http://127.0.0.1:8080` | Optional camera streams |

Map/route file operations and route-following helpers require:

```bash
ros2 launch openamr_ui_package physnode_launch.py
```

For the communication and topic model, read
[Lesson 03](docs/lessons/03-how-the-browser-talks-to-ros.md),
[Lesson 04](docs/lessons/04-data-flow-and-relays.md), and
[Lesson 10](docs/lessons/10-topics-as-the-contract.md).

## Security and Access

Only unauthenticated `AUTH_MODE=open` is implemented. Anyone who can reach the
configured UI and rosbridge ports can potentially view data and command the
robot.

- Keep the dashboard on a trusted local network.
- Do not expose ports `5050`, `9090`, or `8080` directly to the internet.
- Use firewall rules or an authenticated reverse proxy when isolation is not
  sufficient.
- Do not commit `.env` files or API keys.
- Read [SECURITY.md](SECURITY.md) before deploying outside a private lab
  network or reporting a vulnerability.

`local` and `external` are reserved future authentication modes. Requesting
either currently falls back to `open` and shows a warning.

## Data and Backups

| Data | Location |
| --- | --- |
| Backend programs, locations, history, recordings, and certificates | `~/.openamr_ui/` |
| Docker backend data | Named `openamr_ui_data` volume |
| Schedules, missions, devices, profiles, metrics, and preferences | Current browser profile's `localStorage` |
| Maps and routes | `ros2/src/openamr_ui_package/maps/` and `paths/` |

Browser-local data does not automatically move to another browser or computer.
Back up important data before reinstalling, clearing browser storage, or
removing Docker volumes.

## Documentation

| Start here | Purpose |
| --- | --- |
| [Lesson 00](docs/lessons/00-your-first-10-minutes.md) | Safe first session and Demo Mode |
| [Lessons 00–13](docs/lessons/README.md) | Operator and architecture learning paths |
| [Glossary](docs/lessons/glossary.md) | ROS/UI term lookup |
| [Extension guides](docs/extending/README.md) | Add a panel, device, or Blockly block |
| [Installation guide](docs/installation.md) | Docker, manual installation, networking, and launch options |
| [Development guide](docs/development.md) | Frontend and ROS workflows, tests, and generated files |
| [Troubleshooting guide](docs/troubleshooting.md) | Symptom-based checks and ROS commands |
| [Frontend guide](web/README.md) | React development |
| [Blockly guide](web/src/features/blocks/README.md) | Blocks, execution, Voice Command, and troubleshooting |
| [Contributing](CONTRIBUTING.md) | Development and pull-request expectations |
| [Security](SECURITY.md) | Private vulnerability reporting and deployment guidance |

## Development

Frontend:

```bash
cd web
npm ci
npm run dev
```

Production build:

```bash
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
source /opt/ros/jazzy/setup.bash
bash scripts/build_ros.sh
```

Run tests:

```bash
cd web
CI=true npm test -- --watchAll=false
npm run build

cd ../ros2
source /opt/ros/jazzy/setup.bash
colcon test --packages-select openamr_ui_package openamr_ui_msgs
colcon test-result --verbose
```

Read [CONTRIBUTING.md](CONTRIBUTING.md) before changing robot controls,
topics, persistence, networking, or safety behavior.

## Troubleshooting

| Symptom | First check |
| --- | --- |
| Page does not open | UI process and port `5050` |
| Page opens, connection is red | Rosbridge process, configured host, and port `9090` |
| Connection is green, map/pose is frozen | Health page topic freshness; do not drive |
| Map alone is blank | `/map`, `/ui/map`, and `map_volatile_relay` |
| Camera alone is blank | Selected image topic and optional port `8080` service |
| Route/map buttons fail | Optional `physnode_launch.py` helpers |
| UI changes do not appear | Rebuild, sync, rebuild ROS package, then hard-refresh |

Continue with [Lesson 12](docs/lessons/12-debugging-with-ros-cli.md) or the
[troubleshooting guide](docs/troubleshooting.md).

## Project

- Support the project through [FUNDING.md](FUNDING.md).
- OpenAMRobot UI is available under the [MIT License](LICENSE).
