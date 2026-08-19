# Installation and Launch Guide

This guide contains the detailed installation and launch procedures. For the
shortest Demo Mode path, start with the [main README](../README.md).

> [!CAUTION]
> The dashboard's red **E-STOP** is a non-latched software stop. It does not
> replace a tested physical emergency stop on real hardware.

## Contents

- [Choose one method](#choose-one-method)
- [Compatibility matrix](#compatibility-matrix)
- [Docker Compose](#docker-compose)
- [Manual installation](#manual-installation)
- [Start the complete system](#start-the-complete-system)
- [Launch choices](#launch-choices)
- [Ports](#ports)
- [Voice Command](#voice-command)
- [Back up and restore](#back-up-and-restore)
- [Reset and uninstall](#reset-and-uninstall)
- [Next](#next)

## Choose one method

| Method | Best for |
| --- | --- |
| Docker Compose | First-time users, demos, clean machines, and WSL |
| Manual installation | Robot computers, frontend development, ROS debugging, and package development |

Both methods start only the UI workspace. The robot, Nav2, localization,
sensors, docking, and simulator must run separately.

## Compatibility matrix

| Component | Current target | Notes |
| --- | --- | --- |
| Ubuntu | 24.04 LTS | Documented manual host |
| ROS 2 | Jazzy | Package names and launch commands target Jazzy |
| Node.js | 18–20 | Version 18 is the minimum; version 20 is used for current documentation validation |
| npm | Version bundled with a supported Node.js release | Use `npm ci` with the committed lockfile |
| `openamr-platform-sw` | A matching ROS 2 Jazzy branch or commit | No release is pinned yet |
| Robot hardware | An OpenAMRobot platform implementing the documented topic/service contract | No hardware revision is pinned yet |

For repeatable deployments, record:

- UI repository commit.
- `openamr-platform-sw` commit.
- Robot hardware revision and controller firmware.
- ROS domain and middleware.
- Map, parameter, and safety-limit versions.

Do not assume two installations are compatible merely because both use ROS 2
Jazzy. Verify the topic names and message types described in
[Lesson 10](lessons/10-topics-as-the-contract.md).

## Docker Compose

### Requirements

- Docker Engine
- Docker Compose plugin
- Linux or WSL recommended for ROS 2 host networking

Verify Docker:

```bash
docker --version
docker compose version
```

Clone and start:

```bash
cd ~
git clone https://github.com/openAMRobot/openamrobot-ui.git
cd openamrobot-ui
docker compose up --build
```

Open `http://127.0.0.1:5050/`.

Useful commands:

```bash
docker compose up -d
docker compose logs -f
docker compose down
OPENAMR_REBUILD_ON_START=1 docker compose up
```

`docker compose down` retains the named backend-data volume. Running
`docker compose down --volumes` also deletes that volume and its saved
programs, recordings, locations, history, and certificates.

### Runtime configuration

The Compose service accepts:

| Variable | Default | Purpose |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | `0` | ROS 2 discovery domain |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | ROS middleware implementation |
| `OPENAMR_REBUILD_ON_START` | `0` | Rebuild frontend and ROS workspace at container start |
| `AUTH_MODE` | `open` | Requested access mode; only unauthenticated `open` is implemented |
| `ANTHROPIC_API_KEY` | empty | Optional Voice Command backend key |

Pass secrets at runtime:

```bash
ANTHROPIC_API_KEY="your-key" docker compose up
```

Real `.env` files are excluded from the Docker build context. Never bake API
keys into an image or commit them.

### Networking

The default Compose configuration uses `network_mode: host`. This works best
on Linux and WSL for ROS 2 discovery and robot connectivity.

If Docker Desktop host networking is unavailable, replace host networking
with:

```yaml
ports:
  - "5050:5050"
  - "9090:9090"
  - "8080:8080"
```

Explicit port mappings can expose the browser services, but ROS 2 discovery
between the container and an external robot may still require platform-specific
network configuration.

### Docker success check

The logs should show:

- `flask_app`
- `rosbridge_websocket`
- `map_volatile_relay`
- `nav_relays`

The optional `camera` node appears only when `web_video_server` is available.
The dashboard should open on port `5050`; Demo Mode should show a purple banner
and green simulated connection indicator.

## Manual installation

### Recommended environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- `colcon`
- Node.js 18 or newer
- npm

Verify ROS and Node.js:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix rclpy
node --version
npm --version
```

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  nodejs \
  npm \
  ros-jazzy-rosbridge-server \
  ros-jazzy-rosapi \
  ros-jazzy-web-video-server \
  ros-jazzy-nav2-msgs \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-xacro \
  python3-flask \
  python3-yaml \
  python3-serial
```

Clone the repository:

```bash
cd ~
git clone https://github.com/openAMRobot/openamrobot-ui.git
cd openamrobot-ui
```

Install dependencies declared by the ROS packages:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the frontend, synchronize it into the ROS package, and build ROS:

```bash
cd ~/openamrobot-ui
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh

cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Verify the packages:

```bash
ros2 pkg list | grep openamr_ui
```

Expected packages:

- `openamr_ui_bringup`
- `openamr_ui_msgs`
- `openamr_ui_package`

## Start the complete system

Start the robot or simulation workspace first. For example:

```bash
cd ~/openamr-platform-sw
source install/setup.bash
ros2 launch openamrobot_docking bringup_sim.launch.py
```

In another terminal, start the UI:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py
```

Open:

```text
http://127.0.0.1:5050/
```

For another computer on the same network:

```text
http://<ui-computer-ip>:5050/
```

## Launch choices

Recommended UI-only launch:

```bash
ros2 launch openamr_ui_bringup ui.launch.py
```

Direct package launch:

```bash
ros2 launch openamr_ui_package new_ui_launch.py
```

Optional map/route file-management and route-following helpers:

```bash
ros2 launch openamr_ui_package physnode_launch.py
```

The optional helper is required for UI actions that create, rename, delete,
save, or load map and route files.

## Ports

| Port | Protocol | Purpose |
| --- | --- | --- |
| `5050` | HTTP | Flask UI and REST API |
| `9090` | WebSocket | Browser-to-ROS communication |
| `8080` | HTTP | Optional camera/image streams |

Guest networks, client isolation, VPN routing, and local firewalls can prevent
devices on the same WiFi network from communicating.

## Voice Command

For a manual launch:

```bash
cd ~/openamrobot-ui/ros2/src/openamr_ui_package
cp .env.example .env
```

Edit `.env` and set `ANTHROPIC_API_KEY`. The file is gitignored.

For Docker, pass the key through the runtime environment instead. See
[SECURITY.md](../SECURITY.md) for secret-handling guidance.

## Back up and restore

Stop active robot motion, Programs, Missions, recordings, and the UI before
copying or restoring backend data.

### Manual backend data

Backend programs, locations, run history, recordings, and generated
certificates live in `~/.openamr_ui/`.

Back up:

```bash
tar -czf openamr-ui-backend-backup.tar.gz -C ~ .openamr_ui
```

Restore into the same user's home directory:

```bash
tar -xzf openamr-ui-backend-backup.tar.gz -C ~
```

Restoring can overwrite files with matching names. Keep the UI stopped and
inspect the archive before extraction.

### Maps and routes

From the repository root:

```bash
tar -czf openamr-ui-maps-routes-backup.tar.gz \
  ros2/src/openamr_ui_package/maps \
  ros2/src/openamr_ui_package/paths
```

Restore only into a compatible UI/platform checkout, then verify the active
map/route configuration before commanding motion.

### Docker volume

Find the actual Compose volume name:

```bash
docker volume ls --filter name=openamr_ui_data
```

Set the returned name and create an archive:

```bash
OPENAMR_DATA_VOLUME="openamrobot-ui_openamr_ui_data"
docker run --rm \
  -v "${OPENAMR_DATA_VOLUME}:/data:ro" \
  -v "${PWD}:/backup" \
  alpine tar -czf /backup/openamr-ui-docker-data.tar.gz -C /data .
```

To restore, create or identify the destination volume, stop the UI, and
extract:

```bash
OPENAMR_DATA_VOLUME="openamrobot-ui_openamr_ui_data"
docker run --rm \
  -v "${OPENAMR_DATA_VOLUME}:/data" \
  -v "${PWD}:/backup:ro" \
  alpine tar -xzf /backup/openamr-ui-docker-data.tar.gz -C /data
```

The example volume name depends on the Compose project name. Always use the
exact name returned by `docker volume ls`.

### Browser-local data

Schedules, Missions, devices, robot profiles, metrics, waypoints, preferences,
and Blockly drafts are stored in the current browser profile. They are
separate for `localhost`, `127.0.0.1`, and a robot IP address.

There is no single complete browser-data export yet. Before clearing site
data, use the available JSON/export controls for Programs and Events, download
a Health support package, and record any configuration that must be recreated.

## Reset and uninstall

### Reset only browser data

Use the browser's site-data settings for the exact UI address and clear its
stored data. This resets onboarding, Demo Mode, connection profiles, schedules,
Missions, devices, metrics, waypoints, preferences, Notes, and local Blockly
drafts. It does not delete backend recordings or programs.

### Clean generated build output

From the repository root, the following directories contain generated output
and can be regenerated:

```text
web/node_modules/
web/build/
ros2/build/
ros2/install/
ros2/log/
```

Back up user data first. Then remove only those exact generated directories if
a completely clean rebuild is required.

### Remove a manual installation

1. Stop the UI and robot/simulation launch processes.
2. Back up `~/.openamr_ui/`, maps, and routes if needed.
3. Move the repository directory to a backup location or delete that exact
   checkout.
4. Delete `~/.openamr_ui/` only if backend programs and recordings are no
   longer needed.
5. ROS, Node.js, and system packages are shared dependencies; remove them only
   if no other workspace uses them.

### Remove Docker deployment

Stop the container while retaining data:

```bash
docker compose down
```

Remove the container and named data volume only after a verified backup:

```bash
docker compose down --volumes
```

The second command permanently removes backend programs, recordings,
locations, history, and certificates stored in that Compose volume.

## Next

- [Lesson 00 — Your First 10 Minutes](lessons/00-your-first-10-minutes.md)
- [Troubleshooting guide](troubleshooting.md)
- [Development guide](development.md)
