# OpenAMRobot UI

Standalone ROS 2 web UI workspace for OpenAMR autonomous mobile robots.

This repository contains the React browser dashboard, ROS 2 UI packages,
custom UI messages, and helper scripts needed to build and run the UI. It is
intended to live separately from the main robot or simulation workspace. The
robot stack should start Nav2, localization, docking, Gazebo/RViz, sensors, and
the map server. This workspace starts only the web UI, rosbridge, camera web
streaming, and lightweight topic relays used by the browser.

## Beginner Overview

There are two different workspaces involved when using the UI with a robot or
simulation:

| Workspace | What It Does |
| --- | --- |
| Robot/simulation workspace, for example `~/openamr-platform-sw` | Starts the robot, Gazebo simulation, Nav2, localization, map server, docking, sensors, and robot topics |
| This UI workspace, `~/openamrobot-ui` | Starts the browser UI, rosbridge, camera web server, and small relay nodes |

Start the robot or simulation first. Then start the UI. The UI does not replace
Nav2, Gazebo, the map server, docking, or the robot bringup. It connects to
those running ROS topics and services.

Important words:

| Term | Meaning |
| --- | --- |
| ROS 2 | Robot middleware used to connect nodes, topics, services, and actions |
| Node | A running ROS process, such as Flask, rosbridge, Nav2, or a relay |
| Topic | A named stream of messages, such as `/cmd_vel`, `/odom`, or `/map` |
| Launch file | A Python file that starts multiple ROS nodes together |
| Gazebo | The simulator that shows and runs the robot in a virtual world |
| Headless | Simulation runs without the Gazebo graphical window |
| Gazebo GUI | The visible Gazebo window is opened |
| RViz | ROS visualization/debugging tool for maps, TF, costmaps, and Nav2 |
| rosbridge | WebSocket bridge that lets the browser talk to ROS |
| Flask | Python web server that serves the React UI |
| React | JavaScript frontend used by the browser dashboard |

For most beginners, the safest order is:

1. Build this UI workspace.
2. Start the robot or simulation workspace.
3. Start the UI launch from this workspace.
4. Open the browser at `http://127.0.0.1:5050/control`.
5. Confirm the UI says ROS is connected before driving or sending goals.

## System Architecture

The UI is a browser-based ROS 2 dashboard. The browser talks to ROS through
`rosbridge_server`, while the UI package provides small relay/helper nodes for
maps, AMCL pose, Nav2 status, waypoint routes, camera streaming, and map/route
file handling.

![OpenAMR ROS 2 topic and node schematic](docs/assets/openamr_ros_topic_node_schematic.png)

## What This UI Provides

- Browser dashboard served by Flask on port `5050`
- ROS communication through `rosbridge_websocket` on port `9090`
- Camera/image streaming through `web_video_server` on port `8080`
- Map display through a `/map` to `/ui/map` QoS relay
- AMCL pose and navigation/docking status relays under `/ui/*`
- Manual robot control through `/cmd_vel`
- Goal pose, initial pose, route, map, waypoint, docking, and status controls
- Map and route file management through the UI helper nodes

## Repository Layout

```text
openamrobot-ui/
  README.md
  scripts/
    build_frontend.sh          # Install web deps and build React
    sync_frontend_to_ros.sh    # Copy React build into ROS package static/app
    build_ros.sh               # Build ros2/ with colcon
    run_ui_backend.sh          # Run the recommended UI launch
  web/
    package.json               # React scripts and dependencies
    public/ros/                # roslibjs, ros2d, nav2d browser libraries
    src/                       # React app source
  ros2/
    src/openamr_ui_msgs/       # Custom UI messages
    src/openamr_ui_package/    # Main ROS 2 UI package
    src/openamr_ui_bringup/    # Small launch wrapper
```

Generated folders are intentionally ignored:

```text
web/node_modules/
web/build/
ros2/build/
ros2/install/
ros2/log/
```

## Documentation Layout

This top-level README is the source of truth for installing, building, running,
and troubleshooting the UI workspace. Folder-level README files are intentionally
short and only describe local package or directory details.

## Main Components

| Component | Purpose |
| --- | --- |
| `web/` | React frontend source for the browser UI |
| `openamr_ui_msgs` | Custom message package used by the UI |
| `openamr_ui_package` | Flask server, relays, map/route handlers, waypoint navigation helpers |
| `openamr_ui_bringup` | Recommended UI-only launch wrapper |
| `scripts/` | Canonical build and sync commands |

The compiled React app is copied into:

```text
ros2/src/openamr_ui_package/openamr_ui_package/static/app/
```

During `colcon build`, that static app is installed into the package share
directory and served by `openamr_ui_package.flask_app`.

## Prerequisites

Recommended environment:

- Ubuntu with ROS 2 Jazzy
- Python 3
- `colcon`
- Node.js 18 or newer
- npm
- A running OpenAMR robot or simulation stack for full UI functionality

Install common system dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  nodejs \
  npm \
  ros-jazzy-rosbridge-server \
  ros-jazzy-rosapi \
  ros-jazzy-web-video-server
```

The ROS package also uses normal ROS interfaces such as `rclpy`, `std_msgs`,
`geometry_msgs`, `nav_msgs`, `action_msgs`, and Nav2-related packages available
from a complete ROS/Nav2 installation.

## First-Time Installation

Clone or place this repository at:

```bash
cd ~
git clone <repo-url> openamrobot-ui
cd ~/openamrobot-ui
```

Build the frontend:

```bash
bash scripts/build_frontend.sh
```

Sync the frontend build into the ROS package:

```bash
bash scripts/sync_frontend_to_ros.sh
```

Build the ROS 2 workspace:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

You can also use the helper script:

```bash
cd ~/openamrobot-ui
source /opt/ros/jazzy/setup.bash
bash scripts/build_ros.sh
source ros2/install/setup.bash
```

## Running the UI

Start the robot or simulation stack first in its own terminal. For example,
from the main OpenAMR platform workspace:

```bash
cd ~/openamr-platform-sw
source install/setup.bash
ros2 launch openamrobot_docking bringup_sim.launch.py
```

Then start this UI workspace in another terminal:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py use_sim_time:=true
```

Open the UI:

```text
http://127.0.0.1:5050/control
```

If you are running on a robot or another computer, replace `127.0.0.1` with the
robot/computer IP address:

```text
http://<robot-ip>:5050/control
```

## Simulation, Headless Mode, and Gazebo GUI

For normal UI testing, launch the platform simulation headlessly from the main
`openamr-platform-sw` workspace, then launch this UI workspace separately.
This keeps the UI independent from the platform workspace.

```bash
# Terminal 1: headless platform simulation
cd ~/openamr-platform-sw
source install/setup.bash
ros2 launch openamrobot_docking bringup_sim.launch.py gazebo_gui:=false use_rviz:=false
```

```bash
# Terminal 2: web UI
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py use_sim_time:=true
```

Then open:

```text
http://127.0.0.1:5050/control
```

This starts the recommended independent UI mode:

```text
Gazebo GUI: off
RViz: off
UI: on, from the separate openamrobot-ui workspace
```

Use Gazebo GUI mode only when you need to inspect the world, physics,
collisions, sensor placement, or robot movement visually. In that case, run the
simulation and UI separately:

```bash
# Terminal 1: platform simulation with Gazebo GUI/RViz
cd ~/openamr-platform-sw
source install/setup.bash
ros2 launch openamrobot_docking bringup_sim.launch.py
```

```bash
# Terminal 2: web UI
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py use_sim_time:=true
```

The old `headless:=true` / `headless:=false` arguments and the former
`use_ui:=true` platform shortcut are not used by the current independent UI
setup.

## Launch Options

Recommended UI-only launch:

```bash
ros2 launch openamr_ui_bringup ui.launch.py use_sim_time:=true
```

Direct package launch for the web services and relays:

```bash
ros2 launch openamr_ui_package new_ui_launch.py use_sim_time:=true
```

Helper script equivalent:

```bash
bash scripts/run_ui_backend.sh
```

Use `use_sim_time:=true` with simulation. Use `use_sim_time:=false` or omit it
when running against a physical robot that uses wall time.

## Ports and URLs

Defaults are configured in:

```text
ros2/src/openamr_ui_package/param/config.yaml
```

| Service | Default | Used For |
| --- | --- | --- |
| Flask app | `http://127.0.0.1:5050` | Serves the React UI |
| Rosbridge | `ws://127.0.0.1:9090` | Browser to ROS WebSocket |
| Web video server | `http://127.0.0.1:8080` | Camera/image streams |

The React development server runs on:

```text
http://localhost:3000
```

When running through Flask, the frontend connects to rosbridge using the page
hostname. When running through `localhost:3000`, it falls back to
`ROSBRIDGE_SERVER_IP` in:

```text
web/src/shared/constants/index.js
```

Update that IP if your robot is not at the default `192.168.0.100`.

## Using the Web UI

The app has these main routes:

| Page | URL | Purpose |
| --- | --- | --- |
| Map | `/` | Map view, robot pose, goals, map layers |
| Route | `/route` | Route and waypoint management |
| Control | `/control` | Manual driving, docking, robot control, status |
| Info | `/info` | System/topic information |

Typical operating flow:

1. Start the robot or simulation stack.
2. Start the UI launch from this repository.
3. Open `/control` or `/`.
4. Confirm the ROS connection indicator shows connected.
5. Use manual control, map goal setting, route following, docking controls, and
   status panels as needed.
6. Use the route page to create, edit, save, rename, delete, and select routes
   when the optional helper nodes from `physnode_launch.py` are running.
7. Use map controls only when the Nav2 map server is available.

Hard-refresh the browser after rebuilding frontend assets:

```text
Ctrl+Shift+R
```

## Important ROS Topics

The frontend topic names are defined in:

```text
web/src/shared/constants/index.js
```

Common topics:

| Topic | Direction | Purpose |
| --- | --- | --- |
| `/cmd_vel` | UI publishes | Manual velocity commands |
| `/odom` | UI subscribes | Robot pose and velocity |
| `/ui/map` | UI subscribes | Browser-friendly occupancy grid relay |
| `/global_costmap/costmap` | UI subscribes | Global costmap layer |
| `/local_costmap/costmap` | UI subscribes | Local costmap layer |
| `/scan_filtered` | UI subscribes | Laser scan layer |
| `/plan` | UI subscribes | Planned path |
| `/tf`, `/tf_static` | UI subscribes | Robot/map transforms |
| `/ui/amcl_pose` | UI subscribes | Relayed AMCL pose |
| `/ui/navigate_to_pose/status` | UI subscribes | Relayed Nav2 goal status |
| `/ui/dock_robot/status` | UI subscribes | Relayed dock action status |
| `/ui/undock_robot/status` | UI subscribes | Relayed undock action status |
| `/goal_pose` | UI publishes | Navigation goal |
| `/initialpose` | UI publishes | Initial localization pose |
| `/ui_operation` | UI publishes | Map/route/navigation commands |
| `/ui_message` | UI subscribes | Messages from UI helper nodes |
| `/WP_req` | UI publishes | Request waypoint data |
| `/WayPoints_topic` | UI subscribes | Route waypoint array |
| `/battery_status` | UI subscribes | Battery status |
| `/charge_station_connected` | UI subscribes | Charger connection status |

## What the UI Launch Starts

`openamr_ui_package/launch/new_ui_launch.py` starts:

| Node | Purpose |
| --- | --- |
| `openamr_ui_package/flask` | Serves the React build |
| `rosbridge_server/rosbridge_websocket` | WebSocket bridge for roslibjs |
| `rosapi/rosapi_node` | ROS graph helper services |
| `web_video_server/web_video_server` | Browser camera streaming |
| `openamr_ui_package/map_relay` | Relays `/map` to `/ui/map` |
| `openamr_ui_package/nav_relay` | Relays AMCL/navigation/docking status topics |

Additional helper nodes are available in `openamr_ui_package/launch/physnode_launch.py`
if map/route file operations or waypoint route-following helpers are needed:

| Node | Purpose |
| --- | --- |
| `openamr_ui_package/handler` | Map, group, route, and waypoint file operations |
| `openamr_ui_package/nav` | Route-following helper using Nav2 Simple Commander |

## Frontend Development

Run the React dev server:

```bash
cd ~/openamrobot-ui/web
npm install
npm run dev
```

Open:

```text
http://localhost:3000
```

The frontend can render without ROS, but live robot data and controls require
rosbridge to be running and reachable.

Build production assets:

```bash
cd ~/openamrobot-ui
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
```

Rebuild the ROS package after syncing:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select openamr_ui_package
source install/setup.bash
```

Run frontend linting:

```bash
cd ~/openamrobot-ui/web
npm run lint
```

Run frontend tests:

```bash
cd ~/openamrobot-ui/web
npm test
```

## ROS Development

Build all ROS packages:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build selected packages:

```bash
colcon build --symlink-install --packages-select openamr_ui_msgs openamr_ui_package openamr_ui_bringup
```

Run package tests:

```bash
colcon test --packages-select openamr_ui_package openamr_ui_msgs
colcon test-result --verbose
```

After changing message definitions in `openamr_ui_msgs`, rebuild and source the
workspace again before running any nodes:

```bash
colcon build --symlink-install --packages-select openamr_ui_msgs openamr_ui_package
source install/setup.bash
```

## Map and Route Files

Maps are stored under:

```text
ros2/src/openamr_ui_package/maps/
```

Routes are stored under:

```text
ros2/src/openamr_ui_package/paths/
```

The active map and route are tracked in:

```text
ros2/src/openamr_ui_package/param/current_map_route.yaml
```

Route CSV files contain waypoint pose data used by the route UI and waypoint
navigation helper. The UI helper node can create groups, save routes, edit
routes, rename routes/maps/groups, delete routes/maps/groups, and request the
current waypoint list.

## Normal Development Workflow

For frontend changes:

```bash
cd ~/openamrobot-ui
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh

cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select openamr_ui_package
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py use_sim_time:=true
```

For Python or launch file changes:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py use_sim_time:=true
```

For message changes:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select openamr_ui_msgs openamr_ui_package
source install/setup.bash
```

## Troubleshooting

If the page opens but ROS status is disconnected:

- Confirm `rosbridge_websocket` is running.
- Confirm port `9090` is reachable from the browser machine.
- If using the React dev server, update `ROSBRIDGE_SERVER_IP` in
  `web/src/shared/constants/index.js`.
- Check browser console WebSocket errors.

If the page does not open:

- Confirm the UI launch is still running.
- Confirm Flask is listening on port `5050`.
- Check `ros2/src/openamr_ui_package/param/config.yaml` for the configured host
  and port.

If the map is blank:

- Confirm the robot/simulation stack is publishing `/map`.
- Confirm `map_relay` is running.
- Check that the frontend is subscribed to `/ui/map`.
- Confirm the map server belongs to the main robot/Nav2 stack.

If camera is missing:

- Install `ros-jazzy-web-video-server`.
- Confirm `web_video_server` is running on port `8080`.
- Confirm the camera/image topic exists in ROS.

If UI changes do not appear:

- Rebuild the frontend.
- Sync `web/build` into the ROS package.
- Rebuild the ROS package.
- Hard-refresh the browser with `Ctrl+Shift+R`.

If `colcon` cannot find packages:

- Source ROS first with `source /opt/ros/jazzy/setup.bash`.
- Run commands from `~/openamrobot-ui/ros2`.
- Rebuild and source `install/setup.bash`.

## Notes

- Do not run a second standalone map server from this UI workspace when the
  main Nav2 stack already owns `/map_server`.
- `map_server_launch.py` is a deprecated compatibility launch and is namespaced
  under `ui_legacy` to avoid conflicting with the platform map server.
- Keep heavy UI layers such as camera, laser, and costmaps off unless needed for
  debugging, especially when simulation performance is tight.
- The UI is most useful when the robot or simulation stack is already healthy;
  use RViz and ROS CLI tools for deep Nav2, TF, or costmap debugging.
