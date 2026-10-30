# OpenAMRobot UI Package

This ROS 2 package contains the Flask server, rosbridge launch integration,
camera web streaming launch integration, map/navigation relays, and optional
map/route helper nodes used by the OpenAMRobot browser UI.

The top-level `../../../README.md` is the source of truth for workspace setup,
build commands, run commands, ports, topics, and troubleshooting.

## Launch files

- `launch/new_ui_launch.py`: starts the web UI services and browser-friendly
  relays.
- `launch/physnode_launch.py`: starts optional map/route file-operation and
  waypoint route-following helper nodes.
- `launch/map_server_launch.py`: deprecated compatibility launch, namespaced
  under `ui_legacy` to avoid conflicting with the platform map server.
- No launch file in this package starts SLAM, AMCL or the Nav2 controller,
  planner or navigator. The only one that still starts a map_server is
  `launch/map_server_launch.py` above, which openamrobot-ui#41 removes. Mapping
  and navigation run on the robot platform; the Maps page saves and loads maps
  through the platform's map services (openamrobot-ui#32).

## Runtime pieces

- `flask_app.py`: serves the compiled React app.
- `map_relay.py`: republishes `/map` to `/ui/map` with browser-friendly QoS.
- `nav_relays.py`: republishes AMCL and navigation/docking action status under
  `/ui/*`.
- `folders_handler.py`: handles map, group, route, and waypoint file commands.
- `waypoint_nav.py`: optional route-following helper using Nav2 Simple
  Commander.
- `battery.py` (executable `battery`): optional battery-percentage
  publisher. Starts from `new_ui_launch.py` only with
  `start_battery_monitor:=true`. It reads a serial port only when
  `battery_port` names one (Gate A legacy); otherwise it publishes a simulated
  drain for development. On 2.0 the battery state comes from the base
  controller.

## Voice Command API key

`flask_app.py` reads `ANTHROPIC_API_KEY` from the environment for the
`/api/voice-plan` endpoint (Blockly's Voice Command feature). Copy
`.env.example` in this directory to `.env` and set the key there — it's
gitignored, and `launch/new_ui_launch.py` loads it and injects it only into
the `flask_app` node's process, so no shell `export` is needed. See
[launch/README.md](launch/README.md) for details.

## Static app

The React source lives in the repository-level `web/` directory. Production
assets are built and copied into:

```text
openamr_ui_package/static/app/
```

Use the repository-level scripts:

```bash
cd ~/openamrobot-ui
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
```

Then build the ROS 2 workspace from the `ros2/` workspace:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Or use the helper script from the repository root:

```bash
cd ~/openamrobot-ui
source /opt/ros/jazzy/setup.bash
bash scripts/build_ros.sh
source ros2/install/setup.bash
```

After sourcing, confirm the package is installed:

```bash
ros2 pkg prefix openamr_ui_package
```

Run the recommended UI launch:

```bash
ros2 launch openamr_ui_bringup ui.launch.py
```

Open the browser at:

```text
http://127.0.0.1:5050/
```
