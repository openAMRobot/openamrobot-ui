# Troubleshooting Guide

Start with the dashboard's connection indicator and Health page. A green
browser connection does not prove that robot topics are fresh.

> [!CAUTION]
> If motion, localization, or the active robot is uncertain, stop issuing
> commands and verify the physical robot before troubleshooting further.

## Symptom guide

| Symptom | First check |
| --- | --- |
| Page does not open | UI process and port `5050` |
| Page opens, connection is red | Rosbridge process, configured host, firewall, and port `9090` |
| Connection is green, map/pose is frozen | Health page topic freshness; do not drive |
| Map alone is blank | `/map`, `/ui/map`, and `map_volatile_relay` |
| Camera alone is blank | Selected image topic and optional port `8080` service |
| Route/map buttons fail | Optional `physnode_launch.py` helpers |
| UI changes do not appear | Rebuild, synchronize, rebuild ROS, and hard-refresh |
| Program/Mission stalls after WiFi loss | Stop it, verify the robot is stationary, reconnect, and restart deliberately |

## Page does not open

Check the launch or container:

```bash
docker compose logs -f
```

For a manual installation:

```bash
ros2 node list | grep flask_app
```

Confirm Flask is configured for port `5050` in
`ros2/src/openamr_ui_package/param/config.yaml`.

## Browser is disconnected

Confirm rosbridge:

```bash
ros2 node list | grep rosbridge
```

Check that the browser can reach the configured UI computer on port `9090`.
For the frontend development server, verify `ROSBRIDGE_SERVER_IP` in
`web/src/shared/constants/index.js`.

Use the browser developer tools:

1. Open **Network**.
2. Filter for **WS**.
3. Inspect the connection to port `9090`.

## Connected but data is stale

Check the relevant topic rather than restarting rosbridge immediately:

```bash
ros2 topic list
ros2 topic hz /odom
ros2 topic echo /odom --once
```

Inspect publishers, subscribers, and QoS:

```bash
ros2 topic info /odom -v
```

Continue with
[Lesson 11 — Failure Modes and Reconnection](lessons/11-failure-modes-and-reconnection.md)
and [Lesson 12 — Debugging with ROS CLI](lessons/12-debugging-with-ros-cli.md).

## Blank map

```bash
ros2 topic echo /map --once
ros2 topic echo /ui/map --once
ros2 node list | grep map_volatile_relay
```

The robot workspace owns the map server. The UI relay republishes `/map` as
the browser-friendly `/ui/map`.

## Missing camera

Check:

- `web_video_server` is installed and running.
- Port `8080` is reachable.
- The selected image topic exists.
- The image topic is publishing.

```bash
ros2 topic list | grep image
ros2 topic hz /camera/color/image_raw
```

The camera is optional and independent of the rosbridge WebSocket.

## Route or map operations fail

The regular UI launch does not start the file-management helpers. Start:

```bash
ros2 launch openamr_ui_package physnode_launch.py
```

Then check:

```bash
ros2 node list | grep -E "handler|nav"
ros2 topic echo /ui_message
```

## Frontend changes do not appear

```bash
cd ~/openamrobot-ui
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh

cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select openamr_ui_package
source install/setup.bash
```

Restart the launch and hard-refresh with `Ctrl+Shift+R`.

## ROS packages are not found

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep openamr_ui
```

## Collect diagnostics

The Health page can download a support package containing connection
information, the health rollup, recent events, metrics, runtime configuration,
and Nav2 parameters.

Review it for credentials, private addresses, and sensitive operational data
before sharing it.
