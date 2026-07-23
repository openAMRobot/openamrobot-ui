# Lesson 01 — What Is This UI?

## The one-sentence version

OpenAMRobot UI is a browser dashboard: a React app that lets a person see and
control a robot that is already running somewhere else. It does not run the
robot.

## What this repository is

- A React frontend (`web/`) that draws maps, camera feeds, joysticks, route
  editors, and status panels in a browser.
- A small set of ROS 2 nodes (`ros2/src/openamr_ui_package/`) whose whole job
  is to connect that browser to the **ROS 2 graph** — the set of all running
  ROS 2 nodes on the network, discovering each other and exchanging data over
  topics, services, and actions ([Lesson 02](02-ros2-core-concepts.md) defines
  each of those terms). This workspace's nodes serve the compiled React app,
  bridge WebSocket traffic into ROS topics, stream camera images over HTTP,
  and adapt a few topics so a browser client can actually receive them.

That's it. Everything in this workspace exists to get robot data into a
browser tab and UI clicks back out to the robot. See the full node/topic
diagram in [Lesson 03](03-how-the-browser-talks-to-ros.md#the-chain) once
the vocabulary below makes sense.

**A concrete example of the whole path:** the robot workspace's Nav2 stack
publishes robot pose and velocity on `/odom`. You start this UI workspace;
Flask serves the compiled React page to your browser; the browser opens a
WebSocket to rosbridge (see [Lesson 03](03-how-the-browser-talks-to-ros.md));
the Control page's `RobotState` panel then subscribes to `/odom` over that
connection and renders the numbers you see update on screen. None of that
happens until the robot/simulation workspace is already running and
publishing `/odom` — the UI has nothing to show otherwise.

## What this repository is NOT

- It is **not** Nav2, AMCL, the map server, or any localization/planning
  stack. Those run in the robot or simulation workspace.
- It is **not** the robot driver, motor controller, or sensor stack.
- It is **not** a simulator. Gazebo, if used, lives in the robot/simulation
  workspace too.
- It does not "own" any topic that describes the physical robot — it only
  reads those topics, and in a few cases republishes them under `/ui/*` names
  (see [Lesson 04](04-data-flow-and-relays.md)).

## The two-workspace model

Running the full system always means two separate things are running:

| Workspace | Example location | Owns |
| --- | --- | --- |
| Robot or simulation workspace | e.g. `~/openamr-platform-sw` | The robot/simulator, Nav2, localization, map server, docking, sensors, and every topic that describes the physical robot |
| This UI workspace | `~/openamrobot-ui` (this repo) | The browser dashboard, the WebSocket bridge, the camera web server, and a few small relay nodes |

The UI workspace is started **after** the robot/simulation workspace, and it
connects to topics and services that the robot workspace already publishes.
If you stop the UI, the robot keeps running — the UI is a viewer/controller,
not a dependency of the robot stack. If you stop the robot workspace, the UI
will show "disconnected" or stale data, because there is nothing left to
observe.

## Where this shows up in the code

- The two ROS 2 launch layers live in
  [`ros2/src/openamr_ui_bringup/launch/ui.launch.py`](../../ros2/src/openamr_ui_bringup/launch/ui.launch.py)
  (the recommended entry point) and
  [`ros2/src/openamr_ui_package/launch/new_ui_launch.py`](../../ros2/src/openamr_ui_package/launch/new_ui_launch.py)
  (what actually starts Flask, rosbridge, the camera server, and the relays).
  Neither of these starts Nav2, a map server, or a simulator.
- The browser entry point is
  [`web/src/index.js`](../../web/src/index.js), which mounts
  [`web/src/app/App.jsx`](../../web/src/app/App.jsx) — the file that opens the
  one connection this whole app shares with ROS.

## A safety note: there is no authentication

Nothing in this workspace checks who's connecting. There's no login, no
access control, and no per-user permissions anywhere in the chain — Flask
serves the page to any browser that can reach port `5050`, and rosbridge
accepts commands from any WebSocket client that can reach port `9090`
([Lesson 03](03-how-the-browser-talks-to-ros.md)). Anyone who can reach
those ports on the network can drive the robot, exactly as if they were
sitting at the controls themselves. This is a reasonable trade-off for a
LAN-local operator dashboard, but it means network exposure *is* access
control here: don't put these ports on an untrusted network, and treat
`docker-compose.yml`'s `network_mode: host` or any port-forwarding change
as a decision with real consequences, not just a networking convenience.

## Next

[Lesson 02 — ROS 2 Core Concepts](02-ros2-core-concepts.md) explains the
vocabulary (node, topic, message, service, action, launch file) used
throughout the rest of these lessons.
