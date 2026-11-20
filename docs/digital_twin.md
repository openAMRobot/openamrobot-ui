# Operator UI on the digital twin

The browser UI connects to the digital twin the same way it connects to the robot: through rosbridge, with
the same pages, the same status and the same Stop (software) and Resume path. This page covers what the UI
uses from the twin, how to start it, what the operator status panel shows, and the browser acceptance check.

Simulation evidence only. Nothing here is hardware, physical or safety acceptance. Stop (software) is a
functional stop, not the hardwired E-stop and not a safety function.

The Digital Twin Control Room (openamr-upperbody-sw) stays the read-only engineering monitor: CAN signal
paths, playback and offline reports, with no command path. This UI is the operator interface. Neither
replaces the other.

## Command path

The UI publishes no velocity on `/cmd_vel` or any `cmd_vel_*` topic. With openamr-platform-sw#90 on the
stack, manual drive goes `/ui/cmd_vel` (TwistStamped) to `ui_cmd_vel_relay_node`, `cmd_vel_joy`,
twist_mux, the stop gate (`cmd_vel_gated`), velocity_smoother and collision_monitor, which is the only
publisher of `/cmd_vel`. Navigation goals go on `/goal_pose` to `goal_pose_forwarder_node`, which sends one
NavigateToPose goal on the public `navigate_to_pose` action.

On the twin, manual driving from the UI stays disabled: the restricted bridge does not let the browser
publish `/ui/cmd_vel` unless the profile is started with `manual_drive:=true`, which is only for a stack
that carries #90 and only once #90's conditions are met.

Without #90 on the stack there is no stop gate and no `/safety/stop_lock`. The lock state stays unknown,
Stop (software) and Resume cannot work, goals are not sent from the UI, and the status panel says so.

## What the UI uses and what the twin provides

Twin = `openamrobot_bringup/bringup.launch.py sim:=true use_docking:=false` from openamr-platform-sw at
#90's head, or the Build 1 scene of openamr-upperbody-sw (`stack:=build1`), which runs the same Nav2
launch files. "UI relay" means a node of this repository started by the profile.

| UI view | Interface | Type | Direction | Twin source | State |
| --- | --- | --- | --- | --- | --- |
| Map | `/ui/map` | nav_msgs/OccupancyGrid | read | map_server `/map` through `map_relay` (UI relay) | match |
| Map overlays | `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan`, `/scan_filtered` | OccupancyGrid, Path, LaserScan | read | Nav2, scan body filter | match |
| Robot pose | `/odom`, `/ui/amcl_pose`, `/tf`, `/tf_static` | Odometry, PoseWithCovarianceStamped, TFMessage | read | gz bridge, AMCL through `nav_relays` | match |
| Telemetry, Robot Description | `/joint_states`, `/diagnostics`, `/rosout` | JointState, DiagnosticArray, Log | read | robot_state_publisher, navigation_status_node, all nodes | match (platform twin: wheel joints only) |
| Battery | `/battery_status` | std_msgs/Float32 | read | none; the SIL publishes `/sil/bms/battery_state` (sensor_msgs/BatteryState) | gap: battery shows no value |
| Camera | web_video_server, `/rgb_image` | Image | read | gz bridge `/rgb_image` | not started by the twin profile (camera off) |
| Readiness, health, controller state | `/navigation/status` | openamr_nav_msgs/NavigationStatus | read | navigation_status_node | match; shown by the operator status panel |
| Current task | `/navigation/status` task | NavTaskStatus | read | navigation_status_node | match |
| Mission phase | `/baseline1/mission_status` | std_msgs/String JSON | read | Baseline 1 mission (Proposed contract, openamr-upperbody-sw#82) | Proposed; only while a Baseline 1 mission runs |
| Navigation status | `/ui/navigate_to_pose/status`, `/navigate_to_pose/_action/feedback` | GoalStatusArray, feedback | read | goal proxy (#90) or bt_navigator, through `nav_relays` | match |
| Navigation goal | `/goal_pose` | PoseStamped | write | goal_pose_forwarder_node (#90); bt_navigator directly without #90 | match with #90 |
| Cancel | `/navigate_to_pose/_action/cancel_goal` | action_msgs/CancelGoal | service | goal proxy (#90) or bt_navigator | match |
| Stop lock | `/safety/stop_lock` | std_msgs/Bool | read | safety_stop_gate_node (#90) | #90 only |
| Stop state and refusal reason | `/safety/stop_state` | openamr_nav_msgs/StopState | read | safety_stop_gate_node (#90) | #90 only; shown by the operator status panel |
| Stop (software), Resume | `/safety/stop_lock_engage`, `/safety/stop_lock_release` | std_srvs/Trigger | service | safety_stop_gate_node (#90) | #90 only |
| Stop cancels | `navigate_to_pose`, `navigate_through_poses`, `compute_path_to_pose`, `dock_robot`, `undock_robot` cancel services | CancelGoal | service | Nav2 and #90 proxies; docking servers absent | gap: the gate's list and the UI's list differ, and neither covers `/spin`, `/follow_path` or the station alignment server (review of #50) |
| Manual drive | `/ui/cmd_vel` | TwistStamped | write | ui_cmd_vel_relay_node (#90) | disabled on the twin (not in the bridge write set) |
| Clock | `/clock`, `/ui_cmd_vel_relay_node/get_parameters` | Clock, GetParameters | read, service | gz bridge, relay (#90) | match |
| Lifecycle panel | `/<node>/get_state` for map_server, amcl, controller_server, planner_server, bt_navigator | GetState | service | Nav2 | match, read only |
| Docking | `/dock_trigger`, `/undock_robot`, `/dock_trigger_status` | Bool, String | write, read | none (`use_docking:=false`) | off on the twin; trigger topics not in the write set |
| Initial pose | `/initialpose` | PoseWithCovarianceStamped | write | AMCL | not in the write set; the twin sets its initial pose |
| Routes, maps, recordings | `/nav_data_req`, `/new_way_point`, `/ui_operation`, Flask APIs | String, custom | write | `folders_handler` (UI) | not started by the twin profile |

## Start the twin with the UI

Build the platform twin workspace (openamr-platform-sw with #90, openamrobot-interfaces `openamr_nav_msgs`)
and this repository's workspace with a synced frontend:

```bash
bash scripts/build_frontend.sh && bash scripts/sync_frontend_to_ros.sh
source /opt/ros/jazzy/setup.bash
source <platform workspace>/install/setup.bash
bash scripts/build_ros.sh && source ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch openamr_ui_bringup twin.launch.py
```

Open `http://127.0.0.1:5050`. Arguments: `stack:=platform|build1|none`, `gui:=true` (Gazebo window),
`bridge_address`, `bridge_port`, `ui_address`, `ui_port`, `manual_drive` (default `false`).

`/api/build-info` names the commit and the sha256 of the served main JS. Without a matching
`build-info.json` the app answers "No verified frontend build" instead of serving an old page: rebuild and
sync the frontend, then rebuild the ROS package.

## Restricted rosbridge

`ros2/src/openamr_ui_bringup/config/twin_bridge_allowlist.yaml` lists every topic the browser may read and
write and every service it may call; actions: none. Everything else is refused by rosbridge. rosapi runs
with `params_glob` `[]`, so no node parameter can be read or set through the bridge. The bridge and the
Flask app bind `127.0.0.1` by default; to use the UI from another computer, open an SSH tunnel
(`ssh -L 5050:127.0.0.1:5050 -L 9090:127.0.0.1:9090 <twin host>`) instead of binding a wider address.
There is no authentication and no TLS: never expose these ports on a shared network.

## Operator status panel

The panel under the Stop banner shows one display state, most severe first, and the rows behind it. It
only subscribes; it sends nothing.

| Display | When |
| --- | --- |
| DISCONNECTED | rosbridge is not connected; nothing shown is live |
| STOPPED | Stop (software) is engaged (lock, local latch or the gate's state); a refused Resume shows the gate's reason |
| STOP STATE UNKNOWN | no recent `/safety/stop_lock` sample; treated as stopped |
| WAITING FOR STATUS | connected, no `/navigation/status` yet |
| STALE | `/navigation/status` older than 6 s or `/odom` older than 3 s (wall time in the browser) |
| FAULTED | `/navigation/status` health FAULT, with the active reasons |
| NOT READY | navigation readiness NOT_READY, with the reasons by name |
| READY | navigation reports ready |

The stale limits are display values, not safety limits. The mission phase row is labelled Proposed while
`/baseline1/mission_status` is a Proposed contract.

## Automated browser check

`.github/workflows/twin-e2e.yml` starts the platform twin at #90's head with this profile, probes the
bridge (`scripts/twin_bridge_probe.py`) and runs `web/e2e-twin/twin.spec.js` in headless Chromium:
verified bundle, connection, map and pose, Resume, a goal with Cancel, Stop during a goal, the Proposed
mission status (a test message), STALE and DISCONNECTED. Run by hand with the twin up:

```bash
cd web && TWIN_ROS_ENV=/path/to/env.sh npx playwright test -c playwright.twin.config.js
```

## Browser acceptance check (desktop)

For the UI lead, the simulation checker and the software lead, each on their own Ubuntu 24.04 desktop.
Use the same commits as the automated run. Each step says what to do, what you should see, and what to
send back. Write PASS, FAIL or BLOCKED per step; BLOCKED is not PASS.

1. Record the commits: `git rev-parse HEAD` in this repository, openamr-platform-sw and
   openamrobot-interfaces. Send: the three hashes.
2. Start the twin: `ros2 launch openamr_ui_bringup twin.launch.py gui:=true`. See: the Gazebo window with
   the robot. Send: the first 200 lines of the launch output if anything fails.
3. Open `http://127.0.0.1:5050/api/build-info`. See: `"ok": true` and the commit of step 1. Send: the JSON.
4. Open `http://127.0.0.1:5050/map`. See: header "Robot connected", the map, the robot on it, the status
   panel with Robot pose "fresh" and Nav status "fresh". Send: a screenshot.
5. Status panel. See: Readiness with its reasons (the platform twin reports NOT READY with BASE_LINK_LOST
   until a base status adapter exists), Stop state LOCKED after start. Send: the panel text.
6. Press Resume, then Confirm resume. See: Stop state RUNNING within a few seconds, the red banner gone. If
   it stays RELEASE_REFUSED, the panel shows the gate's reason. Send: the Stop state text.
7. Save a waypoint about 1 m in front of the robot (right click on the map, Save waypoint, give it a name, Save), then press its
   button. See: Navigation status "Navigating", Task "Navigation ACTIVE", the robot moving forward in Gazebo.
   Press Cancel. See: "Canceled", the robot stops. Send: screenshot after Cancel.
8. Send the goal again and press Stop (software) while it moves. See: STOPPED at once, the robot stops,
   Task "Navigation CANCELED" (not ABORTED). In a terminal: `ros2 topic info -v /cmd_vel` lists only
   collision_monitor as publisher. Send: the screenshot and the terminal output.
9. Press Resume and confirm. See: RUNNING; the cancelled goal does not restart and the robot stays still for
   at least 5 s. Send: PASS or FAIL.
10. Check that manual drive stays disabled: the joystick does not move the robot. In a terminal:
    `ros2 topic info -v /ui/cmd_vel` shows no publisher from rosbridge. Send: the output.
11. In a terminal: `kill -STOP $(pgrep -f navigation_status_node)`. See: STALE within 10 s. Then
    `kill -CONT $(pgrep -f navigation_status_node)`. See: STALE clears. Send: PASS or FAIL.
12. Stop the launch with Ctrl+C. See: DISCONNECTED and the Stopped (software) banner in the browser. Send:
    a screenshot.

Not covered by this check: FAULTED on the live twin (no fault is injected; covered by unit tests), the
Build 1 scene (`stack:=build1`), hardware, the hardwired E-stop.
