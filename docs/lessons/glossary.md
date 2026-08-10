# Glossary

Every term introduced across the [lessons](README.md), in one place, for
quick lookup. Each entry links to the lesson where it's explained in full —
this page gives the short version only.

**Action** — A goal/feedback/result ROS 2 interaction for long-running,
cancelable work, such as `navigate_to_pose`. See
[Lesson 02](02-ros2-core-concepts.md#action).

**Active context** — The single map and route currently selected on the
Route page, tracked in one file, `current_map_route.yaml`. See
[Lesson 08](08-map-and-route-model.md#the-active-context-one-file-always-current).

**AMCL (Adaptive Monte Carlo Localization)** — The Nav2 node that estimates
the robot's position on the map by matching live LIDAR scans against it,
publishing `/amcl_pose`. Rendered as the "Map-corrected" position badge (with
"AMCL" as a hover tooltip) on the Map, Status, and Robot pages, and as the
localization check on the Health page. See
[`RobotState`](07-ui-components.md#robotstate--robotstatejsx) and
[`SystemHealth`](07-ui-components.md#systemhealth--systemhealthjsx).

**`AppConfig`** — The exported object in
`web/src/shared/constants/index.js` holding every topic/service/action name
the frontend depends on. See
[Lesson 10](10-topics-as-the-contract.md).

**Behavior Tree (`bt_navigator`)** — The Nav2 lifecycle node that
orchestrates navigation as a tree of conditional steps — compute a path,
follow it, run a recovery behavior if something goes wrong — rather than a
single fixed algorithm. One of the five nodes the Lifecycle panel tracks —
see the **Lifecycle node** entry below and
[`LifecycleStatus`](07-ui-components.md#lifecyclestatus--lifecyclestatusjsx).

**Contract (topics as the contract)** — The idea that a topic name plus a
message type is the entire, compiler-unchecked interface between the UI and
the robot. See [Lesson 10](10-topics-as-the-contract.md).

**Costmap** — A grid overlaid on the map where each cell's value reflects
how close it is to an obstacle; Nav2 plans paths that avoid high-cost cells.
Nav2 keeps a "global" costmap (the whole map) and a "local" costmap (just
around the robot), rendered as the Map page's "Obstacles (wide)" and
"Obstacles (near)" layer toggles (labeled "Costmap G"/"Costmap L" in earlier
versions of this UI). See
[`MapLayers`](07-ui-components.md#maplayers--maplayersjsx).

**DDS (Data Distribution Service)** — The pub/sub middleware ROS 2 is built
on. It's what lets nodes discover each other and exchange topics, services,
and actions without a central broker, and it's why a node that restarts
typically rejoins the graph on its own — distinct from the browser's own
rosbridge WebSocket reconnect, which is a separate, application-level retry
this UI implements itself. See [Lesson 11](11-failure-modes-and-reconnection.md).

**Durability (QoS)** — The ROS 2 Quality of Service setting controlling
whether a late-joining subscriber gets the last published message
(`TRANSIENT_LOCAL`) or only future ones (`VOLATILE`). See
[Lesson 04](04-data-flow-and-relays.md#the-problem-qos-not-code).

**Error boundary** — A React component that catches render-time errors
anywhere below it in the tree and shows a fallback UI instead of a blank
crashed page. Only catches render errors, not errors inside event handlers
or `ROSLIB` callbacks. See
[Lesson 07](07-ui-components.md#the-app-shell-routing-providers-and-state).

**Flask** — The Python web server (`flask_app.py`) that serves the compiled
React app and a small REST API; also a ROS 2 node itself. See
[Lesson 05](05-backend-nodes-in-detail.md#flask_apppy--two-unrelated-jobs-in-one-process).

**Generated Plan** — The flat list of action objects Blockly produces from
connected blocks, shown in the Blocks page's right panel. See
[Lesson 09](09-blockly-programming.md#the-pipeline-block--action--execution--ros).

**Group / Map / Route hierarchy** — The three-level structure
(`Group → Map → Route`) that all saved maps and routes are organized under.
See [Lesson 08](08-map-and-route-model.md).

**Launch file** — A Python script that starts a group of ROS 2 nodes
together with configuration. See
[Lesson 02](02-ros2-core-concepts.md#launch-file).

**Lifecycle node** — A ROS 2 node whose runtime state (`unconfigured` →
`inactive` → `active`, plus `cleanup`/`shutdown`) is managed through
explicit `get_state`/`change_state` service calls rather than just
running as soon as it starts — Nav2's `map_server`, `amcl`,
`controller_server`, `planner_server`, and `bt_navigator` all work this
way. See [`LifecycleStatus`](07-ui-components.md#lifecyclestatus--lifecyclestatusjsx).

**Message** — The typed data structure carried on a topic (e.g.
`geometry_msgs/Twist`). See [Lesson 02](02-ros2-core-concepts.md#message).

**Nav2** — The ROS 2 Navigation stack: the family of lifecycle nodes
(`map_server`, `amcl`, `controller_server`, `planner_server`,
`bt_navigator`) that turns a goal pose into an obstacle-avoiding driven
path. Referenced throughout as the thing behind `navigate_to_pose`,
`compute_path_to_pose`, and the Lifecycle panel. See
[Lesson 02](02-ros2-core-concepts.md#action) and
[`LifecycleStatus`](07-ui-components.md#lifecyclestatus--lifecyclestatusjsx).

**Node** — One running ROS 2 process with one job, discovered and connected
to others by the ROS 2 middleware. See
[Lesson 02](02-ros2-core-concepts.md#node).

**Plan Checks** — Validation run on the Generated Plan before `Run` is
enabled — speed limits, missing named locations, risky-action confirmation.
See [Lesson 09](09-blockly-programming.md#plan-checks-one-safety-gate-regardless-of-origin).

**Provider** — In this codebase, a small wrapper component
(`withRouter`, `withStore`, `withErrorBoundary`) that supplies some piece of
app-wide context — routing, the Redux store, error handling — to everything
rendered inside it. Composed together in
`web/src/app/providers/index.js`. See
[Lesson 07](07-ui-components.md#the-app-shell-routing-providers-and-state).

**QoS (Quality of Service)** — A set of delivery-behavior settings attached
to a ROS 2 topic, of which this UI mainly cares about durability. See
[Lesson 04](04-data-flow-and-relays.md#the-problem-qos-not-code).

**React context** — A React mechanism for publishing one value from a
component so anything below it in the tree can read it without manual prop
passing — how the shared ROS connection reaches every panel. See
[Lesson 03](03-how-the-browser-talks-to-ros.md).

**Redux store** — The one piece of centralized frontend state in this
codebase, holding a single slice: the console log message list dispatched
into by `Logs.jsx`. Not a general pattern most panels need — plain
`useState`/`useRef` is the default everywhere else. See
[Lesson 07](07-ui-components.md#the-app-shell-routing-providers-and-state).

**Relay (relay node)** — A small ROS 2 node that republishes a
`TRANSIENT_LOCAL` robot-side topic as `VOLATILE` under a `/ui/`-prefixed
name, so a browser client reliably receives it. See
[Lesson 04](04-data-flow-and-relays.md).

**Reliability (QoS)** — A separate QoS axis (RELIABLE vs BEST_EFFORT, about
retrying dropped packets) not the focus of this UI's relay pattern. See
[Lesson 04](04-data-flow-and-relays.md#the-problem-qos-not-code).

**`ROSLIB.Ros`** — The single shared roslibjs connection object created once
in `App.jsx` and handed to every panel via `useRos()`. See
[Lesson 03](03-how-the-browser-talks-to-ros.md).

**ROS 2 graph** — The set of all currently-running ROS 2 nodes on the
network, discovering each other and exchanging topics/services/actions. See
[Lesson 01](01-what-is-this-ui.md).

**rosbridge (`rosbridge_websocket`)** — The ROS 2 node that exposes a
WebSocket endpoint (port `9090` by default) so a browser can publish,
subscribe, and call services via `roslibjs`. See
[Lesson 03](03-how-the-browser-talks-to-ros.md).

**roslibjs** — The JavaScript client library, loaded as a plain
`<script>` tag, that speaks rosbridge's protocol over WebSocket. See
[Lesson 03](03-how-the-browser-talks-to-ros.md).

**Service** — A ROS 2 request/response call — one request, one reply,
synchronous from the caller's point of view. See
[Lesson 02](02-ros2-core-concepts.md#service).

**TF (transform tree)** — ROS 2's mechanism for tracking how coordinate
frames relate to each other over time — e.g. where the LIDAR sits relative
to the robot's base, and where the robot's base sits relative to the map.
The Health and Status pages check one specific chain,
`map → odom → base_link → lidar_link`; a broken link anywhere in it means
localization and navigation can't work. See
[`SystemHealth`](07-ui-components.md#systemhealth--systemhealthjsx).

**Three-stage build pipeline** — Why editing `web/src` alone doesn't change
what Flask serves: `npm run build` compiles it to `web/build/`,
`sync_frontend_to_ros.sh` copies that into the ROS package source, and
`colcon build` installs it to the share directory Flask actually reads
from. See
[Lesson 05](05-backend-nodes-in-detail.md#flask_apppy--two-unrelated-jobs-in-one-process).

**Topic** — A named, typed stream of messages that any node can publish to
or subscribe to. See [Lesson 02](02-ros2-core-concepts.md#topic).

**Two-workspace model** — The split between the robot/simulation workspace
(owns the robot, Nav2, sensors) and this UI workspace (owns the dashboard,
rosbridge, relays). See
[Lesson 01](01-what-is-this-ui.md#the-two-workspace-model).

**URDF (Unified Robot Description Format)** — The XML file format
describing a robot's physical structure — links, joints, geometry — that
the Robot page's 3D viewer renders. See
[Robot — `RobotDescriptionPage.jsx`](06-the-pages.md#robot--robotdescriptionpagejsx).

**`useRos()` / `useRosStatus()`** — The two hooks exported from `App.jsx`
that every panel uses to reach the shared connection and its status, instead
of creating a connection of its own. See
[`docs/extending/add-a-ui-panel.md`](../extending/add-a-ui-panel.md).

**`/ui/` prefix** — The naming convention for relayed, browser-facing topics
(`/ui/map`, `/ui/amcl_pose`, …), distinguishing them from the original
robot-side topic. See
[Lesson 04](04-data-flow-and-relays.md#the-naming-convention).

**Voice Command** — The Blocks page panel that turns spoken commands into
Blockly blocks via a wake word, the Flask backend, and the Claude API. See
[Lesson 09](09-blockly-programming.md#voice-command).

**`web_video_server`** — The HTTP server that exposes ROS image topics as
plain MJPEG streams — a separate path from rosbridge, used by the Camera
panel. See
[Lesson 03](03-how-the-browser-talks-to-ros.md#the-camera-stream-is-a-third-separate-path).
