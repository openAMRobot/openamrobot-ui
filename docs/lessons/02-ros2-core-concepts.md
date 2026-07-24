# Lesson 02 — ROS 2 Core Concepts

This lesson defines the vocabulary used everywhere else in these lessons and
in the codebase. Each concept links to a real example already in this
repository so the idea has a concrete anchor.

## Node

A node is one running process that does one job inside the ROS 2 graph.
Nodes don't call each other directly — they publish and subscribe to topics,
or offer/call services and actions, and the ROS 2 middleware handles
discovery and delivery between them.

This workspace's own nodes are plain Python classes that subclass
`rclpy.node.Node`. Example: the Flask web server itself is a ROS 2 node, not
just a web server that happens to sit near ROS — see the `ParamFlask(Node)`
class in
[`ros2/src/openamr_ui_package/openamr_ui_package/flask_app.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/flask_app.py).
The map relay and navigation relay are separate, smaller nodes: see
[`map_relay.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/map_relay.py)
and
[`nav_relays.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/nav_relays.py).

## Topic

A topic is a named, typed stream of messages. Any node can publish to a
topic; any node can subscribe to it. Publishers and subscribers don't know
about each other — they only agree on a topic **name** and a message
**type**. That agreement is the whole contract (this idea is developed
further in [Lesson 10](10-topics-as-the-contract.md)).

Example: `/cmd_vel` carries velocity commands the UI publishes and the robot
driver consumes. `/odom` carries robot pose/velocity the UI subscribes to and
the robot stack publishes. Every topic name this frontend depends on is
collected in one place:
[`web/src/shared/constants/index.js`](../../web/src/shared/constants/index.js).

## Message

A message is the data structure carried on a topic — a typed record, similar
to a struct. Most messages used here come from standard ROS 2 packages
(`geometry_msgs/Twist`, `nav_msgs/OccupancyGrid`, `std_msgs/String`, and so
on). When the standard types aren't enough, this workspace defines its own in
a dedicated message package:
[`ros2/src/openamr_ui_msgs/msg/`](../../ros2/src/openamr_ui_msgs/msg/).

Example: a `geometry_msgs/Twist` message (the type carried on `/cmd_vel`) has
exactly two fields — `linear` and `angular` — each an `{x, y, z}` triple. The
UI only ever sets `linear.x` (forward/backward speed) and `angular.z` (turn
rate); the other four numbers stay zero. That's the entire "shape" of a drive
command — nothing more is sent or expected on that topic.

## Service

A service is a request/response call — send one request, get back exactly one
reply, synchronously from the caller's point of view. Use a service when you
need an answer, not a stream. Two examples used in this UI:

- Nav2 lifecycle nodes expose `get_state`/`change_state` services that the
  Health, Fleet, and Config pages poll and call — see
  [`web/src/components/LifecycleStatus.jsx`](../../web/src/components/LifecycleStatus.jsx).
  Example: `get_state` takes an empty request and replies with a single
  `current_state` field whose `label` is a string like `"active"`,
  `"inactive"`, or `"unconfigured"` — that string is exactly what the
  Lifecycle panel's colored dot reflects.
- The Route page asks Nav2 to plan a path with the
  `/compute_path_to_pose` service — see the planning logic in
  [`web/src/pages/RoutePage.jsx`](../../web/src/pages/RoutePage.jsx). The
  request carries a `start` pose and a `goal` pose; the one reply carries the
  full planned `path` (a list of poses) or an empty one if planning failed —
  there's no partial or streaming reply, just the one final answer.

## Action

An action is for long-running work that a service is a poor fit for: you send
a goal, get periodic feedback while it runs, and eventually get a result — and
you can cancel it mid-flight. Nav2's `navigate_to_pose` is an action: the UI
sends a goal pose, watches feedback (distance remaining) and status
(navigating/succeeded/failed), and can cancel it. See
[`web/src/components/NavStatus.jsx`](../../web/src/components/NavStatus.jsx)
for the feedback/status side, and the cancel-goal service call in
[`web/src/pages/MapPage.jsx`](../../web/src/pages/MapPage.jsx) (also called
directly by the E-STOP button in
[`web/src/components/StatusBar.jsx`](../../web/src/components/StatusBar.jsx),
visible on every page).

Concretely, that one navigation looks like three separate messages over
time: a goal pose sent once ("go to x=2, y=1"), a stream of feedback
messages while it drives (each carrying a `distance_remaining` number that
counts down), and one final status update when it's done (a numeric status
code — the UI treats `4` as succeeded, `5`/`6` as canceled/failed). A service
could not represent the "stream of feedback while it's still running" part;
that's precisely what makes this an action instead of a service.

## Launch file

A launch file is a Python script that starts a group of nodes together with
their configuration, instead of starting each node by hand in its own
terminal. This workspace has a small, deliberate layering:

- [`ros2/src/openamr_ui_bringup/launch/ui.launch.py`](../../ros2/src/openamr_ui_bringup/launch/ui.launch.py) —
  the one command most people run; it just includes the package launch below.
- [`ros2/src/openamr_ui_package/launch/new_ui_launch.py`](../../ros2/src/openamr_ui_package/launch/new_ui_launch.py) —
  starts Flask, rosbridge, rosapi, the camera server, and the two relay
  nodes.
- [`ros2/src/openamr_ui_package/launch/physnode_launch.py`](../../ros2/src/openamr_ui_package/launch/physnode_launch.py) —
  optional helper nodes for map/route file management, started separately.

## Next

[Lesson 03 — How the Browser Talks to ROS](03-how-the-browser-talks-to-ros.md)
walks through the actual chain that connects a browser tab to this graph.
