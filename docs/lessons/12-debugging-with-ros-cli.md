# Lesson 12 — Debugging with ROS CLI Tools

[Lesson 11](11-failure-modes-and-reconnection.md) introduced three
independent layers — browser↔rosbridge, rosbridge/relays↔ROS 2 graph, and the
robot workspace itself — and the fact that "connected" and "data is fresh"
are different signals. This lesson turns that into a method: which tool to
reach for, at which layer, in which order, instead of guessing. It pulls
together commands that are otherwise scattered across the
[README's troubleshooting section](../../README.md#troubleshooting) and
[`connect-external-device.md`](../extending/connect-external-device.md)'s
verification step, and explains *why* each one is the right one for its
layer.

## Start with what the UI already tells you

Before opening a terminal: the Header's status dot and the
`SystemHealth`/`SystemAlerts` panels
([Lesson 07](07-ui-components.md#systemhealth--systemhealthjsx)) already
encode the two signals from Lesson 11 — connection state and per-topic
freshness — so check those first. They'll usually tell you which of the two
problems you actually have before you touch a command line.

## Layer 1: browser ↔ rosbridge

Open the browser's DevTools, Network tab, filter to `WS` (WebSocket), and
select the connection to port `9090`. You can watch the exact JSON frames
`roslibjs` sends and receives — this is literally the wire format underneath
the abstraction described in
[Lesson 03](03-how-the-browser-talks-to-ros.md#where-the-connection-is-opened).
It's the fastest way to confirm whether a `publish()` call is actually
leaving the browser at all, versus failing silently before it gets that far.
The browser console is the other half of this layer — `roslibjs` and this
codebase log connection errors there.

## Layer 2: the ROS 2 graph itself

These commands work whether or not the browser is even open — they talk
directly to the ROS 2 graph, which is the layer rosbridge sits in front of.

- **`ros2 node list`** — confirms which nodes are actually running. Compare
  against the node lists in [Lesson 02](02-ros2-core-concepts.md#launch-file)
  and [Lesson 05](05-backend-nodes-in-detail.md) for what a given launch
  should have started.
- **`ros2 node info /node_name`** — shows exactly what a specific node
  publishes, subscribes to, and offers as services/actions. Useful for
  confirming a relay ([Lesson 04](04-data-flow-and-relays.md)) is actually
  wired the way its source says it should be.
- **`ros2 topic list`** — confirms a topic exists on the graph at all, before
  worrying about whether it has data.
- **`ros2 topic echo /topic_name`** — the single most useful command here:
  shows live data as it arrives, or silence if nothing is being published.
  This is exactly the check
  [`connect-external-device.md`](../extending/connect-external-device.md#5-confirm-it-end-to-end)
  uses to isolate a ROS-side problem from a frontend one.
- **`ros2 topic hz /topic_name`** — confirms the *rate*, not just presence.
  Distinguishes "publishing, but too slowly for the UI's timeout" from "not
  publishing at all" — directly relevant to
  `SystemHealth`'s per-topic timeout logic
  ([Lesson 07](07-ui-components.md#systemhealth--systemhealthjsx)).
- **`ros2 topic info /topic_name -v`** — shows the QoS profile, including
  durability. This is how you actually *verify*, rather than assume, whether
  a topic is `TRANSIENT_LOCAL` and therefore a relay candidate — the concept
  from [Lesson 04](04-data-flow-and-relays.md#the-problem-qos-not-code).
- **`ros2 service list`** / **`ros2 service call`** — call a service
  directly, bypassing the UI entirely (e.g. `get_state` on a lifecycle node,
  or `compute_path_to_pose`). If a direct call works but the matching UI
  panel doesn't, the problem is in the frontend, not ROS.

## Layer 3: the robot/simulation workspace

Once Layer 2 confirms the UI-side graph looks correct — nodes running,
topics present — and a problem still exists, it's almost always upstream, in
the robot or simulation workspace itself
([Lesson 01](01-what-is-this-ui.md#the-two-workspace-model)). That workspace
is outside this repository; check its own logs and its own Nav2/driver
status rather than continuing to dig in the UI code.

## A decision order for common symptoms

| Symptom | Check first |
| --- | --- |
| Page doesn't load at all | Flask on port `5050` — [Lesson 05](05-backend-nodes-in-detail.md#flask_apppy--two-unrelated-jobs-in-one-process) |
| Page loads, Header says disconnected | `rosbridge_websocket` process and port `9090` — [Lesson 03](03-how-the-browser-talks-to-ros.md) |
| Header says connected, one panel is blank/frozen | `ros2 topic echo`/`hz` on that panel's topic — [Lesson 11](11-failure-modes-and-reconnection.md#two-different-signals-connected-vs-data-is-fresh) |
| `ros2 topic echo` shows data, panel still blank | Frontend problem: wrong constant, wrong message type, check the browser console — Layer 1 above |
| `ros2 topic echo` shows nothing | Upstream of this UI workspace entirely — Layer 3 above |

## Next

This is the last lesson before the practical guides.
[Lesson 13 — Extending the System](13-extending-the-system.md) is the bridge
from everything covered so far to actually adding a panel, a device, or a
Blockly block.
