# Lesson 11 — Failure Modes and Reconnection

Every lesson so far described the happy path: robot running, UI running,
everything connected. On a mobile robot, WiFi drops, processes restart, and
browsers get refreshed constantly — this lesson covers what actually happens
in each case, and, more importantly, teaches the one distinction that
resolves most "is it broken" confusion: **connection state and data
freshness are two different signals.**

## The WebSocket connection: a fixed 1-second retry, forever

The reconnect logic lives entirely in
[`web/src/app/App.jsx`](../../web/src/app/App.jsx), covered structurally in
[Lesson 03](03-how-the-browser-talks-to-ros.md). What wasn't covered there:
on `close` or `error`, it doesn't back off — it retries on a fixed interval
(`AppConfig.RECONNECTION_TIME`, 1000&nbsp;ms) indefinitely, with no cap on
attempts. That's a deliberate simplicity trade-off for a LAN-local robot UI:
it means the browser will reconnect within about a second of rosbridge
becoming reachable again, at the cost of hitting the WebSocket endpoint once
a second while it's down. This is also why the Header's status dot, the
`SystemHealth`/`SystemAlerts` panels
([Lesson 07](07-ui-components.md#systemhealth--systemhealthjsx)), and the
Blocks page's `Run` button all update within about a second of the
connection actually dropping or returning — they all read the same
`RosStatusContext` value set by this one retry loop.

## What survives a reconnect — and what doesn't automatically

Individual panels never re-run their subscription setup after a reconnect —
the `useEffect` that creates each `ROSLIB.Topic` only runs once, when `ros`
first becomes available (see the pattern in
[`docs/extending/add-a-ui-panel.md`](../extending/add-a-ui-panel.md)). That
works because `roslibjs`'s `Topic` defaults to `reconnect_on_close: true`: it
automatically re-sends its subscribe (or advertise) message once the
underlying WebSocket reopens, without any of this codebase's own code having
to notice or react. So a panel that was subscribed before a drop is
subscribed again after one, with no visible gap in its own logic.

What does *not* recover automatically: anything waiting on a one-time
response mid-flight when the connection drops — a `ROSLIB.Service` call
(`get_state`, `compute_path_to_pose`, the navigation cancel service) that
was in flight, or a Blockly program run
([Lesson 09](09-blockly-programming.md)) sitting in the middle of a
`wait until navigation complete` step — has no built-in retry. It simply
never gets its callback, and (depending on the specific code path) can sit
waiting until you notice and intervene, rather than failing loudly.

## Two different signals: "connected" vs "data is fresh"

This is the most useful distinction in this lesson. `RosStatusContext`
(`connected`/`disconnected`/`error`) only reflects whether the **browser's
WebSocket to rosbridge** is open. It says nothing about whether the *robot
side* is actually publishing anything. If the robot or simulation workspace
crashes but `rosbridge_websocket` itself keeps running, the WebSocket never
closes — `RosStatusContext` stays `"connected"` — while every topic
subscription simply stops receiving new messages.

That second condition is exactly what `SystemHealth`'s per-topic
online/offline tracking exists for
([Lesson 07](07-ui-components.md#systemhealth--systemhealthjsx)): each
watched topic is judged by *when its last message arrived*, independently of
whether the WebSocket itself is open. In practice this means: if the Header
says `Connected` but the map or robot pose looks frozen, don't waste time
checking rosbridge — go straight to `SystemHealth` and the robot/simulation
workspace's own logs, because the browser-to-rosbridge link is fine and the
problem is upstream of it.

## Three independent layers, three independent recoveries

There are really three separate connections in play, each with its own
recovery behavior and no shared code between them:

| Layer | What it is | How it recovers |
| --- | --- | --- |
| Browser ↔ rosbridge | The WebSocket from [Lesson 03](03-how-the-browser-talks-to-ros.md) | `App.jsx`'s 1-second retry loop, described above |
| rosbridge/relays ↔ ROS 2 graph | Native ROS 2 node discovery (DDS) | Self-heals automatically — this UI has no code for it, it's a ROS 2 middleware property |
| Robot/simulation workspace itself | The process(es) covered in [Lesson 01](01-what-is-this-ui.md) | Whatever restart procedure that workspace defines — outside this repo entirely |

A robot workspace restart doesn't kill `map_relay`/`nav_relays` — they're
independent processes that simply have nothing to relay until the robot side
comes back, at which point their own subscriptions pick data up again with
no explicit reconnect logic of their own (that's the DDS discovery layer
doing its job). A browser hard refresh is the bluntest fix available: it
tears down and recreates the `ROSLIB.Ros()` connection from scratch, so
"just refresh the page" genuinely resolves a class of stuck-frontend-state
issues that no amount of waiting for the 1-second retry loop will.

## Next

[Lesson 12 — Debugging with ROS CLI Tools](12-debugging-with-ros-cli.md)
turns this into a repeatable method: which command to reach for at each
layer, in order, before you start guessing.
