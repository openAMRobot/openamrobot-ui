# Lesson 03 — How the Browser Talks to ROS

A browser tab cannot speak the native ROS 2 middleware protocol directly.
This UI bridges that gap with a specific, fixed chain of pieces. Understanding
the chain matters because when something doesn't show up in the UI, the fix
is almost always "which link in this chain is broken," not "the React code is
wrong."

## The chain

```text
React app in the browser
        |
        v
roslibjs  (a JavaScript library, loaded as a plain <script> tag)
        |
        v
WebSocket connection  (a single, long-lived, two-way socket — unlike a
                        normal HTTP request, either side can send a message
                        on it at any time without the other asking first)
        |
        v
rosbridge_server  (a ROS 2 node: rosbridge_websocket)
        |
        v
The ROS 2 graph  (topics, services, actions on the robot/simulation side)
```

The same chain, drawn as a full node/topic-level diagram including the
relay nodes ([Lesson 04](04-data-flow-and-relays.md)) and the robot/simulation
side ([Lesson 01](01-what-is-this-ui.md#the-two-workspace-model)):

![OpenAMRobot UI architecture: React app to rosbridge to ROS 2 UI backend nodes to the robot/simulation stack](../assets/openamr_ros_topic_node_schematic.png)

`roslibjs` is not npm-installed — it's a browser script loaded directly in
[`web/public/index.html`](../../web/public/index.html), alongside `ros2d.js`,
`nav2d.js`, `easeljs.js`, and `eventemitter2.min.js` (map-rendering helper
libraries built on top of it). Once loaded, it's available globally as
`window.ROSLIB`.

## Where the connection is opened

There is exactly one `ROSLIB.Ros()` connection for the whole app, created
once in [`web/src/app/App.jsx`](../../web/src/app/App.jsx). It figures out
the rosbridge host from the page's own URL (falling back to a configured IP
only when running the React dev server on `localhost:3000`), opens a
WebSocket to it, and tracks connected/disconnected/error state. That
connection object and its status are handed down through **React context** —
a React feature that publishes a value from one component so any component
below it in the tree can read it, without passing it down manually through
every layer of props in between — so every page and panel can use the same
connection instead of each one making its own. See
[Lesson 10](10-topics-as-the-contract.md) and
[`docs/extending/add-a-ui-panel.md`](../extending/add-a-ui-panel.md) for how a
panel actually consumes it.

Once connected, a page or component creates a `ROSLIB.Topic` (or `Service`,
or action-related topic) bound to that shared connection, and either
`.publish()`s messages to it or `.subscribe()`s to receive them. This is the
same publish/subscribe model described in
[Lesson 02](02-ros2-core-concepts.md) — roslibjs is just the JavaScript-side
client for it. Concretely: when you click the map in Goal Mode, the Map page
builds a message matching `geometry_msgs/PoseStamped` from the click
coordinates and publishes it on the `/goal_pose` topic name; roslibjs turns
that into a JSON message and sends it down the WebSocket; rosbridge turns it
back into a real ROS 2 message and publishes it on the actual `/goal_pose`
topic, exactly as if a native ROS 2 node had published it.

## Flask's separate job: serving the page itself

Before any of the above can happen, the browser has to load the React app's
HTML/JS/CSS in the first place. That's a plain HTTP job, unrelated to ROS
messaging, done by a Flask node:
[`ros2/src/openamr_ui_package/openamr_ui_package/flask_app.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/flask_app.py).
It serves the compiled React build and a small number of HTTP API endpoints
(used by the Blockly page's voice-command feature). Flask is what puts the
page in the browser; rosbridge is what lets that page talk to ROS once it's
there. They are two different servers on two different ports.

## The camera stream is a third, separate path

Camera images do **not** go through rosbridge/roslibjs. Sending image data
over the same WebSocket as everything else would be slow and heavy, so
`web_video_server` exposes ROS image topics as plain MJPEG HTTP streams
instead. The Camera panel builds a normal `<img>` URL pointing at that
server — see
[`web/src/components/Camera.jsx`](../../web/src/components/Camera.jsx). This
is why camera topic names and the camera HTTP port are configured separately
from the rosbridge topic constants (see `CAMERA_PORT` and
`CAMERA_TOPIC_OPTIONS` in
[`web/src/shared/constants/index.js`](../../web/src/shared/constants/index.js)).

## Summary of the three independent connections

| Connection | Protocol | Purpose | Port (default) |
| --- | --- | --- | --- |
| Flask | HTTP | Serves the compiled React app and a few HTTP APIs | `5050` |
| rosbridge | WebSocket | Publish/subscribe/service calls from the browser into ROS | `9090` |
| web_video_server | HTTP | Camera image streaming (MJPEG) | `8080` |

If the page loads but nothing updates, suspect rosbridge. If the page doesn't
load at all, suspect Flask. If everything works except the camera, suspect
`web_video_server` or the selected image topic. Practical troubleshooting
steps for each are in the main [README](../../README.md#troubleshooting).

## Next

[Lesson 04 — Data Flow and Relays](04-data-flow-and-relays.md) explains why a
few robot-side topics get republished under `/ui/*` names before the browser
ever sees them.
