# Connecting an External Device

A hands-on guide to getting a new **sensor** (a device the UI only *observes*
— a camera, a temperature probe, a new lidar) or **actuator** (a device the
UI *commands* — a gripper, a light, a secondary motor) visible or
controllable in the browser, or more generally any new ROS topic from the
robot side. This assumes the device already publishes (or will publish) to a
ROS 2 topic somewhere in the robot/simulation workspace — this UI workspace
never talks to hardware directly, only to ROS topics and services (see
[Lesson 01](../lessons/01-what-is-this-ui.md)).

## 1. Define or reuse a ROS topic

Decide the topic name and message type the device will publish (or the
command topic the UI will publish to, for an actuator). Prefer reusing a
standard message type (`std_msgs`, `sensor_msgs`, `geometry_msgs`, …) over
inventing a new one. If nothing standard fits, a custom message can be added
to
[`ros2/src/openamr_ui_msgs/msg/`](../../ros2/src/openamr_ui_msgs/msg/)
alongside the existing `ArrayPoseStampedWithCovariance.msg`.

This step happens entirely on the robot/simulation side — this UI workspace
does not define what the device publishes, it only consumes it.

## 2. Decide whether you need a relay

Ask: **is the topic TRANSIENT_LOCAL (latched), or otherwise likely to publish
before the browser subscribes?** If yes, a browser client connecting through
rosbridge can miss it, the same problem solved for `/map` and AMCL pose — see
[Lesson 04](../lessons/04-data-flow-and-relays.md) for the full explanation.

- **If the topic is a normal, continuously-publishing VOLATILE stream**
  (most sensor data — laser scans, continuous odometry-like readings), skip
  straight to step 3 and subscribe to it directly from the panel.
- **If it's latched, low-frequency, or status-like** (a one-shot
  configuration message, a state that only changes occasionally), add a relay
  node modeled on
  [`ros2/src/openamr_ui_package/openamr_ui_package/map_relay.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/map_relay.py)
  (single topic) or
  [`ros2/src/openamr_ui_package/openamr_ui_package/nav_relays.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/nav_relays.py)
  (multiple topics in one node). Give the browser-facing topic a `/ui/`
  prefixed name to follow the existing convention (e.g. a new
  `/battery_temperature` device topic would relay to `/ui/battery_temperature`
  only if it actually needs the QoS conversion — otherwise just use
  `/battery_temperature` directly).

  Register the new relay node in
  [`ros2/src/openamr_ui_package/launch/new_ui_launch.py`](../../ros2/src/openamr_ui_package/launch/new_ui_launch.py),
  following the existing `map_relay`/`nav_relay` `Node(...)` entries — same
  package, a new `executable` name matching what you add to that package's
  `setup.py` entry points.

## 3. Expose the topic name in the frontend constants file

Add the final browser-facing topic name (relayed or direct) as a new key on
`AppConfig` in
[`web/src/shared/constants/index.js`](../../web/src/shared/constants/index.js).
This is the one line that has to change if the topic is ever renamed — see
[Lesson 10](../lessons/10-topics-as-the-contract.md).

```js
// web/src/shared/constants/index.js
export const AppConfig = {
  // ...existing keys
  BATTERY_TEMPERATURE_TOPIC: "/ui/battery_temperature",
};
```

## 4. Render it in a panel

Follow [`add-a-ui-panel.md`](add-a-ui-panel.md): either add a new component
under [`web/src/components/`](../../web/src/components/) or extend an
existing one (for a small addition to an existing status panel, e.g.
[`web/src/components/SystemHealth.jsx`](../../web/src/components/SystemHealth.jsx)
already follows a "watch a list of topics, show online/offline" pattern that
a new stream can be added to). Use `useRos()` to get the shared connection,
subscribe using the constant added in step 3, and render the resulting state.

For an actuator instead of a sensor — something the UI *commands* rather than
*observes* — the same steps apply, except the panel publishes to the topic
(see
[`web/src/components/DockingControl.jsx`](../../web/src/components/DockingControl.jsx)
for a publish-and-watch-status example: it publishes a trigger and separately
subscribes to a status topic to reflect the result).

## 5. Confirm it end to end

1. Start the robot/simulation workspace with the device (real or simulated)
   publishing.
2. Start this UI workspace (`ros2 launch openamr_ui_bringup ui.launch.py`,
   see the main [README](../../README.md) for full launch instructions).
3. If you added a relay, confirm it's running and check the browser-facing
   topic has data before debugging the frontend:
   ```bash
   ros2 node list | grep <your_relay_node_name>
   ros2 topic echo <your_ui_facing_topic>
   ```
4. Open the page/panel in the browser and confirm the new data appears.

If the topic shows data with `ros2 topic echo` but nothing appears in the
browser, the problem is on the frontend side (wrong constant, subscription
not wired, wrong message type) — not the ROS side. If `ros2 topic echo` shows
nothing, the problem is upstream of this UI workspace entirely.
