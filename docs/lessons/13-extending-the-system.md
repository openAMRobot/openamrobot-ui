# Lesson 13 — Extending the System

If you've read Lessons 01–12, you now have the whole mental model this UI
runs on:

- A browser dashboard talking to ROS through rosbridge, over one shared
  connection ([Lesson 01](01-what-is-this-ui.md),
  [Lesson 03](03-how-the-browser-talks-to-ros.md)).
- Nodes, topics, messages, services, actions, and launch files as the basic
  building blocks ([Lesson 02](02-ros2-core-concepts.md)).
- A relay pattern — robot topic → relay node → browser-safe topic — used
  whenever a robot-side topic's QoS doesn't suit a late-joining browser
  client ([Lesson 04](04-data-flow-and-relays.md)).
- What the rest of `openamr_ui_package` runs beyond the relays — the Flask
  web/API server and the two Route-page backend nodes
  ([Lesson 05](05-backend-nodes-in-detail.md)).
- A full tour of every page — most of them thin compositions of panels that
  subscribe/publish through that one shared connection
  ([Lesson 06](06-the-pages.md)) — and exactly what each of the panels
  behind Map and Route does ([Lesson 07](07-ui-components.md)).
- The group → map → route file hierarchy behind the Route page
  ([Lesson 08](08-map-and-route-model.md)).
- Programs (Blockly), where the same publish/subscribe mechanics get
  wrapped in a block → action → execution pipeline instead of direct panel
  code ([Lesson 09](09-blockly-programming.md)).
- Topic names as the real, unchecked-by-any-compiler interface between the UI
  and the robot, which is why they're centralized in one constants file
  ([Lesson 10](10-topics-as-the-contract.md)).
- What breaks and what recovers on its own when WiFi drops or a process
  restarts ([Lesson 11](11-failure-modes-and-reconnection.md)), and which
  `ros2` command to reach for at each layer when it does
  ([Lesson 12](12-debugging-with-ros-cli.md)).

That's everything needed to make three kinds of changes safely. All three
guides below are hands-on and reference exact files — no more theory from
here. [`docs/extending/README.md`](../extending/README.md) is a one-page
router if you just want to jump straight to the right one, and
[`docs/extending/worked-example-adding-a-sensor.md`](../extending/worked-example-adding-a-sensor.md)
walks the first two guides together through one complete, concrete example.

## Adding a new panel or page to the UI

If you want a new screen, or a new self-contained widget on an existing page
(a new status readout, a new control), follow
[`docs/extending/add-a-ui-panel.md`](../extending/add-a-ui-panel.md). It
covers where the file goes, how to register a route, how to reach the shared
ROS connection, and which constants file to add topic names to.

## Connecting a new external device

If you have a new sensor, actuator, or any new ROS topic on the robot side
that you want visible or controllable from the browser, follow
[`docs/extending/connect-external-device.md`](../extending/connect-external-device.md).
It covers deciding whether the topic needs a relay
([Lesson 04](04-data-flow-and-relays.md)'s pattern), where to register that
relay, and how to expose and render the result in a panel.

## Adding a new Blockly block

If you want a new drag-and-drop block on the Programs page, follow
[`docs/extending/add-a-blockly-block.md`](../extending/add-a-blockly-block.md).
It covers defining the block, registering it in the toolbox, and wiring its
execution — the same three-stage pipeline from
[Lesson 09](09-blockly-programming.md).

All three guides assume you're only adding things — none of them ask you to
modify an existing page's behavior.
