# Extending OpenAMRobot UI

Hands-on guides for adding things to this UI, without changing how any
existing page already behaves. If you haven't read the conceptual lessons
yet, start with [`docs/lessons/`](../lessons/README.md) — these guides
assume you already know what a topic, a relay, and the shared ROS
connection are.

## Which guide do I need?

| I want to... | Guide |
| --- | --- |
| Add a new screen, or a new status/control widget on an existing page | [`add-a-ui-panel.md`](add-a-ui-panel.md) |
| Bring a new sensor, actuator, or any new ROS topic into the browser | [`connect-external-device.md`](connect-external-device.md) |
| Add a new drag-and-drop block to the Programs (Blockly) page | [`add-a-blockly-block.md`](add-a-blockly-block.md) |
| See a complete example of the above, start to finish | [`worked-example-adding-a-sensor.md`](worked-example-adding-a-sensor.md) |

Adding a new **panel that displays a new device's data** usually needs the
first two guides together: decide the topic and relay in
`connect-external-device.md`, then build the panel itself in
`add-a-ui-panel.md`. The worked example walks through exactly that
combination for one concrete case.

## What these guides don't cover

None of these guides ask you to modify the behavior of an existing page or
panel — they're additive by design. If you need to change how something
already works, read the relevant lesson first
([Lesson 06](../lessons/06-the-pages.md) for pages,
[Lesson 07](../lessons/07-ui-components.md) for individual panels,
[Lesson 09](../lessons/09-blockly-programming.md) for Blockly) so the change
stays consistent with the rest of the codebase, then edit the source file
directly — there's no separate guide for modifying existing behavior.
