# Adding a Blockly Block

A hands-on guide to adding a new block to the Blocks page. If you haven't
read it yet, [Lesson 09 — Blockly Visual Programming](../lessons/09-blockly-programming.md)
explains the pipeline this guide walks through mechanically: a block
definition becomes an action object, and an executor turns that action object
into a real ROS publish or service call. Adding a block means touching all
three stages.

## 1. Define the block

Edit
[`web/src/features/blocks/blockDefinitions.js`](../../web/src/features/blocks/blockDefinitions.js).
Add a Blockly JSON block definition — this is what the user sees and drags
onto the workspace:

```js
{
  type: "openamr_beep",
  message0: "beep robot",
  previousStatement: null,
  nextStatement: null,
  colour: "#8b5cf6",
  tooltip: "Trigger a robot beep.",
}
```

Then add a matching case to `blockToAction` in the same file — this is what
converts the connected block into a plain action object in the Generated
Plan:

```js
case "openamr_beep":
  return { type: "beep" };
```

If the block takes fields (a number, a dropdown, …), read them off the block
the same way neighboring cases in `blockToAction` already do, and include
them in the returned action object.

## 2. Add it to the toolbox

Edit [`web/src/features/blocks/toolbox.js`](../../web/src/features/blocks/toolbox.js)
and add an entry under whichever category the block belongs in (Program,
Navigation, Motion, Docking, or Robot State — see
[Lesson 09](../lessons/09-blockly-programming.md#block-categories-at-a-glance)):

```js
{ kind: "block", type: "openamr_beep" }
```

Without this step the block exists but never appears in the left sidebar.

## 3. Execute the action

Edit [`web/src/features/blocks/robotActions.js`](../../web/src/features/blocks/robotActions.js)
and add a case matching the action `type` from step 1:

```js
case "beep":
  // publish to your topic here, following the pattern of the
  // neighboring cases in this file (dockTopic.publish(...), etc.)
  return;
```

If the action needs a new topic name, add it to `AppConfig` in
[`web/src/shared/constants/index.js`](../../web/src/shared/constants/index.js)
first and reference it from here — never inline a topic string, same rule as
every other panel (see
[`docs/extending/add-a-ui-panel.md`](add-a-ui-panel.md#4-add-topic-names-to-the-constants-file--never-inline)).
If the underlying topic doesn't exist on the robot side yet, or needs a
relay, see
[`docs/extending/connect-external-device.md`](connect-external-device.md)
first.

## 4. Confirm it

Rebuild the frontend and reinstall it so Flask serves the updated bundle —
Blockly code doesn't hot-reload through the production Flask server the way
`npm run dev` does:

```bash
cd ~/openamrobot-ui
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
cd ros2
colcon build --packages-select openamr_ui_package
source install/setup.bash
```

Restart the UI launch, hard-refresh the browser (`Ctrl+Shift+R`), open
`/blocks`, and confirm the new block appears in its category, connects under
`start robot program`, shows up correctly in the Generated Plan, and (once
the page shows "Robot connected") actually publishes when run.

Full setup, build-mode notes, and troubleshooting for the Blocks page live in
[`web/src/features/blocks/README.md`](../../web/src/features/blocks/README.md).
