# Worked Example: Adding a Front-Range Sensor

A complete, start-to-finish example combining
[`connect-external-device.md`](connect-external-device.md) and
[`add-a-ui-panel.md`](add-a-ui-panel.md) into one concrete case, instead of
two separate step lists. This is a teaching example — the code below isn't
already in the repo, it's what you'd actually write.

**The scenario:** the robot gets a new front-facing ultrasonic range sensor.
Its driver publishes `sensor_msgs/Range` on `/front_range`, continuously, at
a few Hz. We want a small panel on the Control page showing the current
distance and a warning when something is too close.

## Step 1 — Decide: does this need a relay?

Apply the test from
[Lesson 04](../lessons/04-data-flow-and-relays.md#the-problem-qos-not-code):
is `/front_range` latched (`TRANSIENT_LOCAL`), or otherwise likely to
publish before the browser subscribes? A range sensor driver publishes
continuously and doesn't need late-joining subscribers to see a "last
value" — it's a normal `VOLATILE` stream, the same category as `/odom` or
`/scan_filtered`. No relay needed; skip straight to wiring the topic in
directly. (Confirm this for real with `ros2 topic info /front_range -v` —
see [Lesson 12](../lessons/12-debugging-with-ros-cli.md) — rather than just
assuming.)

## Step 2 — Add the topic name to the constants file

Edit
[`web/src/shared/constants/index.js`](../../web/src/shared/constants/index.js):

```js
export const AppConfig = {
  // ...existing keys
  FRONT_RANGE_TOPIC: "/front_range",
};
```

## Step 3 — Build the panel

A new component,
`web/src/components/FrontRangeIndicator.jsx`, following the same shape every
panel in [Lesson 07](../lessons/07-ui-components.md) uses — `useRos()` for
the shared connection, a topic built from the constant just added, subscribe
in a `useEffect`, clean up on unmount:

```jsx
import React, { useEffect, useRef, useState } from "react";
import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants";

const CLOSE_THRESHOLD_M = 0.3;

const FrontRangeIndicator = () => {
  const ros = useRos();
  const [range, setRange] = useState(null);
  const topicRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    topicRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.FRONT_RANGE_TOPIC,
      messageType: "sensor_msgs/Range",
    });

    topicRef.current.subscribe((msg) => {
      setRange(msg.range);
    });

    return () => topicRef.current?.unsubscribe();
  }, [ros]);

  const isClose = range !== null && range < CLOSE_THRESHOLD_M;

  return (
    <div className="rounded-xl border border-borderSubtle bg-bgCard px-4 py-2 font-[RobotoMono]">
      <p className="mb-1 text-xs uppercase tracking-wider text-themeTextGray">
        Front Range
      </p>
      <p className={`text-sm ${isClose ? "text-statusRed" : "text-textWhiteHover"}`}>
        {range === null ? "No data" : `${range.toFixed(2)} m`}
        {isClose && " — too close"}
      </p>
    </div>
  );
};

export default FrontRangeIndicator;
```

This is exactly the shape from
[`add-a-ui-panel.md`](add-a-ui-panel.md#3-reach-the-shared-ros-connection) —
nothing sensor-specific about the wiring, only the topic name, message type,
and what's done with the message once it arrives.

## Step 4 — Render it on a page

Add it to whichever page makes sense — the Control page, since that's where
drive-safety information belongs
([Lesson 06](../lessons/06-the-five-pages.md#control--websrcpagescontrolpagejsx)):

```jsx
// web/src/pages/ControlPage.jsx
import FrontRangeIndicator from "../components/FrontRangeIndicator";
// ...
<FrontRangeIndicator />
```

No route registration needed here — this is a panel on an existing page, not
a new page, so [step 2 of `add-a-ui-panel.md`](add-a-ui-panel.md#2-register-the-route-new-pages-only)
doesn't apply.

## Step 5 — Verify

1. Confirm the robot side is actually publishing:
   ```bash
   ros2 topic echo /front_range
   ```
2. Rebuild and reinstall the frontend, then hard-refresh the browser — see
   [`add-a-ui-panel.md` step 5](add-a-ui-panel.md#5-confirm-it) for the dev
   vs. production build distinction.
3. Open the Control page and confirm the panel shows live data, and that
   moving something within 0.3m in front of the sensor turns the readout
   red.

If step 1 shows data but the panel stays on "No data," the problem is on the
frontend side — wrong constant, wrong message type, or the component isn't
actually rendered on the page. See
[Lesson 12](../lessons/12-debugging-with-ros-cli.md#a-decision-order-for-common-symptoms)
for the general version of that triage.
