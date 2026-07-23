# Adding a UI Panel or Page

A hands-on guide to adding a new screen or a new self-contained widget to the
React app, without touching how the existing pages behave. If you haven't
read them yet, [Lesson 06](../lessons/06-the-five-pages.md) and
[Lesson 10](../lessons/10-topics-as-the-contract.md) explain the pattern this
guide walks through mechanically.

## 1. Decide: new page, or panel on an existing page?

- **A new panel/widget** (a status readout, a control, a small chart) that
  lives on an existing page → add a component under
  [`web/src/components/`](../../web/src/components/), following the shape of
  an existing one such as
  [`web/src/components/NavStatus.jsx`](../../web/src/components/NavStatus.jsx)
  or
  [`web/src/components/SystemAlerts.jsx`](../../web/src/components/SystemAlerts.jsx).
  Then import and render it from the page component you want it on (e.g.
  [`web/src/pages/ControlPage.jsx`](../../web/src/pages/ControlPage.jsx)).
  You're done after step 3 below — no routing changes needed.
- **A whole new page** (its own URL, its own nav entry) → add a component
  under [`web/src/pages/`](../../web/src/pages/), following the shape of an
  existing page such as
  [`web/src/pages/InfoPage.jsx`](../../web/src/pages/InfoPage.jsx) (it's the
  shortest one). Continue to step 2.

## 2. Register the route (new pages only)

Two files need updating together:

- [`web/src/pages/index.jsx`](../../web/src/pages/index.jsx) — add a
  `<Route path="..." element={<YourPage />} />` entry inside the existing
  `<Route path="/" element={<AppLayout />}>` block, alongside `MapPage`,
  `RoutePage`, `ControlPage`, `BlocksPage`, and `InfoPage`.
- [`web/src/components/Header.jsx`](../../web/src/components/Header.jsx) —
  add an entry to the `NAV_LINKS` array (`{ to: "/your-path", label: "Your
  Label" }`) so the new page shows up in both the desktop nav and the mobile
  dropdown menu, which both render from that same array.

## 3. Reach the shared ROS connection

Every page and panel reads the **one** ROS connection the whole app shares —
created once in
[`web/src/app/App.jsx`](../../web/src/app/App.jsx) — through two hooks
exported from that same file:

- `useRos()` — returns the shared `ROSLIB.Ros` instance (or `null` before the
  app has created it).
- `useRosStatus()` — returns the current connection status string
  (`"connected"` | `"disconnected"` | `"error"`), useful for showing a
  connected/offline indicator without wiring your own listener.

A panel never creates its own `ROSLIB.Ros()` connection. The standard shape,
copied from any existing component, is:

```jsx
import { useEffect, useRef } from "react";
import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants";

const YourPanel = () => {
  const ros = useRos();
  const topicRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    topicRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.YOUR_TOPIC,
      messageType: "std_msgs/String",
    });

    topicRef.current.subscribe((msg) => {
      // handle msg
    });

    return () => topicRef.current?.unsubscribe();
  }, [ros]);

  // ...render using state updated by the subscription
};
```

To publish instead of subscribe, call `topicRef.current.publish(new
window.ROSLIB.Message({ ... }))` from an event handler — see
[`web/src/components/Joystick.jsx`](../../web/src/components/Joystick.jsx)
for a minimal publish-only example. See
[`web/src/components/LifecycleStatus.jsx`](../../web/src/components/LifecycleStatus.jsx)
for a `ROSLIB.Service` example if what you need is a request/response call
instead of a topic.

A panel should only declare the topics it actually needs — don't reach into
another panel's subscriptions, and don't subscribe to something "just in
case." This keeps each panel self-contained and easy to delete or move later.

## 4. Add topic names to the constants file — never inline

Every topic, service, or action name goes in
[`web/src/shared/constants/index.js`](../../web/src/shared/constants/index.js),
as a new key on the exported `AppConfig` object (or, for a small related
group like camera options or lifecycle node names, as its own named export —
see `CAMERA_TOPIC_OPTIONS` and `LIFECYCLE_NODES` in that same file for the
pattern). Import the name from there; never write the topic string directly
inside a component. This is what keeps the file the single place that
answers "what does the UI depend on" — see
[Lesson 10](../lessons/10-topics-as-the-contract.md).

If the topic doesn't exist on the robot side yet, or needs QoS adjustment
before a browser client can reliably receive it, see
[`connect-external-device.md`](connect-external-device.md) first.

## 5. Confirm it

Run the frontend dev server and check the new page/panel renders and, once
rosbridge is reachable, subscribes or publishes as expected:

```bash
cd web
npm install
npm run dev
```

Full build/deploy steps are in the main [README](../../README.md).
