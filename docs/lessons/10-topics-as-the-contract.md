# Lesson 10 — Topics as the Contract

| Audience | Time | Prerequisites |
| --- | --- | --- |
| Frontend and ROS developers | 8 minutes | [Lesson 03](03-how-the-browser-talks-to-ros.md) |

## What you'll learn

You will learn why topic, service, and action names form a cross-workspace
contract and why browser code should use centralized constants.

## There is no compiler checking this

The UI (a JavaScript app) and the robot stack (a collection of independent
ROS 2 processes, often in a completely separate workspace or on a separate
machine) never import each other's code and are never built together.
Nothing checks, at build time, that a topic the UI subscribes to actually
exists on the robot side, or that the message type the UI expects matches
what's actually published. The only thing connecting them is an agreed-upon
**topic name string** and an agreed-upon **message type string**, as
introduced in [Lesson 02](02-ros2-core-concepts.md). If the UI subscribes to
`/ui/map` and nothing publishes `/ui/map`, nothing errors — the map panel
just stays empty, silently.

That means the topic name itself is the interface. Renaming a topic on one
side without updating the other doesn't produce a stack trace; it produces a
panel that mysteriously stopped updating. This is why topic names deserve the
same care a function signature or an API schema would get in a normal
codebase — they're just harder to verify automatically, so the discipline has
to be manual.

## Why centralizing the names matters

Nearly every topic, service, and action name this frontend depends on is
collected in one file:
[`web/src/shared/constants/index.js`](../../web/src/shared/constants/index.js)
(plus the small `LIFECYCLE_NODES` and `CAMERA_TOPIC_OPTIONS` lists in the
same file for the handful of places that need more than a single name). This
buys three things:

1. **One place to answer "what does the UI actually depend on."** Reading
   this one file tells you the entire topic-level contract, without grepping
   every page and component.
2. **One place to fix a rename.** If the robot side renames `/scan_filtered`,
   there is exactly one line to change here, instead of hunting for every
   file that might have hardcoded the string.
3. **No silent typo-mismatches.** A string literal duplicated in three files
   can drift — one gets updated, two don't. A single imported constant
   can't drift; every consumer reads the same value.

Pages and components import from this file rather than writing topic strings
inline — see how
[`web/src/pages/MapPage.jsx`](../../web/src/pages/MapPage.jsx) or
[`web/src/components/SystemHealth.jsx`](../../web/src/components/SystemHealth.jsx)
reference `AppConfig.GOAL_POSE_TOPIC`, `AppConfig.SCAN_TOPIC`, and so on
instead of the literal topic strings.

The discipline isn't perfect: `/rosout`
([`RosoutConsole.jsx`](../../web/src/components/RosoutConsole.jsx)),
`/reinitialize_global_localization`
([`LocalizationStatus.jsx`](../../web/src/components/LocalizationStatus.jsx)),
and `/diagnostics`
([`useSystemDiagnostics.js`](../../web/src/shared/hooks/useSystemDiagnostics.js))
are hardcoded string literals rather than `AppConfig` entries — a handful of
exceptions worth knowing about before you trust `AppConfig` as a truly
exhaustive list, and worth fixing the next time you're in one of those
files. It's also one-sided: the Python backend nodes covered in
[Lesson 05](05-backend-nodes-in-detail.md) (`/map`, `/amcl_pose`,
`ui_operation`, `/WayPoints_topic`, and more) have no equivalent shared
constants module — every backend file hardcodes its own topic strings ad
hoc. The contract this lesson describes is only centrally documented on the
frontend side of it.

## Relays are part of the same contract

[Lesson 04](04-data-flow-and-relays.md) covered *why* relay nodes exist. From
the contract's point of view, a relay just means the "official" browser-facing
name (`/ui/map`) is different from the robot's original publishing name
(`/map`). The UI-side constant always stores the *final* name the browser
actually subscribes to — the relay is an implementation detail on the ROS
side that the frontend doesn't need to know about beyond that one string.

## Try it

Choose one constant in `web/src/shared/constants/index.js` and find every
publisher or subscriber that relies on it. Confirm its message type on both
sides.

**You're ready to continue when:** you know all files that must change when a
topic name or message type changes.

## Next

[Lesson 11 — Failure Modes and Reconnection](11-failure-modes-and-reconnection.md)
covers what happens when this contract breaks down at runtime — WiFi drops,
rosbridge restarts, the robot workspace crashes.
[Lesson 12](12-debugging-with-ros-cli.md) then turns those failure modes into
a debugging method before [Lesson 13](13-extending-the-system.md) bridges from
theory to practical extension guides.

---

[← Lesson 09](09-blockly-programming.md) · [Lesson index](README.md) ·
[Next: Lesson 11 →](11-failure-modes-and-reconnection.md)
