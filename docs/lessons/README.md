# OpenAMRobot UI — Lessons

These lessons explain how this UI actually works, in order. First-time
operators should always begin with Lesson 00. For install/build/run/troubleshoot steps, see
the [top-level README](../../README.md). For hands-on "add a thing" guides,
see [`docs/extending/`](../extending/README.md). For a quick term lookup
instead of a full lesson, see the [glossary](glossary.md).

| # | Lesson | Covers |
| --- | --- | --- |
| 00 | [Your First 10 Minutes](00-your-first-10-minutes.md) | Demo Mode, shared controls, readiness, a safe first session, and software-stop limitations |
| 01 | [What Is This UI?](01-what-is-this-ui.md) | What the UI is/isn't, the two-workspace model, and a safety note on the lack of authentication |
| 02 | [ROS 2 Core Concepts](02-ros2-core-concepts.md) | Node, topic, message, service, action, launch file |
| 03 | [How the Browser Talks to ROS](03-how-the-browser-talks-to-ros.md) | The roslibjs → rosbridge chain, Flask, the camera path |
| 04 | [Data Flow and Relays](04-data-flow-and-relays.md) | Why `map_relay`/`nav_relays` exist; the relay pattern |
| 05 | [Backend Nodes in Detail](05-backend-nodes-in-detail.md) | `flask_app.py`, `folders_handler.py`, `waypoint_nav.py`, `battery.py`, and why the two-stage build exists |
| 06 | [A Tour of Every Page](06-the-pages.md) | All 18 pages — Map, Routes, Maps, Programs, Scheduler, Missions, Status, Robot, Devices, Health, Metrics, Recordings, Events, Console, Parameters, Fleet, Config, Notes — purpose and topics, one real screenshot each |
| 07 | [UI Components in Detail](07-ui-components.md) | Every panel behind Map and Route under `web/src/components/`, plus the app shell (routing, providers, Redux, theme, always-mounted background components) |
| 08 | [The Map and Route File Model](08-map-and-route-model.md) | The group → map → route file hierarchy |
| 09 | [Blockly Visual Programming](09-blockly-programming.md) | The Programs page's block → action → execution pipeline, Voice Command |
| 10 | [Topics as the Contract](10-topics-as-the-contract.md) | Why centralizing topic names matters |
| 11 | [Failure Modes and Reconnection](11-failure-modes-and-reconnection.md) | What breaks, what recovers on its own, what doesn't |
| 12 | [Debugging with ROS CLI Tools](12-debugging-with-ros-cli.md) | Which command to reach for at which layer |
| 13 | [Extending the System](13-extending-the-system.md) | Bridge to the hands-on guides |

Each lesson identifies its audience, approximate reading time, prerequisites,
and learning goal. A small exercise and readiness checkpoint make it clear
when to continue. Source links point to the implementation rather than copying
code that can become stale.

## Reading paths by role

The full 00–13 order is the default and the safest first pass, but if
you're short on time, these narrower paths cover what each role actually
needs:

- **Just operating the robot day to day:** 00, 01, 06, 11, then the
  plain-language part of 12. Skip backend/build internals unless something
  breaks.
- **Automating routine tasks (Scheduler, Missions, Programs):** 00, 01, then
  [Lesson 06](06-the-pages.md)'s Scheduler/Missions/Programs sections, then
  [Lesson 09](09-blockly-programming.md) if Programs specifically is the
  goal.
- **Adding a UI panel or a new device:** 01–04, then straight to
  [`docs/extending/`](../extending/README.md) — the extending guides are
  self-contained enough not to require 05–12 first, though
  [Lesson 10](10-topics-as-the-contract.md) is worth reading before you add
  your first constant.
- **Working on the Route/mapping backend:** 01–05, 08.
- **Working on Blockly specifically:** 01–04, 09, then
  [`docs/extending/add-a-blockly-block.md`](../extending/add-a-blockly-block.md).
- **Maintaining or debugging the deployed system:** everything, but 11 and
  12 are the two to reread when something's actually on fire.
