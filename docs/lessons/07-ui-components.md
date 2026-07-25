# Lesson 07 — UI Components in Detail

| Audience | Time | Prerequisites |
| --- | --- | --- |
| Frontend developers | 20 minutes | [Lesson 03](03-how-the-browser-talks-to-ros.md) and [Lesson 06](06-the-pages.md) |

## What you'll learn

You will learn how shared components consume the ROS connection, how the app
shell keeps background features mounted, and where component state lives.

[Lesson 06](06-the-pages.md) toured every page at a glance. This lesson goes
one level deeper on the shared panels behind Map and Route specifically —
the two pages built as a composition of smaller panels under
[`web/src/components/`](../../web/src/components/) — what each one renders,
exactly which ROS names it uses (from
[`web/src/shared/constants/index.js`](../../web/src/shared/constants/index.js)
unless noted), which page(s) render it today, and anything about its
internal behavior that isn't obvious from the name. Programs/Blockly gets its
own deep dive in [Lesson 09](09-blockly-programming.md) instead, since it
isn't built this way.

### A note on the old Control page

Earlier versions of this UI had a separate Control page. It's gone now —
its manual-drive, docking, and telemetry panels were folded directly into
the Map page (see [Lesson 06](06-the-pages.md#map--mappagejsx)). Several
components below (`Joystick`, `DockingControl`, `NavStatus`, `SystemAlerts`)
are exactly the panels that page used to render; only *which page* renders
them has changed, not what they do or which topics they use. If you're
reading an older doc, an issue, or a code comment that still says "Control
page," this is what it's referring to.

## Quick reference

| Component | Talks to ROS? | Used on |
| --- | --- | --- |
| [`Map`](#map--mapjsx) | Yes (indirectly, via NAV2D) | Map, Route |
| [`MapLayers`](#maplayers--maplayersjsx) | No | Map, Route |
| [`Camera`](#camera--camerajsx) | No (plain HTTP, not rosbridge) | Map, Status |
| [`Joystick`](#joystick--joystickjsx) | Yes | Map |
| [`NavStatus`](#navstatus--navstatusjsx) | Yes | Map |
| [`RobotState`](#robotstate--robotstatejsx) | Yes | Map (compact), Status |
| [`SystemHealth`](#systemhealth--systemhealthjsx) | Yes | Status, Health, Fleet (compact), Config (compact) |
| [`SystemAlerts`](#systemalerts--systemalertsjsx) | Yes | Map |
| [`LifecycleStatus`](#lifecyclestatus--lifecyclestatusjsx) | Yes | Health, Fleet (compact), Config (compact) |
| [`DockingControl`](#dockingcontrol--dockingcontroljsx) | Yes | Map (compact) |
| [`Header`](#header--headerjsx) | Reads status only | Every page (via `AppLayout`) |
| [`Logs`](#logs--logsjsx) | Yes | Every page (floating diagnostics-console drawer — a different thing from the dedicated [Console page](06-the-pages.md#console--consolepagejsx)) |
| [`TimeModal`, `RouteModal`, `TextInputModal`](#the-route-page-modals) | No (delegate to Route page) | Route |
| [`ControlSwitcher`, `CircularProgressBar`, `FilesModal`, `Topics`](#components-not-currently-used) | — | None (not wired into any page) |

## Map — `Map.jsx`

This is the component with the biggest gap between "what it looks like" and
"what it does." `Map.jsx` itself does very little ROS work directly — it
mostly acts as a thin React wrapper around `ros2d.js`/`nav2d.js`, the
imperative browser libraries loaded in
[`web/public/index.html`](../../web/public/index.html) (see
[Lesson 03](03-how-the-browser-talks-to-ros.md)). On mount, it builds a
`ROS2D.Viewer` canvas and hands the shared `ros` connection to a global
`window.NAV2D.InitMap(ros)` call; from that point on, the map grid, laser
scan, costmap, path, and trail rendering are all driven by that library
subscribing to topics on its own, outside React's state — `Map.jsx` doesn't
re-render when new map data arrives. The zoom and pan buttons in the map
corner call `ROS2D.ZoomView`/`canvas.shift()` directly on that same viewer.

The two things `Map.jsx` *does* do through React/ROS directly: it publishes
once, on connect, to `AppConfig.WP_REQ` (`/WP_req`) with an empty message —
a "please send current waypoints" trigger — and it subscribes to
`AppConfig.MAP_TOPIC` (`/ui/map`) purely to bump an internal counter that
forces a re-render tick; NAV2D reads the actual occupancy grid data itself.
It also exposes a `getMapRef()` handle via `ref`, which is how the Route
page attaches its own `mouseup` listener for waypoint placement — see
[`web/src/pages/RoutePage.jsx`](../../web/src/pages/RoutePage.jsx).

## MapLayers — `MapLayers.jsx`

Purely a settings panel — it holds no ROS connection at all and never calls
`useRos()`. Its checkboxes/sliders update local React state for 7 layers
(map, costmap, laser scan, path, goal, waypoints, trail) and call
`window.NAV2D.setLayerVisible()`/`setLayerOpacity()`, the same global object
`Map.jsx` initializes. This is a useful contrast to
[Lesson 06](06-the-pages.md#the-pattern-across-every-page)'s "every page
reads the shared ROS connection" — that's true of *pages*, but individual
panels inside a page are free to not touch ROS at all when there's nothing
to publish or subscribe to.

## Camera — `Camera.jsx`

Also doesn't use rosbridge. As covered in
[Lesson 03](03-how-the-browser-talks-to-ros.md#the-camera-stream-is-a-third-separate-path),
it builds a plain `<img>` pointing at `web_video_server`'s HTTP MJPEG
endpoint, using `AppConfig.CAMERA_PORT` and the selected entry from
`CAMERA_TOPIC_OPTIONS`. The quality selector (low/balanced/high) only changes
the JPEG quality/width/height query parameters sent to `web_video_server` —
it's a client-side stream setting, not anything published to ROS.

## Joystick — `Joystick.jsx`

Publishes to `AppConfig.CMD_VEL_TOPIC` (`/cmd_vel`) only — no subscriptions.
While being dragged, it re-publishes the current stick position on a
100&nbsp;ms interval (not just on movement events), and publishes one final
zero-velocity message on release. The `maxSpeed` prop (passed in by the Map
page's speed slider) scales `linear.x`; `angular.z` is always scaled by the
fixed
`AppConfig.MAX_ANGULAR_SPEED`, which isn't adjustable per instance.

## NavStatus — `NavStatus.jsx`

Subscribes to `AppConfig.NAV_STATUS_TOPIC` (goal status) and
`AppConfig.NAV_FEEDBACK_TOPIC` (distance-remaining feedback) — the action
described in [Lesson 02](02-ros2-core-concepts.md#action). It keeps a small
rolling history (last 5) of terminal status changes, de-duplicated by goal
ID so the same finished goal doesn't get logged twice. On success/cancel/fail
it clears the drawn path (`window.NAV2D.clearPath()`) after 1s and resets the
status badge after 3s. Its `Cancel` button doesn't call the cancel service
itself — it calls an `onCancelGoal` callback passed in as a prop, and the
Map page owns the actual `ROSLIB.Service` call to
`AppConfig.NAV_CANCEL_GOAL_SERVICE`. That split exists so the cancel-service
client is created once per page, not once per panel — the same service is
also what the Map page's top-bar E-STOP button
([`StatusBar.jsx`](06-the-pages.md#the-chrome-thats-on-every-page)) calls
from outside this component entirely. This is a one-shot software intervention,
not a latched or safety-rated physical emergency stop.

## RobotState — `RobotState.jsx`

Renders two stat cards, Velocity and Position, exported as `State` from this
file. Velocity always comes from `AppConfig.ROBOT_POSE_TOPIC` (`/odom`).
Position prefers `AppConfig.AMCL_POSE_TOPIC` (badge shows `AMCL`) but falls
back to odometry's own position field (badge shows `ODOM`) until the first
AMCL message arrives — useful to know because the badge itself is a live
signal of whether localization is publishing at all. It pulls in the `three`
(Three.js) package, but only for its `Euler`/`Quaternion` classes as a
quaternion-to-yaw-angle math utility — there's no 3D rendering happening
here.

## SystemHealth — `SystemHealth.jsx`

The most detailed diagnostic panel. It subscribes to 7 streaming topics
(laser scan, odom, AMCL, Nav2 status, map, global costmap, plan), each with
a `throttle_rate` tuned to how often that topic realistically needs
re-checking (scan every 1s, map/costmap every 3s, everything else every
500ms) — this only throttles the health *bookkeeping*, it doesn't affect
what any other subscriber to those topics receives. Separately, it
subscribes to `/tf` and `/tf_static` and reconstructs which parent→child
frame links have been seen, checking three fixed links every second:
`map->odom`, `odom->base_link`, `base_link->lidar_link`. A topic is marked
"online" only if a message arrived within its configured timeout; it also
tracks a rolling Hz estimate shown in the non-compact view. Rendered
full-detail on Status and Health, compact on Fleet and Config.

## SystemAlerts — `SystemAlerts.jsx`

A lighter, user-facing cousin of `SystemHealth` — it renders nothing at all
when everything is fine, and only appears as a small banner when something
specific is stale: map, AMCL pose, or plan data, plus rosbridge disconnects.
Unlike `SystemHealth`'s broad diagnostics, `plan` going stale here is
`warningOnly` (renders yellow); map/AMCL going stale and rosbridge
disconnecting are always treated as errors (red). Rendered on Map only —
not Route, Status, Health, Fleet, or Config.

## LifecycleStatus — `LifecycleStatus.jsx`

Polls `get_state` every 3 seconds for each node in the `LIFECYCLE_NODES`
list (`map_server`, `amcl`, `controller_server`, `planner_server`,
`bt_navigator`), and separately builds a `change_state` service client per
node for the four action buttons. The buttons call `change_state` on *all
five* nodes at once with a fixed transition ID (`configure=1`, `cleanup=2`,
`activate=3`, `deactivate=4` — the real `lifecycle_msgs` transition IDs) and
don't wait for or react to the individual responses — the displayed state
only updates on the next 3-second poll, so there's a brief delay between
clicking a button and seeing the result.

## DockingControl — `DockingControl.jsx`

Mostly a status display driven by subscriptions, not by its own button
clicks. `Dock`/`Undock` publish a single boolean to
`AppConfig.DOCK_TRIGGER_TOPIC`/`UNDOCK_TRIGGER_TOPIC` and nothing else — the
visible status (idle/docking/docked/undocking/failed) comes entirely from
subscribing to `AppConfig.DOCK_TRIGGER_STATUS_TOPIC`. It also watches
`AppConfig.NAV_STATUS_TOPIC` for one specific case: after undocking finishes,
it publishes a "standby" goal to a fixed pose (`x=0, y=0, yaw=0`, hardcoded
in the component) and watches the nav status to know when that standby
navigation succeeds or fails, before returning to idle. It keeps a rolling
5-entry event log for the non-compact view.

## Header — `Header.jsx`

Now reads its nav entries from
[`web/src/pages/registry.js`](../../web/src/pages/registry.js) — the same
`PAGE_REGISTRY` array covered in [Lesson 06](06-the-pages.md) — instead of a
hardcoded list, plus the rosbridge status dot via `useRosStatus()` and a
light/dark theme toggle backed by `localStorage`. Its **Console** toggle
button doesn't own its own open/closed state — that state lives in
[`web/src/layouts/appLayout.jsx`](../../web/src/layouts/appLayout.jsx), which
passes `showLogs`/`onToggleLogs` down as props, so `Header` stays a pure
display component.

Don't confuse this with the dedicated [Console page](06-the-pages.md#console--consolepagejsx)
in the sidebar — same name, two different things. Header's `Console` button
opens a small floating drawer (`Logs`, below) with a scrollback of UI-level
messages, available no matter which page you're on. The sidebar's Console
page is a full page with a live `/rosout` viewer and a topic-echo panel.

## Logs — `Logs.jsx`

Exported as `RobotLog`, rendered inside `AppLayout`'s floating "System
Diagnostics Console" drawer (opened via Header's `Console` button, above) on
every page. Subscribes to `AppConfig.UI_MESSAGE_TOPIC` and appends each
message to a Redux store (see
[`web/src/stores/index.js`](../../web/src/stores/index.js)) rather than
local component state — the only component covered in this lesson that
persists its data centrally instead of with `useState`, which is why the
drawer keeps its scrollback even if `Logs` itself unmounts and remounts.

## The Route page modals

Three small dialogs, all rendered only by
[`web/src/pages/RoutePage.jsx`](../../web/src/pages/RoutePage.jsx) and none
of which touch ROS directly — each just collects input and calls a
`modalHandler(data)` prop; the Route page is what actually publishes
anything afterward.

- **`TimeModal.jsx`** — collects hours/minutes (validated to 0–23/0–59) when
  placing a waypoint on the map in Create/Edit mode. Worth knowing: the
  handler wired to it in `RoutePage.jsx` is source-commented `// TODELETE`
  and calls `window.NAV2D.sendPointToRobot(ros, data)` directly — it's part
  of the live flow today, but that comment signals the maintainers already
  consider it due for cleanup, so confirm with them before building more on
  top of it.
- **`RouteModal.jsx`** — a single-level list picker used for `Change` (pick
  an existing route to browse).
- **`TextInputModal.jsx`** — a name-input dialog with route-naming rules baked
  in (no underscores, no reserved names like `Null`/`New route`, no
  duplicates), used for `Save` and `Rename`.

## Components not currently used

Four components exist in the codebase but are not imported by any page or
other component today — worth knowing about so you don't assume they're
live, and worth considering before adding a similar new one from scratch:

- **`ControlSwitcher.jsx`** — a generic on/off switch that publishes one of
  two string values to any topic name passed in as a prop. Looks designed to
  be a reusable building block for a future simple toggle-style panel.
- **`CircularProgressBar.jsx`** — a labeled circular gauge that parses a
  `"name_value"`-formatted string and renders it as a percentage of a
  min/max range.
- **`modal/FilesModal.jsx`** — a fuller three-level group → map → route
  browser/picker; a superset of what `RouteModal.jsx` does today.
- **`Topics.jsx`** — a placeholder panel that renders "Coming soon."

## The app shell: routing, providers, and state

Everything above is what renders *inside* a page. This section covers the
layer above that — the parts of the frontend that have nothing to do with
ROS and exist purely to wire the React app together.

**Routing.** [`web/src/pages/index.jsx`](../../web/src/pages/index.jsx)
derives the route table from
[`pages/registry.js`](../../web/src/pages/registry.js) (see
[`docs/extending/add-a-ui-panel.md`](../extending/add-a-ui-panel.md) for
adding to it), all nested under
[`web/src/layouts/appLayout.jsx`](../../web/src/layouts/appLayout.jsx), which
renders `Header` plus whichever page is active, and owns the floating
console drawer's open/closed state (passed down to `Header` as props — see
[Header](#header--headerjsx) above).

**Always-mounted alongside the active page.** `AppLayout` also renders a
handful of components that have nothing to do with whichever page is
currently showing, and stay mounted across every navigation instead of
belonging to one page: `StatusBar` (battery + E-STOP, see
[Lesson 06](06-the-pages.md#the-chrome-thats-on-every-page)),
`NotificationsWatcher` and `EventRecorder` (feed the browser-notification
and [Events page](06-the-pages.md#events--eventspagejsx) logs respectively,
regardless of which page triggered the event), `SchedulerRunner` and
`MissionRunner` (the headless executors behind the
[Scheduler](06-the-pages.md#scheduler--schedulerpagejsx) and
[Missions](06-the-pages.md#missions--missionspagejsx) pages — they keep
firing even if you navigate away from the page that created the
schedule/mission), the `DemoModeBanner`/`ReplayModeBanner`/`AuthModeBanner`
trio, and `HelpWidget`/`OnboardingWizard`. None of these hold ROS
subscriptions of their own beyond what's needed for their one job — they're
listed here because "why does this keep running when I'm on a different
page" is exactly the kind of question this lesson exists to answer.

**Providers.** [`web/src/app/App.jsx`](../../web/src/app/App.jsx) is
exported wrapped in `withProviders`, defined in
[`web/src/app/providers/index.js`](../../web/src/app/providers/index.js) as
`compose(withBoundary, withRouter, withStore)`. Composed this way, the
render tree nests outside-in as: error boundary → router → Redux store
provider → `App` itself. Error boundary outermost is deliberate — it's the
only thing positioned to catch a crash anywhere else in the tree, including
inside routing.

**State management: two different mechanisms, used for different things.**
Most panels covered in this lesson hold their own state with plain
`useState`/`useRef` — that's the default, and it's what you should reach for
too (see [`add-a-ui-panel.md`](../extending/add-a-ui-panel.md)). The one
exception is [`Logs.jsx`](#logs--logsjsx), which dispatches into a small
Redux store ([`web/src/stores/index.js`](../../web/src/stores/index.js),
wired up in
[`web/src/app/store/index.js`](../../web/src/app/store/index.js)) holding
exactly one slice of state: the log message list. Redux exists in this
codebase for that one reason — centralized state that needs to survive a
panel unmounting and remounting — not as a general pattern every new panel
should follow.

**Error boundary.**
[`web/src/app/ErrorBoundary/ErrorBoundary.jsx`](../../web/src/app/ErrorBoundary/ErrorBoundary.jsx)
is the fallback UI shown if any component in the tree throws during render —
a plain "Something went wrong" message with a link back to `/`. It only
catches render-time errors (React's `componentDidCatch`), not errors inside
event handlers or `ROSLIB` callbacks — a `.subscribe()` callback that throws
won't trigger it.

**Theme.** The light/dark toggle seen in `Header` is backed by
`ThemeContext`, defined alongside `RosContext` in `App.jsx` and persisted to
`localStorage` — the same React context mechanism from
[Lesson 03](03-how-the-browser-talks-to-ros.md), just carrying UI
preference instead of a ROS connection.

## Try it

Choose one Map panel and trace its page import, ROS topic constant,
subscription or publisher, cleanup function, and rendered status.

**You're ready to continue when:** you can add a read-only panel without
creating a second ROS connection or leaking a subscription.

## Next

[Lesson 08 — The Map and Route File Model](08-map-and-route-model.md) goes
one level deeper on the Route page specifically — the group/map/route file
hierarchy behind everything `RoutePage.jsx` and `folders_handler.py` do —
before [Lesson 09 — Blockly Visual Programming](09-blockly-programming.md)
covers the fifth page, Blocks, the same way this lesson covered the other
four.

---

[← Lesson 06](06-the-pages.md) · [Lesson index](README.md) ·
[Next: Lesson 08 →](08-map-and-route-model.md)
