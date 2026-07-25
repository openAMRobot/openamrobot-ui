# Lesson 06 — A Tour of Every Page

| Audience | Time | Prerequisites |
| --- | --- | --- |
| Operators and UI contributors | 25 minutes | [Lesson 00](00-your-first-10-minutes.md) and [Lesson 01](01-what-is-this-ui.md) |

## What you'll learn

You will learn which page to use for a task, which controls can move the
robot, and which pages depend on optional backend helpers or browser-side
state.

This UI started with a handful of pages. It has grown a lot since — the
sidebar now lists 18 of them. That sounds like a lot to learn, but almost
every page follows the same shape described in
[Lesson 03](03-how-the-browser-talks-to-ros.md): it reads the one shared ROS
connection, subscribes or publishes to a few named topics, and renders what
it hears. Once you've seen a few pages, the rest read the same way.

This lesson walks each page at a glance — what it's for, what you can
actually click, and which topics it depends on — with a real screenshot from
the running UI (Demo Mode on, so the numbers are simulated but the layout is
exactly what you'll see). For a deeper look at the individual panels behind
Map/Route, see [Lesson 07](07-ui-components.md); for the group → map → route
file model, see [Lesson 08](08-map-and-route-model.md); for the Programs
page specifically, see
[Lesson 09 — Blockly Visual Programming](09-blockly-programming.md).

Every page is a route registered in one place,
[`web/src/pages/registry.js`](../../web/src/pages/registry.js) — that file
is also what draws the sidebar, so adding a page there is the only step
needed to make it show up everywhere. See
[`docs/extending/add-a-ui-panel.md`](../extending/add-a-ui-panel.md) if
you're the one adding the next one.

## The chrome that's on every page

Three things are visible no matter which page you're on, all rendered by
[`web/src/layouts/appLayout.jsx`](../../web/src/layouts/appLayout.jsx) above
the page itself, so every screenshot below shows them:

- The top bar (`Header.jsx`) — the sidebar/nav, a connection dot
  ("Connected"/"Disconnected"), and the light/dark theme toggle.
- The status strip right under it (`StatusBar.jsx`) — live battery
  percentage and a big red **E-STOP** button that zeroes `/cmd_vel` and
  cancels the active Nav2 goal from anywhere, not just the Map page. This is a
  non-latched software stop and does not replace a physical emergency stop.
- Whichever banner applies: a purple **Demo Mode** banner when
  [Demo Mode](#config--configpagejsx) is on, a replay banner while a
  [recording](#recordings--recordingspagejsx) is playing back, or an
  auth-mode banner (see the note in
  [Lesson 01](01-what-is-this-ui.md#a-safety-note-there-is-no-authentication)).

## Quick map

| Group | Pages |
| --- | --- |
| Operate the robot | [Map](#map--mappagejsx), [Routes](#routes--routepagejsx), [Maps](#maps--mapspagejsx), [Programs](#programs--blockspagejsx), [Scheduler](#scheduler--schedulerpagejsx), [Missions](#missions--missionspagejsx) |
| Keep an eye on it | [Status](#status--infopagejsx), [Robot](#robot--robotdescriptionpagejsx), [Devices](#devices--devicespagejsx), [Health](#health--healthpagejsx), [Metrics](#metrics--metricspagejsx), [Recordings](#recordings--recordingspagejsx), [Events](#events--eventspagejsx) |
| Under the hood | [Console](#console--consolepagejsx), [Parameters](#parameters--paramspagejsx), [Fleet](#fleet--fleetpagejsx), [Config](#config--configpagejsx) |
| Extending it | [Notes](#notes--example-plugin) |

## I want to...

| Task | Start here | Check before acting |
| --- | --- | --- |
| Drive manually or send one goal | [Map](#map--mappagejsx) | Physical E-stop, clear area, fresh pose and map |
| Build or run a reusable route | [Routes](#routes--routepagejsx) | Correct group/map/route and optional helper launch |
| Change or create a map | [Maps](#maps--mapspagejsx) | Current localization state and map-server ownership |
| Automate a sequence | [Programs](#programs--blockspagejsx), [Scheduler](#scheduler--schedulerpagejsx), or [Missions](#missions--missionspagejsx) | Tab remains open, connection is fresh, motion area is clear |
| Understand a warning | [Health](#health--healthpagejsx), [Events](#events--eventspagejsx), then [Console](#console--consolepagejsx) | Connection state and topic freshness are separate |
| Change robot parameters | [Parameters](#parameters--paramspagejsx) | Correct robot and node; record the old value first |
| Switch robots | [Fleet](#fleet--fleetpagejsx) | Which physical robot this browser will command |
| Practice without hardware | [Config](#config--configpagejsx) → Demo Mode | Purple Demo Mode banner is visible |

## Map — `MapPage.jsx`

The main operating page — situational awareness and manual driving in one
place. (There used to be a separate Control page; its driving, docking, and
telemetry panels now live here — see the note in
[Lesson 07](07-ui-components.md#a-note-on-the-old-control-page).)

![OpenAMR UI Map page: occupancy grid with robot pose, camera panel, manual drive joystick, speed presets, docking control, saved waypoints, and mode buttons](<../assets/map/map.png>)

- The map panel shows the occupancy grid, robot pose, and optional overlays
  (costmaps, laser, path, goal, waypoints, keep-out zones, trail) via the
  `LAYERS` row, each with its own opacity slider.
- **Send Goal** (labeled "Goal Mode" in earlier versions of this UI)
  publishes a navigation goal — click to send the robot there, drag before
  releasing to set a heading. **Correct Robot's Position** (labeled "Set
  Pose" in earlier versions) publishes a one-shot `/initialpose` correction
  (use this after localization drifts or resets) — it's deliberately named
  and styled to read as a different kind of action from the two navigation
  modes next to it, since it corrects the robot's own self-localization
  rather than sending it anywhere. **Add Waypoint** queues up several goals
  to run in order via **Execute Queue**. **Go Home** sends the robot to
  `(0, 0)`.
- Right-clicking the map is a shortcut for the same three actions (send a
  goal, save a waypoint, correct the robot's position) without switching
  modes first.
- The controls row combines a manual joystick with its own **STOP** button, a
  max-speed slider with quick presets, live velocity/position (`RobotState`,
  compact), dock/undock controls and status (`DockingControl`, compact), and
  your saved waypoint library (go to one, or save the robot's current spot).
- `SystemAlerts` near the top only appears when something specific is stale
  (map, localization, plan, or the connection itself) — silence there is a
  good sign, not a missing feature.

## Routes — `RoutePage.jsx`

The route-authoring page: build and manage named, reusable waypoint
sequences for a given map, as opposed to Map's one-off goals.

![OpenAMR UI Routes page: route authoring header with group/map/route selectors, layer toggles, and route operations panel](<../assets/Routes/routes.png>)

- The `GROUP` / `MAP` / `CURRENT ROUTE` header shows exactly which route
  you're editing — always check it before saving, since a route only makes
  sense for the map it was drawn on (see
  [Lesson 08](08-map-and-route-model.md)).
- **Create** starts a new route; click the map to drop waypoints. **Edit**
  reopens the current route for changes. **Switch route** (labeled "Change"
  in earlier versions) picks a different saved route. **Switch map**
  (labeled "Change map") switches which map you're routing on — the two
  are now named distinctly on purpose, since switching the map is a much
  bigger change than switching the route. **Auto-plan** asks Nav2 to compute
  a path between two points and turns it into waypoints automatically.
  **Save**, **Rename**, and **Delete** act on the current route; **Clear
  waypoints** (labeled "Clear points") wipes the in-progress editor only.
- Exchanges file state with the `folders_handler` backend node over
  `/nav_data_req`, `/nav_data_resp`, and `/ui_operation` — this only works
  if the optional
  [`physnode_launch.py`](../../ros2/src/openamr_ui_package/launch/physnode_launch.py)
  helper is running (see [Lesson 05](05-backend-nodes-in-detail.md)).

## Maps — `MapsPage.jsx`

Map management: build a brand-new map from scratch, save the one currently
loaded, and organize saved maps into groups.

![OpenAMR UI Maps page: build-a-new-map and save-current-map panels, plus a saved-maps list grouped by folder](<../assets/Maps/Maps.png>)

- **Start mapping** launches mapping mode (SLAM) and stops
  navigation/localization — drive the robot around the space (with the
  joystick on the Map page, for example), then come back here and use
  **Save current map** once you're done.
- The saved-maps list lets you **Switch** the active map (reloads it on the
  robot immediately — set the initial pose again afterward, since old
  localization won't match a new map), **Rename**, or **Delete** it, and add
  or delete groups.
- This is the same `folders_handler` node and `/ui_operation` protocol the
  Routes page uses (see [Lesson 05](05-backend-nodes-in-detail.md) and
  [Lesson 08](08-map-and-route-model.md)) — this page is what finally
  exposes that node's mapping functions through an actual button, rather
  than requiring a raw topic publish.

## Programs — `BlocksPage.jsx`

The visual-programming page (the sidebar calls it "Programs"; the file,
code, and the rest of this lesson series still call it "Blocks" or
"Blockly" — same page). Build a robot program by dragging blocks instead of
writing code, then press `Run`.

![OpenAMR UI Programs page: Blockly workspace with a start-robot-program block chain, plus the ROSBridge status, Run/Stop, Voice Command, Program Templates, and Run History panels](<../assets/Programms/Blockly.png>)

- The left toolbox groups blocks into `Program`, `Navigation`, `Motion`,
  `Docking`, and `Robot State`. Only blocks connected below `start robot
  program` run.
- The right panel covers connection status ("Robot connected"/"Robot
  offline"), `Run`/`Stop`, **Voice Command**
  (speak a command after the wake word "Monsieur"), **Program Templates**
  (ready-made starter programs), **Run History**, **Backend Programs**
  (save/load workspaces on the server, not just in this browser), **Named
  Locations**, **Plan Checks** (safety warnings before running), and the
  **Generated Plan** (the exact steps that will execute).
- This page is different enough — a full visual-programming pipeline, not
  just a set of panels — that it gets its own full lesson:
  [Lesson 09 — Blockly Visual Programming](09-blockly-programming.md), plus
  the practical
  [`web/src/features/blocks/README.md`](../../web/src/features/blocks/README.md)
  guide.

## Scheduler — `SchedulerPage.jsx`

Trigger navigation at set times of day.

![OpenAMR UI Scheduler page: add-schedule form with name, time, repeat, and target, and a list of configured schedules](<../assets/Scheduler/Scheduler.png>)

- A schedule has a name, a time of day, a repeat (`Daily` or `Once`), and a
  target — go home, navigate to a saved waypoint, or run a whole
  [mission](#missions--missionspagejsx).
- Enable/disable a schedule without deleting it, or delete it outright.
- Honest caveat right in the page description: this fires from
  [`SchedulerRunner`](../../web/src/components/SchedulerRunner.jsx), a
  component mounted in `AppLayout` that only runs while a browser tab with
  this UI open is actually open. It is a convenience scheduler, not
  robot-side cron — closing every tab stops it from firing.

## Missions — `MissionsPage.jsx`

Chain several actions into one ordered sequence — go here, wait, dock — and
run the whole thing as a "mission."

![OpenAMR UI Missions page: mission list with step count and run/stop controls, and a new-mission form](<../assets/missions/missions.png>)

- A mission is an ordered list of steps: go to a saved waypoint, go home,
  wait N seconds, dock, or undock. Reorder steps with the up/down arrows,
  or remove one.
- **Run** executes the mission live, one step at a time, via the headless
  [`MissionRunner`](../../web/src/components/MissionRunner.jsx) component
  (mounted in `AppLayout`, same "needs an open tab" caveat as the
  Scheduler). Each step gets a ✓/✗ next to it as it completes.
- A mission is itself a valid Scheduler target — build it here, then trigger
  it on a timer from the [Scheduler page](#scheduler--schedulerpagejsx).

## Status — `InfoPage.jsx`

The calm diagnostics page (the sidebar calls it "Status"; the file is
`InfoPage.jsx`) — camera, battery, charging, and system health, with no
drive controls to accidentally bump.

![OpenAMR UI Status page: camera feed, battery level with recent-trend sparkline, charging station status, velocity/position, and full system health panel](<../assets/Status/status.png>)

- Battery shows a live percentage plus a small rolling **trend** sparkline
  of the last 40 readings — useful for spotting "draining faster than
  usual" at a glance, not just the instantaneous number.
- Charging station status is a simple connected/not-connected read from
  `/charge_station_connected`.
- The full-detail `SystemHealth` panel here is the same component shown
  compact elsewhere (Fleet, Config) — see
  [Lesson 07](07-ui-components.md#systemhealth--systemhealthjsx) for exactly
  what it checks.
- No battery data just means no node is publishing `/battery_status` (the
  standard launch doesn't start `battery.py` by default) — the page keeps
  working fine either way, it just shows "No battery telemetry."

## Robot — `RobotDescriptionPage.jsx`

A 3D digital twin of the robot, built directly from its real URDF/Xacro
description — no physical robot required to look at it.

![OpenAMR UI Robot page: 3D model viewer, kinematic tree, link information panel, and joint controls](<../assets/robot%20description/image.png>)

- **Description Mode** vs **Live Mode** (the switch near the top): Description
  Mode loads the model locally and needs no ROS connection at all — joint
  sliders just pose the 3D viewer. Live Mode instead subscribes to real
  `/joint_states` and pose topics, so the model reflects the actual robot —
  its joint sliders become read-only, since this robot only has one coupled
  `/cmd_vel` for both drive wheels, not a per-joint position command a
  slider could safely drive.
- The **Kinematic Tree** shows every link and joint, `FIXED` or movable, in
  parent → child order; clicking one selects it in both the tree and the 3D
  view.

  ![OpenAMR UI Robot page kinematic tree panel, expanded showing links and joints](<../assets/robot%20description/kinematictree.png>)

- Selecting a link or joint fills in **Link/Joint Information** — parent,
  children, geometry, mass, inertia, or (for a joint) type, axis, limits, and
  origin — read directly from the parsed URDF.

  ![OpenAMR UI Robot page joint information panel showing a fixed joint's parent link, child link, and origin](<../assets/robot%20description/jointinformation.png>)

- **Joint Controls** only ever shows sliders for the robot's actual movable
  joints — on this robot, that's exactly two continuous wheel joints, since
  every other joint (casters, lidar, camera) is fixed and correctly gets no
  slider.

  ![OpenAMR UI Robot page joint controls panel with left and right wheel joint sliders](<../assets/robot%20description/jointcontrols.png>)

- **Display Layers** toggles what's drawn on the model: visual mesh,
  collision geometry, TF/frame axes, joint axes, link names, joint names,
  center-of-mass markers, and a computed footprint outline.

  ![OpenAMR UI Robot page display layers panel with all eight layer toggles enabled](<../assets/robot%20description/displaylayers.png>)

  ![OpenAMR UI Robot page 3D viewer with joint axes, joint names, and TF axes layers turned on](<../assets/robot%20description/description.png>)

## Devices — `DevicesPage.jsx`

A manual registry for external hardware — USB, CAN, network, or
Raspberry-Pi-attached devices — with a live status badge wherever a ROS
topic is available.

![OpenAMR UI Devices page: detected serial ports, a register-a-device form, and the registered devices list](<../assets/devices/devices.png>)

- **Detected serial ports** lists real USB-serial devices currently plugged
  into the machine running the Flask backend — click one to prefill the
  registration form. This only sees serial ports on that one host; it won't
  see CAN interfaces, network devices, or hardware on a different Pi.
- Registering a device just needs a name and a connection target (serial
  path, CAN interface, or host:port). The status topic is optional — leave
  it blank for a device you're just keeping a record of, or point it at
  whatever topic that device's driver publishes to get a live
  online/offline badge.
- There is deliberately no plug-and-play auto-detection beyond serial ports —
  you register what's connected.

## Health — `HealthPage.jsx`

"Health Centre" — one place to answer "is the whole robot actually ready?",
aggregating signals that are otherwise scattered across several pages.

![OpenAMR UI Health Centre: overall status banner, system health, lifecycle, devices, battery, robot description, diagnostics, and recent faults](<../assets/health/health.png>)

- The banner at the top rolls everything up into one label — Ready, Ready
  with warnings, Needs attention, or Not ready — with a short reason
  underneath when it isn't simply "Ready."
- Below that: `SystemHealth` (topics/TF), `LifecycleStatus` (Nav2 lifecycle
  nodes), a Devices summary (with a link to [Devices](#devices--devicespagejsx)
  to fix anything offline), Battery, whether the robot's URDF is available
  (feeds [Robot](#robot--robotdescriptionpagejsx)), raw `/diagnostics`
  messages, and whether all expected topics are present.
- A session-scoped **Recent faults** log records exactly when something went
  from fine to not-fine — a broken TF chain, an unknown lifecycle node, a
  topic that's gone silent — color-coded by severity, so you can tell "this
  happened once and recovered" from "this is still broken" at a glance.

  ![OpenAMR UI Health Centre Recent Faults panel showing one warning and five errors, each timestamped](<../assets/health/recentfaults.png>)

- **Export support package** bundles all of the above — connection info,
  health rollup, recent events, a metrics snapshot, runtime config, and a
  best-effort Nav2 parameter snapshot — into one file, handy for sharing
  with someone debugging remotely.

## Metrics — `MetricsPage.jsx`

The robot's track record: distance, uptime, and how often goals and docking
actually succeed — all derived client-side from telemetry the stack already
publishes, no extra instrumentation needed.

![OpenAMR UI Metrics page: distance travelled, current speed, session uptime, and goals-run cards, plus navigation goals and docking success/fail breakdowns](<../assets/metrics/metrics.png>)

- Counters (distance, goal/dock outcomes, peak speed) accumulate across
  page reloads — they're kept in the browser, not reset just because you
  refreshed. **Reset counters** zeroes them explicitly, with a confirmation
  first since it can't be undone.
- Distance and speed integrate the robot's own odometry; goal/dock outcomes
  come from watching the same navigation-status and docking-status topics
  covered in [Lesson 07](07-ui-components.md).

## Recordings — `RecordingsPage.jsx`

Record real `ros2 bag` sessions and replay them later — useful for
debugging, demos, and dataset collection.

![OpenAMR UI Recordings page: start-a-recording form with topic selection, and the saved recordings list](<../assets/recordings/recordings.png>)

- Record **all topics** or hand-pick from a checklist (scan, odometry, map,
  AMCL pose, nav status, battery, joint states, TF). Give it a name and an
  optional description.
- Saved recordings can be **Replayed** (at an adjustable rate, with
  pause/resume), **Downloaded**, or **Deleted**. Stopping a replay takes a
  few seconds in the real world — `ros2 bag play` needs that long to shut
  down cleanly after a stop request — and the button says so while it
  waits, rather than looking stuck.
- Replayed telemetry is always clearly labeled as a replay, never presented
  as if it were a live robot.

## Events — `EventsPage.jsx`

A reviewable, persisted timeline of what happened — navigation outcomes,
docking, low battery, emergency stops — for looking back after the fact.

![OpenAMR UI Events page: type and severity filter pills, and a timestamped list of recorded events](<../assets/events/evnets.png>)

- Recorded automatically by
  [`EventRecorder`](../../web/src/components/EventRecorder.jsx) (mounted in
  `AppLayout`, so it's always watching, not just while this page is open)
  into the browser's local storage — it survives reloads.
- Filter by type (navigation, docking, battery, safety, system) and severity
  (info, success, warning, error). **Export** downloads the full log as
  JSON; **Clear** wipes it.

## Console — `ConsolePage.jsx`

A live `/rosout` log console plus an "echo any topic" panel — debug the
running stack from the browser instead of needing a sourced terminal
alongside it.

![OpenAMR UI Console page: live rosout log on the left, topic echo on the right showing raw /scan messages](<../assets/console/console.png>)

- `ROSOUT` streams ROS log messages with a level filter, node filter, and
  text search, and a `FOLLOW`/`Pause` toggle for when you want to freeze the
  view to read something.
- `TOPIC ECHO` subscribes to any topic name you type and shows raw messages
  as they arrive — the browser equivalent of `ros2 topic echo`, covered
  alongside the rest of the CLI toolkit in
  [Lesson 12](12-debugging-with-ros-cli.md).

## Parameters — `ParamsPage.jsx`

Live Nav2 parameter tuning without a terminal.

![OpenAMR UI Parameters page: a table of node/parameter/type/value rows with Read and Set buttons](<../assets/parameters/parameters.png>)

- Each row is one parameter on one node — add a row, type the node name
  (e.g. `/controller_server`) and parameter name (e.g.
  `FollowPath.max_vel_x`), pick its type, then **Read** its current value or
  **Set** a new one.
- This calls the same standard `rcl_interfaces` `get_parameters`/
  `set_parameters` services any ROS 2 node exposes ([Lesson 02](02-ros2-core-concepts.md#service))
  — nothing OpenAMR-specific, just a browser front end for it.
- Changes are **runtime-only**: they revert the moment the target node
  restarts, exactly like running `ros2 param set` by hand would.

## Fleet — `FleetPage.jsx`

Manage more than one robot from this UI, and switch which one it's actually
talking to.

![OpenAMR UI Fleet page: active robot health rollup, and a roster of robots with reachability dots](<../assets/fleet/fleet.png>)

- This app holds exactly one live rosbridge connection at a time — the
  robot marked **active**. The roster's other entries only get a
  lightweight reachability ping (can a WebSocket even open to that
  host:port?), not a full health rollup, until you actually connect to
  them.
- **Connect** repoints the app's runtime config at that robot's host/port —
  the same mechanism as the [Config page's](#config--configpagejsx)
  connection fields, just one click instead of retyping them.
- **Active robot health** mirrors the [Health Centre](#health--healthpagejsx)
  rollup for whichever robot is currently connected, so you don't have to
  leave this page to notice a problem.

## Config — `ConfigPage.jsx`

Connection and safety defaults for this browser — saved locally, never
shared with other operators or persisted on the robot itself.

![OpenAMR UI Config page: Demo mode toggle](<../assets/configuration/demomode.png>)

- **Demo mode** — explore the whole interface with simulated telemetry, no
  robot or robot connection required. Every page shows a permanent badge
  while it's on, and nothing simulated is ever presented as live. (Every
  screenshot in this lesson was taken with Demo Mode on.)

  ![OpenAMR UI Config page: connection host/port fields resolving to a ws:// address](<../assets/configuration/connection.png>)

- **Connection** — the "Robot address override"/"Robot connection port"
  fields (backed by rosbridge host/port under the hood) and the camera
  stream port. Leave the address blank to auto-use whichever address the
  page itself was loaded from (the normal case once this is deployed on the
  robot). A **Connection diagnostics** card right below mirrors the Health
  Centre rollup, so a bad setting shows its effect right where you'd go to
  fix it — including its "Ready with warnings" state, not just the nominal
  one.

  ![OpenAMR UI Config page: Connection diagnostics card showing a "Ready with warnings" state with a stale costmap topic flagged](<../assets/configuration/connection%20diagnostics.png>)

- **Saved robots** — name the current connection and save it as a profile,
  then switch with one click (the same profiles the Fleet page's roster
  builds on).

  ![OpenAMR UI Config page: saved robots profile list, empty](<../assets/configuration/savedrobot.png>)

- **Manual-drive safety limits** — ceiling values for the joystick and the
  Map page's max-speed slider; lowering either takes effect immediately for
  new drive commands.

  ![OpenAMR UI Config page: manual-drive safety limits showing max linear speed 0.2 m/s and max angular speed 2 rad/s](<../assets/configuration/manualrivesafetylimit.png>)

- **Notifications** — browser notifications for nav completion, docking, and
  low battery (needs the browser's notification permission granted, shown
  here already granted), plus the percentage threshold that counts as "low."

  ![OpenAMR UI Config page: notifications panel, disabled, with permission granted and a 20% low-battery threshold](<../assets/configuration/notifications.png>)

- **Keep-out zones** — rectangular no-go areas drawn in map coordinates;
  visible on the Map page's `Zones` layer, but this is a visual aid only —
  real enforcement needs a Nav2 `keepout_filter` configured on the robot
  side.

  ![OpenAMR UI Config page: keep-out zones editor with name/position/size fields](<../assets/configuration/keepoutzones.png>)

## Notes — example plugin

Not in the static page list at all — `/notes` is added by a real, working
example plugin at
[`web/src/plugins/notesPlugin/`](../../web/src/plugins/notesPlugin/), wired
up with a single `installNotesPlugin()` call in
[`web/src/index.js`](../../web/src/index.js). It's a small localStorage
scratchpad, deliberately simple, whose only job is to prove the plugin
registry works end to end without touching `Header.jsx` or
`pages/index.jsx` directly.

![OpenAMR UI Notes page: a note list on the left and an empty note editor on the right](<../assets/notes/notes.png>)

If you're building your own page as a plugin instead of editing the core
registry, this folder is the template to copy — see
[Lesson 13 — Extending the System](13-extending-the-system.md) for where
that fits into the bigger picture.

## The pattern across every page

Most pages above follow the same shape: read the shared ROS connection (see
[Lesson 10](10-topics-as-the-contract.md)), create `ROSLIB.Topic`/`Service`
instances bound to names pulled from the shared constants file, subscribe or
publish, and clean up on unmount. None of them open their own connection —
they all share the one described in
[Lesson 03](03-how-the-browser-talks-to-ros.md). This is exactly the shape
[`docs/extending/add-a-ui-panel.md`](../extending/add-a-ui-panel.md) asks
you to follow for a new page or panel.

A few pages reach that same shared connection through one extra layer of
indirection instead of wiring topics directly into the page component,
because what needs to run isn't known until later: Programs goes through
`robotActions.js` (the workspace isn't read until you press `Run` — see
[Lesson 09](09-blockly-programming.md)), and Scheduler/Missions go through
their own headless runner components (`SchedulerRunner`/`MissionRunner`,
mounted in `AppLayout` so they keep working even if you navigate away from
the page that created them). The underlying rule is identical either way:
one shared connection, no page or runner creates its own.

## Try it

In Demo Mode, use the task table to visit Map, Health, Config, and one
automation page. On each page, identify whether its data comes from ROS, the
backend REST API, or browser-local storage.

**You're ready to continue when:** you can choose the correct page for a
single goal, reusable route, map change, health warning, and automated
mission—and identify which of those can move real hardware.

## Next

[Lesson 07 — UI Components in Detail](07-ui-components.md) drills into the
individual panels behind Map and Route specifically — what each one
renders, exactly which topics/services it uses, and how its internal state
works.

---

[← Lesson 05](05-backend-nodes-in-detail.md) · [Lesson index](README.md) ·
[Next: Lesson 07 →](07-ui-components.md)
