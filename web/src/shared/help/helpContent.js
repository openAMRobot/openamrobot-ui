/**
 * Per-page "What is this?" help content, keyed by the same absolute path
 * used in web/src/pages/registry.js. Read by HelpWidget.jsx. Pure data —
 * a contributor adding a page via registerPage() can add an entry here
 * too, but the app works fine (falls back to a generic message) if they
 * don't.
 */
export const PAGE_HELP = {
  "/": {
    title: "Map",
    summary:
      "The main operational view: the live occupancy map, the robot's position, and the controls to send it somewhere.",
    tips: [
      "Drag to pan, scroll or pinch to zoom, or use the +/− buttons.",
      "Layer toggles above the map control what's drawn — costmap, laser scan, planned path, saved waypoints, the robot's trail.",
      "Goal Mode / Set Pose / Add Waypoint change what a click on the map does.",
      "Right-click the map for a quick menu — send a goal, save a waypoint, or set the initial pose — without switching modes first.",
      "The joystick in the corner drives the robot manually at any time.",
    ],
  },
  "/route": {
    title: "Routes",
    summary:
      "Author reusable waypoint sequences for the currently active map, separate from one-off goals sent from the Map page.",
    tips: [
      "Pick a group and map at the top, then Create or Edit a route.",
      "While editing, click the map to add points — Save when you're happy with the sequence.",
      "Change map switches which map this route belongs to; Auto-plan asks Nav2 to compute a path between two points for you.",
    ],
  },
  "/control": {
    title: "Control",
    summary:
      "Manual driving plus a compact live-telemetry view — velocity, position, system health, and Nav2 lifecycle state, all in one place.",
    tips: [
      "The joystick drives the robot directly; the max-speed slider caps how fast.",
      "Health shows whether topics like TF, odometry, and the laser scan are actually publishing right now.",
      "Lifecycle shows Nav2's own node states (active/inactive/unconfigured) — useful when navigation isn't behaving as expected.",
      "Dock/Undock trigger the robot's charging-dock behaviors, when supported.",
    ],
  },
  "/blocks": {
    title: "Programs",
    summary:
      "A visual, block-based way to script a sequence of robot actions — navigate, wait, change speed, repeat — without writing code.",
    tips: [
      "Drag blocks from the palette and snap them together to build a program.",
      "Run executes the program step by step against the real robot; run history keeps a record of past runs.",
      "Named locations (set on the Config-adjacent locations list) let you say \"go to Charging Station\" instead of typing coordinates.",
    ],
  },
  "/info": {
    title: "Status",
    summary:
      "A live operational readout: camera feed, pose/velocity telemetry, battery, and system health — the page to glance at while the robot is doing something.",
    tips: [
      "The camera panel is paused by default to save bandwidth — press Start when you need to actually see through it.",
      "The battery trend sparkline shows the last several readings, not just the instantaneous value.",
    ],
  },
  "/robot": {
    title: "Robot",
    summary:
      "A 3D model of the robot from its real URDF description — Description Mode is a safe, offline visualization; Live Mode overlays real robot data.",
    tips: [
      "Description Mode needs no ROS connection — joint sliders here only move the on-screen model, nothing is published anywhere.",
      "Live Mode positions the model at its real map-frame pose and draws Nav2's planned path — but joint sliders become read-only telemetry, since this robot only has /cmd_vel (no per-joint position command).",
      "The kinematic tree and layer toggles (TF axes, joint axes, link names, centre-of-mass, footprint) work in both modes.",
    ],
  },
  "/devices": {
    title: "Devices",
    summary:
      "A manual registry of external hardware — USB, CAN, network, and Raspberry-Pi-attached devices — with live status wherever a ROS topic is available.",
    tips: [
      "There's no plug-and-play auto-detection here (that needs OS-level access this app doesn't have) — register what's connected yourself.",
      "The detected-serial-ports list is real, though: it reads actual USB-serial devices present on the machine running the backend.",
      "A device only shows Online if its status topic is actively publishing — a device is never assumed connected just because you registered it.",
    ],
  },
  "/health": {
    title: "Health Centre",
    summary:
      "One place to answer \"is the whole robot ready?\" — an overall rollup built from every other page's live signals.",
    tips: [
      "Click any listed issue to jump straight to the page where it can actually be fixed.",
      "Ready with warnings means nothing is broken, but something (low battery, a missing topic, an offline device) is worth a look.",
      "Recent faults is a running log for this browser session only — it resets on reload, it isn't a persisted history.",
    ],
  },
  "/recordings": {
    title: "Recordings",
    summary:
      "Record real rosbag sessions and replay them later — useful for debugging, demos, lessons, and dataset collection.",
    tips: [
      "Replayed telemetry is always clearly labeled (a Replay mode banner appears on every page) — it's never shown as if it were a live robot.",
      "Stopping a replay can take several seconds — ros2 bag play needs a moment to shut down cleanly, that's expected, not a stuck button.",
      "Recording all topics is the safest default if you're not sure what you'll need later.",
    ],
  },
  "/config": {
    title: "Config",
    summary:
      "Connection settings, saved robot profiles, safety limits, and notification preferences for this browser — nothing here is shared with other operators.",
    tips: [
      "Demo mode lets you explore the whole app with simulated data and no robot at all — flip it on here any time.",
      "Save a named connection profile once you've set a host/port, so switching robots later is a single click.",
      "Changing the rosbridge host or port reconnects immediately.",
    ],
  },
};

export const DEFAULT_HELP = {
  title: "OpenAMRobot",
  summary: "No page-specific help is available here yet.",
  tips: [],
};
