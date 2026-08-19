# Web Frontend

This folder contains the React browser dashboard for OpenAMR robot control,
status, maps, routes, docking, and camera views.

The top-level `../README.md` is the source of truth for full workspace setup,
ROS launch commands, ports, and troubleshooting.

## Development

Install dependencies and run the React development server:

```bash
npm install
npm run dev
```

The dev server runs at:

```text
http://localhost:3000
```

The frontend can render without ROS, but live robot data and controls require
rosbridge to be running and reachable. If the robot isn't at the default
address, the easiest fix is the in-app Config page (`/config`) — it overrides
the rosbridge host/port at runtime, no rebuild needed. `ROSBRIDGE_SERVER_IP`
in `src/shared/constants/index.js` is only the fallback default the dev
server (`:3000`) uses when no runtime override is set — see
`src/shared/constants/runtimeConfig.js`'s `resolveRosbridgeHost()`.

Also worth knowing: Demo Mode (toggle on the Config page) renders the entire
UI with simulated telemetry and no rosbridge connection at all — useful for
frontend-only work when a robot or simulator isn't available.

## Production build

From the repository root, use the canonical scripts:

```bash
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
```

`build_frontend.sh` creates `web/build/`. `sync_frontend_to_ros.sh` copies that
build into the ROS package static app directory so Flask can serve it.

## Structure

- `public/ros/`: browser ROS libraries copied into the production build
  (`roslib.js`, `ros2d.js`, `nav2d.js`, `easeljs.js`, `eventemitter2.min.js`).
- `src/app/`: top-level React app setup, providers, and global styles.
- `src/assets/`: images, icons, and fonts.
- `src/components/`: shared UI and robot-control components.
- `src/features/`: larger, self-contained feature modules, each with its own
  API/model/component files — `blocks/` (Blockly programming, toolbox, action
  executor, and the dedicated guide), `devices/`, `recordings/`, and
  `robotDescription/` (the URDF/Xacro digital-twin viewer).
- `src/layouts/`: page layout components (`appLayout.jsx` — header, status
  bar, always-mounted background components, and the active page).
- `src/pages/`: route-level pages, driven by `src/pages/registry.js`'s
  `PAGE_REGISTRY` (the single source of truth for routing and the sidebar —
  see `docs/extending/add-a-ui-panel.md`).
- `src/plugins/`: example/optional page plugins registered through
  `src/shared/plugins/registerPlugin.js` instead of editing the core
  registry directly (see `notesPlugin/` for a complete working example).
- `src/shared/`: constants, styles, reusable UI primitives, and small
  cross-cutting feature folders — `hooks/`, `events/` (the Events page's
  log), `missions/` and `schedules/` (Missions/Scheduler state and runners),
  `support/` (support-package export), `demo/` (Demo Mode), `help/` and
  `tour/` (onboarding/help widget), `i18n/`, and `plugins/` (the plugin
  registry itself).
- `src/stores/`: Redux store setup (used for exactly one thing — the console
  log message list; see `docs/lessons/07-ui-components.md`).
- `package.json`: frontend dependencies and npm scripts.

For what each top-level page actually shows, see
[`docs/lessons/06-the-pages.md`](../docs/lessons/06-the-pages.md).
