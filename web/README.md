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
rosbridge to be running and reachable. When using the dev server, update
`ROSBRIDGE_SERVER_IP` in `src/shared/constants/index.js` if the robot is not at
the default address.

## Production Build

From the repository root, use the canonical scripts:

```bash
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
```

`build_frontend.sh` creates `web/build/`. `sync_frontend_to_ros.sh` copies that
build into the ROS package static app directory so Flask can serve it.

## Structure

- `public/ros/`: browser ROS libraries copied into the production build.
- `src/app/`: top-level React app setup, providers, and global styles.
- `src/assets/`: images, icons, and fonts.
- `src/components/`: shared UI and robot-control components.
- `src/features/blocks/`: Blockly robot-programming blocks, toolbox, action
  executor, and the dedicated Blockly guide.
- `src/layouts/`: page layout components.
- `src/pages/`: route-level pages.
- `src/shared/`: constants, styles, and reusable UI primitives.
- `src/stores/`: Redux store setup.
- `package.json`: frontend dependencies and npm scripts.
