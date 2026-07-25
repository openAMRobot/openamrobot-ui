# Contributing to OpenAMRobot UI

Thank you for improving OpenAMRobot UI. Changes can affect real robot motion,
so keep patches focused, test them in Demo Mode or simulation first, and
describe any safety impact clearly.

## Before you start

1. Read the [README](README.md) and the
   [lessons](docs/lessons/README.md).
2. For a new panel, device, or Blockly block, choose the matching
   [extension guide](docs/extending/README.md).
3. Search existing issues and pull requests before starting duplicate work.
4. Never commit API keys, `.env` files, robot credentials, recordings with
   sensitive data, or private network details.

## Development setup

Use the
[manual installation guide](docs/installation.md#manual-installation).
For frontend-only work:

```bash
cd web
npm ci
npm run dev
```

For the production UI:

```bash
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
source /opt/ros/jazzy/setup.bash
bash scripts/build_ros.sh
```

## Checks

Run checks appropriate to your change:

```bash
cd web
CI=true npm test -- --watchAll=false
npm run build
```

`npm run lint` currently applies formatting and ESLint fixes. Run it only
when you are ready to inspect and keep those edits.

For ROS packages:

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
colcon test --packages-select openamr_ui_package openamr_ui_msgs
colcon test-result --verbose
```

Also verify all changed user workflows manually. Prefer this order:

1. Demo Mode.
2. Simulation.
3. Real hardware with a clear area, low speed limits, and a tested physical
   emergency stop.

## Pull requests

Include:

- What changed and why.
- The pages, ROS topics, services, or actions affected.
- The commands and manual scenarios used for verification.
- Screenshots for visible UI changes.
- Any compatibility, migration, persistence, networking, or safety impact.

Keep generated folders such as `web/build`, `web/node_modules`, `ros2/build`,
`ros2/install`, and `ros2/log` out of commits.

## Documentation

Update the README or lessons whenever setup commands, routes, configuration,
topics, persistence, failure behavior, or operator-visible controls change.
Write for a reader who is new to this repository, define unfamiliar terms,
and include an observable success condition.

## License

Contributions are accepted under the repository's [MIT License](LICENSE).
