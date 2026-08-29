# Contributing to OpenAMRobot UI

Thank you for improving OpenAMRobot UI. Changes can affect real robot motion,
so keep patches focused, test them in Demo Mode or simulation first, and
describe any safety impact clearly.

## Safety notice

This repository can affect real-robot behavior. Contributors and users are
responsible for validating robot safety (E-stop, watchdog, fault handling),
navigation, docking, motor control, sensor integration, deployment
suitability, and regulatory compliance for their own setup.

This software is provided for research, education, and development.

## Before you start

1. Read the [README](README.md) and the
   [lessons](docs/lessons/README.md).
2. For a new panel, device, or Blockly block, choose the matching
   [extension guide](docs/extending/README.md).
3. Search existing issues and pull requests before starting duplicate work.
4. Never commit API keys, `.env` files, robot credentials, recordings with
   sensitive data, or private network details.
5. Keep a pull request scoped to the package or feature directory the task
   is about — for example, a Blockly change should normally stay within
   `web/src/features/blocks/`, and a devices change within
   `web/src/features/devices/` and the device-facing code in
   `ros2/src/openamr_ui_package/`. If a change genuinely needs to touch a
   sibling area too (a shared constant, a relay another feature depends on),
   explain why in the PR.

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

`npm run lint` runs Prettier (`web/prettier.config.js`) followed by ESLint.
The Prettier step works and rewrites files in place — run it only when you're
ready to inspect and keep those edits. The ESLint step does not: this repo
has no ESLint config file, so `lint:js` has no rules to check against.
`npm run build` is the real correctness gate — `react-scripts build` runs
Create React App's own bundled lint rules regardless of a project config.

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
Third-party assets bundled as static files are attributed in the README's
[Third-party notices](README.md#third-party-notices) section.

## Legal requirements

OpenAMRobot is operated by **Botshare LTD**. Before a contribution can be accepted:

1. every commit must be signed off under the [DCO](https://github.com/openAMRobot/.github/blob/main/DCO.md);
2. the contributor must be covered by the applicable [Individual or Corporate Contributor Agreement](https://github.com/openAMRobot/.github/blob/main/CLA.md);
3. any necessary employer, university, client, sponsor, co-author, or institutional authorization must be obtained; and
4. third-party and material AI-assisted content must be disclosed with its source, licence, and required notices.

The Contributor Agreement governs assignment of transferable economic rights in accepted contributions to Botshare LTD. DCO confirms provenance and authority but does not replace that agreement. Do not submit unauthorized confidential information, personal data, credentials, export-controlled material, or third-party material with unclear rights.
