# Development Guide

Read [CONTRIBUTING.md](../CONTRIBUTING.md) before changing robot controls,
topics, persistence, networking, or safety behavior.

## Frontend development server

```bash
cd ~/openamrobot-ui/web
npm ci
npm run dev
```

Open `http://localhost:3000`.

The development server can render without ROS. Live robot data and controls
require rosbridge on port `9090`. When the page runs on port `3000`, the
fallback robot address comes from `ROSBRIDGE_SERVER_IP` in
`web/src/shared/constants/index.js`.

The development server bypasses Flask. Flask REST features therefore still
require the backend on port `5050`.

## Production frontend workflow

The production path has three stages:

```text
web/src
  → npm production build in web/build
  → synchronized ROS static/app directory
  → installed openamr_ui_package share directory
```

Run:

```bash
cd ~/openamrobot-ui
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh

cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select openamr_ui_package
source install/setup.bash
ros2 launch openamr_ui_bringup ui.launch.py
```

Hard-refresh the browser with `Ctrl+Shift+R` after rebuilding.

## Frontend checks

```bash
cd ~/openamrobot-ui/web
CI=true npm test -- --watchAll=false
npm run build
```

`npm run lint` applies Prettier and ESLint fixes. Run it only when you are
ready to review and keep those edits.

## ROS development

Build all packages:

```bash
cd ~/openamrobot-ui/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build selected packages:

```bash
colcon build --symlink-install \
  --packages-select openamr_ui_msgs openamr_ui_package openamr_ui_bringup
```

Run ROS tests:

```bash
colcon test --packages-select openamr_ui_package openamr_ui_msgs
colcon test-result --verbose
```

After changing messages:

```bash
colcon build --symlink-install \
  --packages-select openamr_ui_msgs openamr_ui_package
source install/setup.bash
```

## Run-state fixture (development/test only)

A deterministic run-state fixture feeds the read-only current-run view at
`/dev/current-run`. The envelope is **Proposed, fixture-only, pending owner
review**. See [the proposal](proposals/i5-run-state-fixture.md). The
fixture is not an executor, and its data is development test data.

- It is off by default. With the flag unset, `/api/dev/run-state-fixture/*`
  returns 404 and the view shows "Fixture backend disabled". There is no
  Config toggle, and Demo Mode is unchanged.
- To enable it with Compose (this is the only opt-in):

  ```bash
  OPENAMR_RUN_STATE_FIXTURE_SCENARIO=failed \
    docker compose -f docker-compose.yml -f docker-compose.run-state-fixture.yml up
  ```

  Outside Compose, start the backend with `OPENAMR_RUN_STATE_FIXTURES=1`.
  The available scenarios are `success` (the default), `failed`,
  `producer_restart`, `new_run` and `stale`.
  `OPENAMR_RUN_STATE_FIXTURE_STEP_SECONDS` sets the step interval
  (default 3).
- Pure backend tests, run without ROS from `ros2/src/openamr_ui_package`:

  ```bash
  python3 -m pip install -r test/requirements-run-state-fixture.txt
  python3 -m pytest test/test_run_state_fixture.py
  ```

- Frontend tests, run from `web/`:

  ```bash
  CI=true npx react-scripts test --watchAll=false --testPathPattern='(runState|CurrentRunFixture)'
  ```

## Where to make changes

| Change | Start here |
| --- | --- |
| Add a page or panel | [UI panel guide](extending/add-a-ui-panel.md) |
| Connect a topic or external device | [External-device guide](extending/connect-external-device.md) |
| Add a Blockly block | [Blockly extension guide](extending/add-a-blockly-block.md) |
| Understand page composition | [Lesson 06](lessons/06-the-pages.md) |
| Understand shared components | [Lesson 07](lessons/07-ui-components.md) |
| Change a topic name or message type | [Lesson 10](lessons/10-topics-as-the-contract.md) |
| Change backend nodes | [Lesson 05](lessons/05-backend-nodes-in-detail.md) |

## Generated directories

Do not edit or commit:

```text
web/node_modules/
web/build/
ros2/build/
ros2/install/
ros2/log/
```

Edit sources under `web/src/` and `ros2/src/`.

## Verification order

For a behavior that can move hardware:

1. Verify static checks and unit tests.
2. Test in Demo Mode when the feature supports it.
3. Test in simulation.
4. Test on hardware at low speed with a clear area and tested physical
   emergency stop.

Record the commands, scenario, and observed result in the pull request.
