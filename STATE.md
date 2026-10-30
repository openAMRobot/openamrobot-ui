# STATE: openamrobot-ui

Read this before working in this repository and update it in the same pull
request (AGENTS.md, Decisions). Last updated by the Gate A command path PR
(branch `ui/gate-a-command-path`, parent `0ca798b4`, 30 Oct 2026).

## UI state

- React frontend in `web/`, Flask and ROS packages in `ros2/`. One shared
  rosbridge connection (`web/src/app/App.jsx`).
- Manual drive (joystick, gamepad, map page drive controls, Blockly drive,
  rotate and stop blocks, voice-planned drive actions run through Blockly)
  goes through one module, `web/src/shared/motion/uiCommand.js`. It
  publishes `geometry_msgs/TwistStamped` on `/ui/cmd_vel` only, at 20 Hz
  while held and one zero on release. No UI code publishes `/cmd_vel` or any
  `cmd_vel_*` topic; a Jest static guard and a page runtime test enforce it.
  There is no keyboard teleop in the UI today.
- Manual drive is enabled only when rosbridge is connected, the platform
  stop lock (`/safety/stop_lock`) is known and released, and the robot clock
  is synced. Simulation stamps with the latest `/clock`; the real robot
  stamps from `/rosapi/get_time` samples. The mode follows
  `/ui_cmd_vel_relay_node`'s own `use_sim_time`.
- Stop (software) engages `/safety/stop_lock_engage`, cancels every active
  `navigate_to_pose`, `compute_path_to_pose`, `dock_robot` and
  `undock_robot` goal, and aborts UI programs, missions and the map page
  waypoint queue. Resume is the only call to `/safety/stop_lock_release`,
  behind a confirmation. It is a functional stop (E-01, issue #34), not the
  hardwired E-stop and not a safety function.
- `new_ui_launch.py` passes `use_sim_time` to every node it starts.
- Until openamr-platform-sw#90 is merged, the relay and stop lock do not
  exist on platform `main`, so manual driving from the UI is intentionally
  inert there (the lock stays unknown).
- The generated React bundle is not committed (`.gitignore`,
  `ros2/.../static/app/README.md`); build it with `scripts/build_frontend.sh`
  and `scripts/sync_frontend_to_ros.sh`.

## Gate A status (6 Nov 2026)

- Needs the Gate A command path PR merged after openamr-platform-sw#90.
- #31 and #34 stay open until the simulation reruns S4, S6 and S7a pass
  (simulation checks: Nikolai). Pull requests say "Part of", never "Closes".
- Not verified in this repository: simulation, Getac ZX80 panel, physical
  robot.

## Open issues

| Issue | Title | Notes |
|---|---|---|
| #31 | UI teleop into a twist_mux input ahead of the smoother | Gate A. UI side in the Gate A command path PR. |
| #34 | Stop (software) with stop lock, not labelled E-STOP | Gate A. UI side in the Gate A command path PR. |
| #19 | Map navigation and base status to Robot Status and Reporting Mode | Reporting Mode, owner Parth. |
| #32 | UI no longer starts its own navigation or mapping stack | Draft PRs #40, #41. |
| #33 | Recording replay restricted to non-command topics | Draft PR #43. |
| #35 | Server-side limits on voice-planned actions | Draft PR #45. |
| #36 | Report real route outcomes and stale pose state | Draft PR #46. |
| #37 | Store maps and routes outside the installed package | |
| #28 | Harness adoption: repository not yet rolled out | |
| #29 | AGENTS.md is still shared-rules v1 | |
| #39 | Harness: check disk capacity and containerd isolation before Docker deployments | |

## Open pull requests

| PR | Branch | Status |
|---|---|---|
| Gate A command path and stop lock | `ui/gate-a-command-path` | Draft. Merges only after openamr-platform-sw#90. |
| #46 | `ui/route-outcomes` | Draft |
| #45 | `ui/voice-bounds` | Draft |
| #44 | `ui/bag-process-recovery` | Draft |
| #43 | `ui/replay-policy` | Draft |
| #42 | `ui/lifecycle-read-only` | Draft |
| #41 | `ui/remove-legacy-launch` | Draft |
| #40 | `ui/no-ui-nav-launch` | Draft |
| #21 | `claude/eloquent-newton-7h8tnn` | Draft. Earlier fixture prototype for Reporting Mode. |

## Owners (Alex, 19 Oct 2026)

- Parth Keshari: UI lead and Reporting Mode owner.
- Mohamed Sayed and ND: reviewers.
- Nikolai: simulation checks.
- Kunal: platform side (openamr-platform-sw#90).
