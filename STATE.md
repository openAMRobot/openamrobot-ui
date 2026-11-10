# STATE: openamrobot-ui

Working state of this repository for contributors and agents. Read it before
starting work; update it in the same pull request (AGENTS.md, Decisions).

Last updated: 10 Nov 2026, base `main` at
`51b4af38d6b7eeb260610e8ab2ebde0a340beea9` (#48 merged). The B items below
were written against `main` at `0ca798b424cadbb77d6791f5298bb4f7e0f37a89`.

This file covers the backend and process items for v2.0.0-rc.1 listed
below (B1 to B10) and the Gate A UI command path (A1, at the end). Reporting
Mode is tracked by its own pull request.

## Ownership

Decision of 19 Oct: Parth Keshari (UI lead, Reporting Mode owner); Mohamed
Sayed and ND (reviewers); Nikolai (simulation checks); Kunal (platform
side). Every item below is an open draft pull request that stays draft until
Parth writes adopt, adapt or reject in its thread.

## Items

Each item is one draft pull request from `main`. "Evidence" is the highest
layer reached; none of these items has been run in simulation, on the Getac
ZX80 or on a robot.

### B1 `ui/no-ui-nav-launch` (part of #32)

- Maps actions no longer run `ros2 launch`, `ros2 run` or `ros2 lifecycle`,
  and no longer publish `/initialpose`. Save map calls the platform map saver
  (`nav2_msgs/SaveMap`, parameter `map_save_service`, default
  `/map_saver/save_map`) and then `/map_server/load_map`. Build map only
  explains that mapping runs on the platform.
- To confirm: the platform's map saver service name; whether the platform
  map_server can read the UI data directory (see B8).
- Evidence: unit tests (pure and rclpy).

### B2 `ui/remove-legacy-launch` (part of #32)

- Deletes `launch/map_server_launch.py` (unreferenced). Keeps
  `mapping_launch.py`, `navigation_launch.py`, `gmapping_launch.py`,
  `move_base_launch.py`, `amcl_launch.py` and `param/move_base/`, because
  `folders_handler.py` on `main` still launches them.
- Next step: once B1 is merged, delete the kept files in a follow-up.
- Evidence: unit test (pure).

### B3 `ui/lifecycle-read-only`

- The lifecycle panel only calls `GetState`; the Prepare, Start, Pause and
  Reset buttons are gone.
- Evidence: component tests.

### B4 `ui/replay-policy` (part of #33)

- Replay plays only the allowlist in `param/replay_topics.yaml` via
  `--topics`, and refuses (HTTP 400) any request or recording containing
  command, transform, goal, pose-reset, safety, action or service topics.
- To confirm: the allowlist contents (proposed default). A recording made
  with "Record all topics" can no longer be replayed.
- Evidence: unit tests (pure and Flask test client).

### B5 `ui/bag-process-recovery`

- Bag processes are recorded in `~/.openamr_ui/recordings/bag_processes.json`
  (pid, start time, command line). On server start a leftover is stopped only
  when all three match, and its recording is marked interrupted.
- Evidence: unit tests (fake process table and one real process).

### B6 `ui/voice-bounds` (part of #35)

- `param/voice_limits.yaml`, `GET /api/limits`, and `/api/voice-plan`
  rejecting (HTTP 422) non-finite or over-limit plans. The Blocks page reads
  the limits from the backend.
- To confirm: distance per action 5 m and duration per action 30 s are
  proposed defaults (ND and Parth).
- Evidence: unit tests (pure, Flask test client, Jest).

### B7 `ui/route-outcomes` (part of #36)

- Route following reports SUCCEEDED, ABORTED, CANCELED or TIMEOUT per
  waypoint; a route succeeds only if every waypoint did. `nav_relays.py`
  repeats the AMCL pose only while the map to odom transform is younger
  than `pose_stale_after_s` (2.0 s).
- To confirm: `waypoint_timeout_s` default 300 s (proposed); the 2.0 s
  threshold against the platform's AMCL `transform_tolerance`.
- Not done here: mission outcomes in the browser mission runner (left to the
  UI command path work).
- Evidence: unit tests (pure and rclpy with a fake navigator).

### B8 `ui/persistent-maps-routes` (part of #37)

- Maps, routes and `current_map_route.yaml` live in `~/.openamr_ui`, seeded
  from the package without overwriting. `docker-compose.yml` already keeps
  that directory on the `openamr_ui_data` volume.
- Evidence: unit tests (pure and rclpy).

### B9 `ui/ci`

- CI gains a `ros` job (ros:jazzy container, colcon build and test, fails on
  zero executed tests) and an `e2e` job (Playwright, every page in Demo Mode
  at 1280 x 800 and 1920 x 1200, no console errors).
- The zero-test guard is `scripts/count_test_results.py`. It reads the JUnit
  files colcon writes, prints total, skipped and executed separately, and
  fails when no result file exists or executed (total minus skipped) is 0,
  so an all-skipped run fails (review of #48). Its own tests
  (`scripts/test_count_test_results.py`) run as a step of the same job.
- Malformed or unreadable XML under the build folder also fails the guard,
  even when another result file has executed tests; it names each such file
  and labels the counts incomplete. Well-formed non-JUnit XML is still
  ignored. This fix is Mohamed's #57, merged into `ui/ci` as
  `bd82a1ada6f5a02dcb1f1d25385fb44e65e63216` (second review of #48).
- Evidence: job steps reproduced locally; the jobs themselves run on the
  pull request.

### B10 `ui/docs-ownership`

- README and docs: Stop (software), Getac ZX80, data directory, replay
  policy; AGENTS.md ownership line; CODEOWNERS adds @parthkeshari; issue #19
  body updated.
- Merge after B4 and B8, whose behaviour it documents.
- Evidence: Jest docs consistency test.

## Known gaps

- The Stop (software) lock is not on `main` yet; see A1 below.
- The Blocks page loads Blockly sounds and sprites from
  `blockly-demo.appspot.com`, which an offline robot cannot reach.
- `test_flake8.py`, `test_pep257.py` and `test_copyright.py` are skipped
  without a tracking issue, and most existing Python files are not flake8
  clean.

## A1 `ui/gate-a-command-path` (part of #31, part of #34)

Base `main` at `51b4af38d6b7eeb260610e8ab2ebde0a340beea9`, merged into the
branch at `d527ea65` (branched from `0ca798b4`). Merge only after
openamr-platform-sw#90, the source of the interfaces below. The interfaces
were checked at #90 `710c98bc3d384a3d0612e87ff5e8c0c00b804dc0` and the stop
gate tick rate at `a23bca14`. #90's current head
`2425d4d75fd309ca97b1e75b92db47e7ba08df8f` (open, D-13 status-readiness
changes pending) is not re-checked: refresh the pin once #90's D-13 changes
are accepted, and re-check topic names, QoS and timings at its merged
commit. Evidence from the older heads is not current for #90's final
interface.

- Manual drive (joystick, gamepad, map page drive controls, Blockly drive,
  rotate and stop blocks, voice-planned drive actions run through Blockly)
  goes through one module, `web/src/shared/motion/uiCommand.js`. It
  publishes `geometry_msgs/TwistStamped` on `/ui/cmd_vel` only, at 20 Hz
  while held and one zero on release. No UI code publishes `/cmd_vel` or
  any `cmd_vel_*` topic; a Jest static guard and a page runtime test check
  it. There is no keyboard teleop in the UI.
- Manual drive is enabled only when rosbridge is connected, the platform
  stop lock (`/safety/stop_lock`) is known and released, and the robot
  clock is synced. Simulation stamps with the latest `/clock`; the real
  robot stamps from `/rosapi/get_time` samples (rosbridge_suite 2.7.1
  returns wall time from it even with `use_sim_time` true). The mode
  follows `/ui_cmd_vel_relay_node`'s own `use_sim_time`.
- Stop (software) engages `/safety/stop_lock_engage`, cancels every active
  `navigate_to_pose`, `compute_path_to_pose`, `dock_robot` and
  `undock_robot` goal, and aborts UI programs, missions and the map page
  waypoint queue. Resume is the only call to `/safety/stop_lock_release`,
  behind a confirmation. It is a functional stop (E-01, issue #34), not the
  hardwired E-stop and not a safety function.
- `new_ui_launch.py` passes `use_sim_time` to every node it starts.
- Until #90 is merged, manual driving from the UI is intentionally inert on
  `main` (the lock stays unknown).
- Gate A (6 Nov): needs A1 merged after #90. #31 and #34 stay open until
  Nikolai's simulation reruns S4, S6 and S7a pass.
- Confirmed by Parth (1 Nov) and Kunal (4 Nov): 10 ms `/clock` throttle,
  1 s stop lock staleness (20 missed ticks at the gate's 20 Hz), 2 s retry
  for the relay parameter read.
- Stamping (Kunal, 4 Nov, platform-sw#110): the browser stamps on the
  robot clock as above; the relay does not restamp, so a replayed or
  delayed command still ages out. #110 is resolved by this design.
- Parth adopted A1 on 1 Nov; merge after #90. Before merge, re-check topic
  names, QoS and timings against #90's merged commit.
- Stop (software) also latches locally at once (`motionGate` `localStop`),
  before the engage request answers; held input, programs, missions and
  goals stay blocked until a confirmed Resume whose release succeeds. A
  released lock sample or a reconnect does not clear it (review of #50).
- While the lock is engaged or unknown, no Nav2 goal is sent from the map
  page (goal, home, waypoints, queue, docking), missions, route planning or
  programs; the reason is shown (`shared/motion/navGoalGate.js`).
- README and docs/ describe the latched Stop (software), the explicit
  Resume and the disabled controls (`web/src/shared/motion/stopDocs.test.js`).
  Draft #49 describes the earlier one-shot stop; where both change the same
  lines, keep this PR's text.
- Platform gap (openamr-platform-sw, owner Kunal): the `dock_trigger.py`
  docking sequence is not an action and cannot be cancelled by the UI.
- `.github/decisions.yaml` entries for the UI command topic and the stop
  lock interface: needed, handled separately in openAMRobot/.github.
- Evidence: unit and component tests (Jest, mocked ROSLIB), pure launch
  test. Not run in a browser, in simulation, on the Getac ZX80 or on a
  robot.

Open issues on 30 Oct: #19, #28, #29, #31, #32, #33, #34, #35, #36, #37,
#39. Open pull requests on 30 Oct: drafts #21 and #40 to #46, plus A1.

## Focused B9 follow-up, 30 October 2026 (#57)

Written in #57, which was stacked on #48 at
`216f79629636023c924fd070701f8b814f860cbd` and is now merged into `ui/ci`
(#48) as `bd82a1ada6f5a02dcb1f1d25385fb44e65e63216`. The inherited item
summaries above are not whole-stack acceptance.

The counter now fails on malformed or unreadable XML under the requested
build directory even when another result contains executed tests. It names
each unreadable file and labels the counts incomplete. Well-formed
non-JUnit XML remains ignored. Existing skipped-test counting is unchanged;
colcon test-result still owns ordinary test-failure reporting.

The existing standard-library test command runs nine cases with no skips.
Keeping these tests and restoring the original helper produces three
failures and one error; the fixed helper passes all nine. No ROS, Docker,
web build, simulation or hardware run was performed locally for this fix.
#48 merged on 10 Nov 2026 as 51b4af3, owned by the platform lead. #51 incorporates the fix by merging `ui/ci`; integration candidate
#56 must be rebuilt by its owner to include it.
