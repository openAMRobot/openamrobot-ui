# STATE: openamrobot-ui

Working state of this repository for contributors and agents. Read it before
starting work; update it in the same pull request (AGENTS.md, Decisions).

Last updated: 31 Oct 2026. This copy is on `ui/rc1-candidate`, an
integration candidate (draft openAMRobot/openamrobot-ui#56) built from `main` at
`0ca798b424cadbb77d6791f5298bb4f7e0f37a89` by merging the open draft pull
requests #40 to #55 at their current heads. It is a test artifact: it never
merges to `main` and does not replace per-PR review or adoption. Each item
section below is taken from its own pull request's head; the "In this tree"
and "Review" lines and the other sections describe the combined tree.

It covers the backend and process items for v2.0.0-rc.1 (B1 to B13), the
Gate A UI command path (A1, #50), Reporting Mode (A2, #54) and the
read-only upper-body view (C1, #55).

## Ownership

Decision of 19 Oct: Parth Keshari (UI lead, Reporting Mode owner); Mohamed
Sayed and ND (reviewers); Nikolai (simulation checks); Kunal (platform
side). Every item below is a draft pull request until Parth writes adopt,
adapt or reject in its thread.
- #55's own STATE.md names Mohamed (interim) as owner role, citing the
  AGENTS.md on `main`; with #49 merged here, AGENTS.md carries the 19 Oct
  ownership above.

## Accepted defaults (owner decision, 30 Oct)

- Replay telemetry allowlist in `param/replay_topics.yaml`.
- Voice plan limits: 0.2 m/s, 2.0 rad/s, 5 m and 30 s per action.
- `waypoint_timeout_s` 300 s; `pose_stale_after_s` 2.0 s.
- Platform map save service `/map_saver/save_map`.
- Maps on the shared `openamrobot_maps` volume at `/data/maps`.

## Items

"Evidence" is the highest layer the item's own pull request reached. None
of these items has been run in simulation, on the Getac ZX80 or on a robot.
Unit tests and fixtures are not safety evidence. "Review" lines summarise the
latest review on each pull request as of this build.

### B1 #40 `ui/no-ui-nav-launch` (part of #32)

- Maps actions no longer run `ros2 launch`, `ros2 run` or `ros2 lifecycle`,
  and no longer publish `/initialpose`. Save map calls the platform map saver
  (`nav2_msgs/SaveMap`, parameter `map_save_service`, default
  `/map_saver/save_map`) and then `/map_server/load_map`. Build map only
  explains that mapping runs on the platform.
- Rename and delete work for maps saved by SaveMap (`<name>.yaml` and the
  image only); a legacy `<name>_ros.yaml` is handled when it exists. A
  failed save or load reaches the UI as a clear message.
- Rename and delete change only files inside the map's own folder
  (`map_files.py`). The yaml's `image` path is resolved, symlinks included;
  an image outside the folder (absolute path, `..`, or a symlink out) refuses
  the whole operation, changes nothing and tells the UI why. An image that
  another map in the folder also names is kept (review of #40).
- Owner decision (30 Oct): save service `/map_saver/save_map`; maps on the
  shared `openamrobot_maps` volume at `/data/maps` in the UI and platform
  containers (#53). The platform-side mount and launch are a platform-sw
  change owned by Kunal; the full Maps workflow needs that and a simulation
  check (Nikolai).
- Evidence: unit tests (pure and rclpy); one local run against real
  nav2_map_server and map_saver_server with a temporary map folder.
- In this tree: with #41 and #52 also merged, the package and launch READMEs say no launch starts SLAM, AMCL, Nav2 or map_server and name both deletions.
- Review (latest, Mohamed, comment on `8ea0c498`): the external-image P1 is addressed. Open: the platform `/map_saver/save_map`, the shared `/data/maps` mount and permissions, and SaveMap, list, LoadMap and restart on the combined stack, plus unavailable-service and failed-load behaviour.

### B2 #41 `ui/remove-legacy-launch` (part of #32)

- Deletes `launch/map_server_launch.py` (unreferenced). Keeps
  `mapping_launch.py`, `navigation_launch.py`, `gmapping_launch.py`,
  `move_base_launch.py`, `amcl_launch.py` and `param/move_base/`, because
  `folders_handler.py` on `main` still launches them.
- Next step: once B1 is merged, delete the kept files in a follow-up.
- Evidence: unit test (pure).
- In this tree: B1 (#40) and B12 (#52) are merged too, so the kept files are
  deleted by #52; no legacy launch remains.
- Review (latest, Kunal, approved `d5576438`): no open finding.

### B12 #52 `ui/remove-nav-launches` (part of #32, stacked on #40)

- Deletes `mapping_launch.py`, `navigation_launch.py`, `gmapping_launch.py`,
  `move_base_launch.py`, `amcl_launch.py` and `param/move_base/`. This is
  the follow-up named under B2; it needs B1, which removed the last code
  that launched them.
- Keeps `launch/map_server_launch.py`; B2 (#41) deletes it, and the READMEs
  say so.
- Its reference test scans only runtime and configuration consumers (launch
  files, package code, setup.py, setup.cfg, package.xml, Dockerfile,
  docker-compose.yml, scripts/), not test directories.
- Evidence: unit test (pure source and file check); combined with B1 and B2
  in a local scratch merge.
- In this tree: B2 (#41) is merged too, so `map_server_launch.py` is deleted.
- Review: approved by Mohamed on `208368d8`; Kunal found no platform-sw reference to the deleted files. No open finding. Head `06a21d29` adds only the merge of #40's fix.

### B3 #42 `ui/lifecycle-read-only`

- The lifecycle panel only calls `GetState`; the Prepare, Start, Pause and
  Reset buttons are gone.
- Evidence: component tests.
- Review (latest, Mohamed, approved `679cbd58`): no open finding.

### B4 #43 `ui/replay-policy` (part of #33)

- Replay plays only the allowlist in `param/replay_topics.yaml` via
  `--topics`. Without a topic list it plays the recording's allowlisted
  topics and lists the skipped ones; HTTP 400 only when a request names a
  command, transform, goal, pose-reset, safety, action or service topic.
- The allowlist is the accepted default (owner decision, 30 Oct).
- `replay_policy.py`'s opening docstring now says the same: a recording with
  denied topics is filtered, not rejected (review of #43).
- Evidence: unit tests (pure and Flask test client).
- Review (latest, Mohamed, comment on `de189b71`): the docstring and owner-decision items are fixed; no open code finding. Open acceptance gates: Nikolai's simulation replay of a mixed telemetry, command and TF recording showing zero robot travel, a denied request starting no process, the skipped-topic display in the frontend, and Parth's adoption.

### B5 #44 `ui/bag-process-recovery` (merges after #43)

- Bag processes are recorded in `~/.openamr_ui/recordings/bag_processes.json`
  (pid, start time, command line). On server start a leftover is stopped only
  when all three match, and its recording is marked interrupted.
- Every read-modify-write of that registry runs under one lock (a
  threading lock plus an fcntl lock on a sibling .lock file); each write
  goes through a unique temporary file, fsync and os.replace. If the
  registry write fails after a bag process starts, that process is stopped
  and reaped and the request returns 500.
- The same happens when the new process's start time and command line cannot
  be read from /proc: `remember()` raises instead of writing nothing, and
  `track_or_stop()` stops and reaps the child, so no bag process runs
  untracked (review of #44).
- Right after `Popen`, /proc can still show an empty command line (between
  fork and exec). `remember()` reads it again, up to 20 times 10 ms apart
  (`IDENTITY_READ_ATTEMPTS`, `IDENTITY_READ_INTERVAL_S`), and treats a
  command line still empty after that as unreadable, so a wrong identity is
  never recorded.
- This branch contains #43 (merged, not rebased) and merges after it. #43's
  replay API tests give their fake bag process a readable pid, start time
  and command line; a real replay process whose details cannot be read is
  stopped and the request returns 500.
- Evidence: unit tests (fake process table, real child processes, threads
  meeting at barriers).
- In this tree: the replay start runs B4's policy checks before the process starts and B5's `track_or_stop` after it; #43's replay API tests give their fake process a readable identity.
- Review (latest, Mohamed, on `9c0f573f`, shown as dismissed after the next push): the unreadable-identity and empty-command-line findings are cleared. No open code finding; `3e5402ab` (the merge of #43's new head) is not re-reviewed. Open: a real `ros2 bag` run and a Flask restart.

### B6 #45 `ui/voice-bounds` (part of #35)

- `param/voice_limits.yaml`, `GET /api/limits`, and `/api/voice-plan`
  rejecting (HTTP 422) non-finite or over-limit plans. The Blocks page reads
  the limits from the backend.
- Distance per action 5 m and duration per action 30 s: accepted default
  (30 Oct 2026).
- Evidence: unit tests (pure, Flask test client, Jest).
- Review (latest, Mohamed, approved `53f531c8`): no open code finding. Open: record the source of the 30 Oct owner decision on the limits.

### B7 #46 `ui/route-outcomes` (part of #36)

- Route following reports SUCCEEDED, ABORTED, CANCELED or TIMEOUT per
  waypoint; a route succeeds only if every waypoint did. `nav_relays.py`
  repeats the AMCL pose only while the map to odom transform is younger
  than `pose_stale_after_s` (2.0 s), measured with a monotonic clock. A
  backward ROS clock jump (simulation reset) drops the cached pose until a
  new pose arrives. Non-finite or non-positive thresholds stop the node.
- `waypoint_timeout_s` 300 s and the 2.0 s pose threshold: accepted default
  (30 Oct 2026).
- Not done here: mission outcomes in the browser mission runner (left to the
  UI command path work).
- Evidence: unit tests (pure and rclpy with a fake navigator).
- Review (latest, Mohamed, approved `2a25e28a`): no open code finding. Open: Nav2 TF/QoS and stale behaviour in Nikolai's simulation; platform owner to confirm the parameters.

### B8 #47 `ui/persistent-maps-routes` (part of #37)

- Maps, routes and `current_map_route.yaml` live in `~/.openamr_ui`.
  `docker-compose.yml` already keeps that directory on the `openamr_ui_data`
  volume.
- Seeding from the package runs once per data directory (`.seeded` marker),
  so a deleted example stays deleted. Seeded files are published with
  `os.link`, the selection file and marker with `O_CREAT | O_EXCL`; nothing
  existing is ever overwritten.
- Platform readability: maps reach the platform map saver and map server
  through the `openamrobot_maps` volume at `/data/maps` (#53). The platform
  mount is a platform-sw change (Kunal); save, list and load across a
  container restart is a simulation check for Nikolai.
- Evidence: unit tests (pure and rclpy).
- Review (latest, Mohamed, approved `ca5ca0a7`): no open code finding. Open: cross-container save and load (with #53 and the platform mount).

### B13 #53 `ui/shared-maps-volume` (part of #37, stacked on #47)

- Maps go to `OPENAMROBOT_MAPS_DIR` (default: `maps` in the data
  directory); routes stay in the data directory.
- `docker-compose.yml` sets it to `/data/maps` on the named volume
  `openamrobot_maps`.
- Non-destructive migration: when `OPENAMROBOT_MAPS_DIR` differs from the
  data directory's `maps` folder and that folder has maps, they are copied
  (never moved, deleted or overwritten; the old folder stays as a backup)
  and the selection is repointed when its map was copied.
- A map counts as migrated only when its image is inside the old folder,
  exists, is readable, and was copied or found byte-identical in the new
  folder, and its name is free there (reviews of #53). Otherwise it is
  skipped and logged with its reason: an image outside the old folder
  (absolute path elsewhere, `..`, or a symlink out), a missing or unreadable
  image, or a name that already exists in the new folder. Nothing of a
  skipped map is copied or selected, and the outside file is not touched.
- Owner decision of 30 Oct 2026: while any map is skipped, selected or not,
  `.maps_migrated` is not written; the migration runs again on every start
  and logs each skipped map with its reason until it is fixed or removed.
  Once nothing is skipped, the marker is written. A skipped selected map
  keeps the old selection, with an error that says why and how to fix it.
  Maps an earlier pass already copied count as migrated, so a retry does
  not report or rewrite them; the check (`_already_copied`) needs every file
  present and identical on both sides.
- Open, outside this repository: the platform compose must mount the same
  `openamrobot_maps` volume at `/data/maps` (a pending platform-sw change).
- Evidence: unit tests (pure and rclpy) and a compose file check.
- Review (latest, Mohamed, comment on `d0a8f6ef`): the completion-marker and retry finding is fixed (owner decision of 30 Oct: any skipped map keeps the migration open); no new code finding. Not an integration approval; the earlier changes-requested review on `eb359fde` has not been replaced by an approval. Open: Parth's adoption, the platform volume mount at `/data/maps`, and a real SaveMap, LoadMap and restart check.

### B9 #48 `ui/ci`

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

#57 follow-up, from #48's head:


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
Hosted checks and Parth's adopt/adapt/reject decision on #48 are still
required. #51 incorporates the fix by merging `ui/ci`; integration candidate
#56 must be rebuilt by its owner to include it.
- Review (latest, Mohamed, approved `cd5dffcf`): the zero-executed and malformed-result findings are resolved. No open finding.

### B11 #51 `ui/blockly-offline-media` (stacked on #48)

- The Programs page loads Blockly's sounds, sprites and cursors from the app
  (`/blockly-media/`, copied from the installed blockly package at build
  time) instead of `blockly-demo.appspot.com`.
- The route smoke test now blocks every external host and fails on any
  attempt to reach one.
- Builds on B9; merge after it.
- Evidence: Playwright smoke test with external hosts blocked.
- Review (latest, Mohamed, approved): no open finding. Head `edb57f31` adds only the merge of #48's current head.

### B10 #49 `ui/docs-ownership`

- README and docs: Stop (software) as it works on `main` today, the Getac
  ZX80, the data directory and maps location, the replay policy;
  AGENTS.md ownership; CODEOWNERS adds @parthkeshari; issue #19 updated.
- Merges after #43, #47 and #53.
- In this tree: the Stop (software) passages in README.md, docs/installation.md and lessons 00, 02, 06 and 07 carry #50's latched text (see A1); #49's docs consistency test accepts either wording.
- Review (latest, Mohamed, approved `2dbd1b04`): no open finding.

## A1 #50 `ui/gate-a-command-path` (part of #31, part of #34)

Base `main` at `0ca798b424cadbb77d6791f5298bb4f7e0f37a89`. Merge only after
openamr-platform-sw#90, the source of the interfaces below (head
`710c98bc3d384a3d0612e87ff5e8c0c00b804dc0`).

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
- To confirm (proposed defaults): 10 ms `/clock` throttle, 1 s stop lock
  staleness, 2 s retry for the relay parameter read.
- Stop (software) also latches locally at once (`motionGate` `localStop`),
  before the engage request answers; held input, programs, missions and
  goals stay blocked until a confirmed Resume whose release succeeds. A
  released lock sample or a reconnect does not clear it (review of #50).
- While the lock is engaged or unknown, no Nav2 goal is sent from the map
  page (goal, home, waypoints, queue, docking), missions, route planning or
  programs; the reason is shown (`shared/motion/navGoalGate.js`).
- README and docs/ describe the latched Stop (software), the explicit
  Resume and the disabled controls (`web/src/shared/motion/stopDocs.test.js`).
- Platform gap (openamr-platform-sw, owner Kunal): the `dock_trigger.py`
  docking sequence is not an action and cannot be cancelled by the UI.
- `.github/decisions.yaml` entries for the UI command topic and the stop
  lock interface: needed, handled separately in openAMRobot/.github.
- Evidence: unit and component tests (Jest, mocked ROSLIB), pure launch
  test. Not run in a browser, in simulation, on the Getac ZX80 or on a
  robot.

Open issues on 30 Oct: #19, #28, #29, #31, #32, #33, #34, #35, #36, #37,
#39. Open pull requests on 30 Oct: drafts #21 and #40 to #46, plus A1.
- In this tree: where #49 and #50 change the same Stop lines, #50's text is kept.
- Review (latest, Kunal, comment on `d873b7bd`): the local-latch P1 is confirmed closed. Open: openamr-platform-sw#90 merge, Nikolai's S4, S6 and S7a reruns, and the `dock_trigger.py` sequence the UI cannot cancel (platform item).

## A2 #54 `ui/reporting-mode` (part of #19)

Branched from A1 (`ui/gate-a-command-path`); merges after A1.

- Read-only, full-screen route `/report` for the detachable panel, outside
  the app layout. Header "Base and navigation". Rows for Arms, Lift and
  Physical E-stop show "Not reported".
- One consumer-neutral adapter, `web/src/shared/reporting/reportingAdapter.js`,
  fed by `reportingSource.js` (subscriptions only). States: DISCONNECTED,
  UNKNOWN, STOPPED (SOFTWARE), FAULTED, NEEDS_ASSISTANCE, RECOVERING,
  BLOCKED, EXECUTING, WAITING, READY. SAFETY_STOPPED is reserved and never
  derived. Thresholds, mapping rows and priority: `docs/reporting-mode.md`.
- Mission, program and route runs are browser-side (this tab only). The
  proposed robot-side envelope is `docs/proposals/i5-run-state.md`
  (Proposed, no shared message added).
- READY cannot appear on today's simulation: NavigationStatus stays
  NOT_READY with reason 9001 until a base status adapter exists.
- State priority (owner decision): a known, engaged software lock ranks
  above UNKNOWN navigation data; only DISCONNECTED ranks higher.
- Screen Wake Lock needs HTTPS and the panel uses HTTP on the robot network,
  so the ZX80 screen timeout is set to Never in Android display settings
  (docs/reporting-mode.md, Panel setup).
- `.github/decisions.yaml` entry for the Reporting Mode display vocabulary:
  handled separately in openAMRobot/.github.
- Thresholds (stale multipliers, 25% low battery) are accepted defaults
  (owner decision, 30 Oct 2026). Type sizes and the 2 s exit long press stay
  proposed until checked on the ZX80.
- `/diagnostics` validation (owner decision, 30 Oct 2026, from the #54
  review): status array required; each entry needs a name and a level 0 to
  3, invalid entries are ignored and logged; freshness is per entry name;
  READY needs a valid fresh entry and no source latched in fault.
- Diagnostics fault latch (owner decision, 30 Oct 2026, second #54 review
  point): an ERROR or STALE report latches that name in fault until the same
  name reports OK or WARN, or a new rosbridge connection starts (then
  WAITING until valid diagnostics arrive). The page shows FAULTED with the
  source name and the time of its last error.
- READY reads "No blocking faults in current reported data" (or "Ready,
  with reported warnings shown below"), not a claim that every check passes
  (review of #54). Fault times use the panel clock, so the ZX80 keeps
  "Automatic date and time" on (docs/reporting-mode.md, Panel setup).
- Evidence: unit, component and browser (Playwright at 1920 x 1200 against
  a scripted rosbridge stand-in). Not run against simulation, on the panel
  or on a robot.

Open issues on 30 Oct: #19, #28, #29, #31, #32, #33, #34, #35, #36, #37,
#39. Open pull requests on 30 Oct: drafts #21 and #40 to #46, #50 (A1) and
#54 (A2).
- Review (latest, Mohamed, approved `bf232917`): no open finding. Open: real rosbridge, simulation and ZX80 checks.

## C1 #55 `ui/upper-body-view` (read-only prototype)

- Robot Description page subscribes to `/robot_description` and `/joint_states` only. When the
  published URDF has `lift_joint` and OpenArm joints it shows that model under the banner
  "Candidate model, provisional geometry" (openamr-upperbody-sw PR #14, checked against 4629075).
- Configuration identity (column, base_plate, head_pitch, base_tilt, datums revision) shows
  "model or configuration unknown": no runtime producer exists (PR #14 known gaps). A reported
  hp25 would be flagged against the P-03 rev20.1 head pitch hp30.
- Freshness is tracked per joint from its measured publish rate. A group (lift, each arm, each
  gripper) is Updating only when every joint has a finite, matching position inside its own
  stale window; otherwise it is Partial or Stale with the missing, invalid or stale joints
  named. NaN, Infinity and unmatched positions are rejected. A ROS reconnect clears positions
  and sampling history (review of 2026-10-30 on e54753b).
- Lift panel: shoulder height from the reported lift_joint and the published model, against
  L1000, L1175 and L1350. Stop sensors: not reported until the I8 producer exists
  (openamrobot-interfaces #30).
- Camera panel: head camera is "not reported"; no contracted head camera topic exists.
- No publishers, services, actions or parameters; Jest tests fail if the page uses any.
- From #55's STATE.md: lift stops L1000, L1175, L1350 as calibration IDs; head_pitch hp30 per P-03 rev20.1.
- Known gap (#55): meshes of packages other than openamrobot_description are not served by the backend, so the combined model shows the arms as placeholder boxes.
- Not verified (#55): real robot, an accepted upper-body model, the Getac ZX80 panel, and rosbridge delivery of the transient-local `/robot_description` topic.
- Review (latest, Mohamed, approved `f69f3b94`): no open code finding; the published configuration ID stays unknown without a robot producer.

## Known gaps

- The Stop (software) lock path (A1, #50) is in this tree, but its platform
  side, openamr-platform-sw#90, is not merged. Without #90 the lock state
  stays unknown, so manual drive, programs, missions and new navigation
  goals stay disabled.
- `test_flake8.py`, `test_pep257.py` and `test_copyright.py` are skipped
  without a tracking issue, and most existing Python files are not flake8
  clean.
- The map migration (B13) logs its problems, and retries on every start
  while any map is skipped, in the node only; nothing is shown in the UI.

Open pull requests on 31 Oct: drafts #21 and #40 to #56. This candidate
merges #40 to #55; #21 is not included. None is merged to `main`.
