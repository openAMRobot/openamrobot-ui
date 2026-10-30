# STATE: openamrobot-ui

Working state of this repository for contributors and agents. Read it before
starting work; update it in the same pull request (AGENTS.md, Decisions).

Last updated: 30 Oct 2026. This is the reconciled state for the backend and
process items for v2.0.0-rc.1 (B1 to B13). It describes the repository after
#43, #47 and #53, which merge before #49 (owner decision, 30 Oct). Items in
other pull requests still open are named with their PR. Other work, such as
the UI command path, the Stop (software) lock (#50) and Reporting Mode, is
tracked by its own owners and pull requests and is not described here.

## Ownership

Decision of 19 Oct: Parth Keshari (UI lead, Reporting Mode owner); Mohamed
Sayed and ND (reviewers); Nikolai (simulation checks); Kunal (platform
side). Every item below is a draft pull request until Parth writes adopt,
adapt or reject in its thread.

## Accepted defaults (owner decision, 30 Oct)

- Replay telemetry allowlist in `param/replay_topics.yaml`.
- Voice plan limits: 0.2 m/s, 2.0 rad/s, 5 m and 30 s per action.
- `waypoint_timeout_s` 300 s; `pose_stale_after_s` 2.0 s.
- Platform map save service `/map_saver/save_map`.
- Maps on the shared `openamrobot_maps` volume at `/data/maps`.

## Items

None of these items has been run in simulation, on the Getac ZX80 or on a
robot. Unit tests and fixtures are not safety evidence.

### B1 #40 `ui/no-ui-nav-launch` (part of #32)

- Maps actions run no `ros2 launch`, `ros2 run` or `ros2 lifecycle` command
  and publish no `/initialpose`. Save calls the platform map saver, then
  `/map_server/load_map`; failures reach the UI as clear messages.
- Rename and delete work for maps saved by SaveMap (`<name>.yaml` and the
  image); a legacy `<name>_ros.yaml` is handled when it exists.
- Open: the platform-side mount and launch (platform-sw, Kunal) and a
  simulation check of save, list, load and reload (Nikolai).

### B2 #41 `ui/remove-legacy-launch` (part of #32)

- Deletes the unused `launch/map_server_launch.py`.

### B12 #52 `ui/remove-nav-launches` (part of #32, stacked on #40)

- Deletes `mapping_launch.py`, `navigation_launch.py`, `gmapping_launch.py`,
  `move_base_launch.py`, `amcl_launch.py` and `param/move_base/`.
- Merge order: #40, then #41, then #52.

### B3 #42 `ui/lifecycle-read-only`

- The lifecycle panel only calls `GetState`; it has no transition buttons.

### B4 #43 `ui/replay-policy` (part of #33)

- Replay publishes only topics on the telemetry allowlist, passed to
  `ros2 bag play --topics`.
- With no topic list, a replay plays the recording's topics that are on the
  allowlist and reports every other recording topic as skipped, with the
  reason.
- HTTP 400 is returned only when a request explicitly names a denied topic
  (`/cmd_vel`, `/ui/cmd_vel`, any `cmd_vel*`, `/tf`, `/tf_static`,
  `/goal_pose`, `/initialpose`, `/safety/*`, action or service topics).
  Nothing left to play is 409, a malformed topic list 422, an unreadable
  recording 500. No process starts in any of these cases.

### B5 #44 `ui/bag-process-recovery`

- ros2 bag processes are recorded in `~/.openamr_ui/recordings/bag_processes.json`
  (pid, start time, command line) under a thread and fcntl lock; a leftover
  is stopped on server start only when all three match.

### B6 #45 `ui/voice-bounds` (part of #35)

- `GET /api/limits` serves `param/voice_limits.yaml`; `/api/voice-plan`
  rejects (HTTP 422) non-finite or over-limit plans.

### B7 #46 `ui/route-outcomes` (part of #36)

- Route following reports SUCCEEDED, ABORTED, CANCELED, TIMEOUT or UNKNOWN
  per waypoint; a route succeeds only if every waypoint did.
- The AMCL pose relay repeats a pose only while the map to odom transform is
  younger than 2.0 s, timed with a monotonic clock, and drops it on a ROS
  clock reset.
- Open: simulation evidence for real Nav2 outcomes and a localization reset.

### B8 #47 `ui/persistent-maps-routes` (part of #37)

- Routes and `current_map_route.yaml` live in the data directory
  `~/.openamr_ui` (Docker: the `openamr_ui_data` volume).
- Shipped examples are seeded once per data directory (`.seeded` marker);
  every seeded file is created atomically and never overwrites.

### B13 #53 `ui/shared-maps-volume` (part of #37)

- Maps live in `OPENAMROBOT_MAPS_DIR`, default `~/.openamr_ui/maps`. Docker
  Compose sets it to `/data/maps` on the named volume `openamrobot_maps`;
  routes stay in the data directory.
- Once, on upgrade, maps in `~/.openamr_ui/maps` are copied into
  `OPENAMROBOT_MAPS_DIR` without moving, deleting or overwriting anything
  (`.maps_migrated` marker).
- Open: the platform must mount the same `openamrobot_maps` volume at
  `/data/maps` (pending platform-sw change, Kunal).

### B9 #48 `ui/ci`

- CI `ros` job (ros:jazzy, colcon build and test, fails on zero tests) and
  `e2e` job (Playwright, every page in Demo Mode at 1280 x 800 and
  1920 x 1200).

### B11 #51 `ui/blockly-offline-media` (stacked on #48)

- Blockly media is served by the app; the smoke test blocks every external
  host.

### B10 #49 `ui/docs-ownership`

- README and docs: Stop (software) as it works on `main` today, the Getac
  ZX80, the data directory and maps location, the replay policy;
  AGENTS.md ownership; CODEOWNERS adds @parthkeshari; issue #19 updated.
- Merges after #43, #47 and #53.

## Known gaps

- Stop (software) on `main` is a one-shot software stop that does not latch.
  #50 replaces it with a latched Stop (software) and an explicit Resume and
  updates the docs; until #50 merges, the docs describe the one-shot stop.
- `test_flake8.py`, `test_pep257.py` and `test_copyright.py` are skipped
  without a tracking issue, and most existing Python files are not flake8
  clean.
