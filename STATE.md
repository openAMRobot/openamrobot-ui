# STATE: openamrobot-ui

Working state of this repository for contributors and agents. Read it before
starting work; update it in the same pull request (AGENTS.md, Decisions).

Last updated: 30 Oct 2026, base `main` at
`0ca798b424cadbb77d6791f5298bb4f7e0f37a89`.

This file currently covers only the backend and process items for
v2.0.0-rc.1 listed below (B1 to B10). Other work in this repository, such as
the UI command path and Reporting Mode, is tracked by its own owners and
pull requests and is not described here.

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

### B13 `ui/shared-maps-volume` (branched from `ui/persistent-maps-routes`)

- Maps go to `OPENAMROBOT_MAPS_DIR` (default: `maps` in the data
  directory); routes stay in the data directory.
- `docker-compose.yml` sets it to `/data/maps` on the named volume
  `openamrobot_maps`.
- One-time, non-destructive migration: when `OPENAMROBOT_MAPS_DIR` differs
  from the data directory's `maps` folder and that folder has maps, they are
  copied (never moved or deleted; collisions skipped and logged), the
  selection is repointed when its map was copied, and `.maps_migrated` is
  written.
- A map whose image resolves outside the old maps folder (absolute path
  elsewhere, `..`, or a symlink out) is skipped and logged with the reason;
  nothing of it is copied, and the old folder, the new folder and the outside
  file stay unchanged. If it is the selected map, the selection stays on the
  old folder and is logged. Skipped maps are not retried after
  `.maps_migrated` is written; the old folder keeps them (review of #53).
- Still open from the review of #53: a colliding selected map is only logged,
  not reported as a migration blocker.
- Open, outside this repository: the platform compose must mount the same
  `openamrobot_maps` volume at `/data/maps` (a pending platform-sw change).
- Evidence: unit tests (pure and rclpy) and a compose file check.

### B9 `ui/ci`

- CI gains a `ros` job (ros:jazzy container, colcon build and test, fails on
  zero tests) and an `e2e` job (Playwright, every page in Demo Mode at
  1280 x 800 and 1920 x 1200, no console errors).
- Evidence: job steps reproduced locally; the jobs themselves run on the
  pull request.

### B10 `ui/docs-ownership`

- README and docs: Stop (software), Getac ZX80, data directory, replay
  policy; AGENTS.md ownership line; CODEOWNERS adds @parthkeshari; issue #19
  body updated.
- Merge after B4 and B8, whose behaviour it documents.
- Evidence: Jest docs consistency test.

## Known gaps

- The Stop (software) lock is not on `main` yet and is not documented here;
  it belongs to the UI command path work.
- The Blocks page loads Blockly sounds and sprites from
  `blockly-demo.appspot.com`, which an offline robot cannot reach.
- `test_flake8.py`, `test_pep257.py` and `test_copyright.py` are skipped
  without a tracking issue, and most existing Python files are not flake8
  clean.
