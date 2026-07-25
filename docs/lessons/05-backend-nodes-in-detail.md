# Lesson 05 — Backend Nodes in Detail

| Audience | Time | Prerequisites |
| --- | --- | --- |
| Backend developers and maintainers | 15 minutes | [Lesson 04](04-data-flow-and-relays.md) |

## What you'll learn

You will learn the responsibilities of the Flask server and optional helper
nodes, and why frontend assets must be built, synchronized, and installed.

[Lesson 04](04-data-flow-and-relays.md) covered two of the nodes in
[`ros2/src/openamr_ui_package/openamr_ui_package/`](../../ros2/src/openamr_ui_package/openamr_ui_package/) —
`map_relay.py` and `nav_relays.py`. This lesson covers the rest of what that
package runs: the web server itself, the two nodes behind the Route page's
file operations, and the battery node. Same idea as
[Lesson 07](07-ui-components.md) — one node at a time, what it does, and
anything non-obvious about it.

## flask_app.py — two unrelated jobs in one process

`flask_app.py` (executable `flask`, ROS node name `flask_app`) does two
things that have nothing to do with each other, bundled into one process
because they both need to run on the UI computer:

**Job 1 — serve the compiled React app.** `serve_spa()` is a catch-all route:
if the requested path matches a real file in the build output, it serves
that file; otherwise it always serves `index.html`. That's what makes deep
links like `/control` or `/blocks` work on a hard refresh — there's no
`/control` file on disk, React Router just takes over once `index.html`
loads and reads the URL itself.

Here's the detail that explains the single most common "I edited the code
and nothing changed" confusion: `REACT_BUILD_DIR` isn't `web/build/` — it's
`get_package_share_directory("openamr_ui_package")/app`, a path inside the
*installed* ROS package. That directory only gets populated by a three-step
pipeline: `npm run build` (or `scripts/build_frontend.sh`) compiles
`web/src` into `web/build/`; `scripts/sync_frontend_to_ros.sh` copies that
into
`ros2/src/openamr_ui_package/openamr_ui_package/static/app/`; then
`colcon build` installs *that* into the package share directory Flask
actually reads from. Editing `web/src` alone changes none of those three
copies — Flask keeps serving whatever was last installed. This is why every
frontend change in this workspace needs the full rebuild-and-reinstall flow
(see the
[development guide](../development.md#production-frontend-workflow)), and why `npm run
dev` exists as a separate path entirely: it serves straight from `web/src`
on port `3000`, bypassing this pipeline (and, as a result, Flask's REST API
and rosbridge's usual host-based connection logic) for fast iteration.

**Job 2 — a small REST API, unrelated to rosbridge.** Everything the Blocks
page's `Backend Programs`, `Named Locations`, `Run History`, and
`Voice Command` panels do goes through plain HTTP JSON endpoints
(`/api/block-programs`, `/api/block-locations`, `/api/block-run-history`,
`/api/voice-plan`) — not ROS topics, not rosbridge. This is a genuinely
different communication path from everything covered in
[Lesson 03](03-how-the-browser-talks-to-ros.md): the frontend code for this
lives in `web/src/features/blocks/backendPrograms.js`,
`backendLocations.js`, and `backendRunHistory.js`, calling `fetch()`
directly rather than `window.ROSLIB`. See
[Lesson 09](09-blockly-programming.md) for what those panels do.

`flask_app.py` is *also* a ROS node — the `ParamFlask` class declares
`appAddress`/`portApp` parameters, populated from
[`ros2/src/openamr_ui_package/param/config.yaml`](../../ros2/src/openamr_ui_package/param/config.yaml)
by the launch file — so its host and port are configured the same way every
other node's parameters are, even though Flask itself has nothing to do with
ROS messaging.

One more detail worth knowing: `flask_app.py` can optionally serve over
HTTPS if it finds a cert/key pair at `~/.openamr_ui/certs/`, falling back to
plain HTTP if they're missing. This is the actual mechanism behind Voice
Command's "secure origin" requirement mentioned in the
[Blockly guide](../../web/src/features/blocks/README.md#voice-command-requirements) —
browsers refuse microphone access on a plain-HTTP LAN address, so this is the
opt-in fix for that, not a separate system.

## folders_handler.py — the node behind the Route page

Executable `handler`, started only by
[`physnode_launch.py`](../../ros2/src/openamr_ui_package/launch/physnode_launch.py)
(the optional helper launch). The class defaults to ROS node name
`ui_folders` internally, but `physnode_launch.py` overrides that to
`folders_handler` — that's the name you'll actually see in `ros2 node
list`/`ros2 node info` when it's running. This is what actually reads and
writes map/route files when you use `Save`, `Rename`, `Delete`, `Change`, or
`Create` on the Route page.

It listens on one topic, `ui_operation`, but the messages on it aren't
free-form — they're a lightweight command format: the string before the
first `/` is a command name (`save_route`, `change_map`, `delete_group`, …),
and everything after it is a JSON payload. `RoutePage.jsx` builds these
strings and `folders_handler.py`'s `ui_callback()` dispatches on the command
name to one of ~14 handler methods. It's worth noticing that this is a
hand-rolled RPC-style pattern layered on top of a plain `std_msgs/String`
topic — not a ROS service ([Lesson 02](02-ros2-core-concepts.md)) — which
means there's no built-in request/response pairing or error reporting; the
node just publishes human-readable status strings back on `ui_message` and
the frontend's `Logs` panel is the only place those are visible.

Every map/route operation reads or updates one file,
`param/current_map_route.yaml`, which tracks which map and route are
*currently active* — see [Lesson 08](08-map-and-route-model.md) for the full
file model this node manages.

Beyond route file CRUD, this same node also has mapping-mode functions
(`build_map_func`, `save_map_func`) that shut down AMCL/map_server/move_base
lifecycle-wise and launch a separate mapping launch file. These are what the
Maps page's **Start mapping**/**Save current map** buttons actually trigger
(see [Lesson 06](06-the-pages.md#maps--mapspagejsx)) — for a long time this
was functionality present in the node with no button wired to it; it has one
now.

## waypoint_nav.py — a second subscriber on the same topic

Executable `nav`, also started only by `physnode_launch.py`. The class
defaults to ROS node name `Way_points_handler` internally, but
`physnode_launch.py` overrides that to `waypoint_nav` — same pattern as
`folders_handler.py` above. This is the optional route-*following* helper — it
reads the same active route CSV and drives the robot through it point by
point using Nav2's `BasicNavigator` (`nav2_simple_commander`) directly,
rather than publishing to `/goal_pose` the way every panel covered in
[Lesson 07](07-ui-components.md) does. That's a second, independent way this
codebase commands Nav2 — worth knowing if you're tracing "why did the robot
just start moving."

The subtle part: `waypoint_nav.py` *also* subscribes to `ui_operation` —
same topic name as `folders_handler.py` above — but recognizes a completely
different, non-overlapping vocabulary: plain strings like `follow_route`,
`next_point`, `previous_point`, `home`, `stop`, with no JSON payload. Both
nodes get every message published on `ui_operation`; each one simply ignores
strings it doesn't recognize. Two nodes sharing one topic name only works
safely because their command vocabularies never collide — if you're adding a
new `ui_operation` command (for either node), check both files, not just the
one you're editing.

## battery.py — no launch path exists yet

This one is more than "off by default": there is currently no way to start
it at all short of writing your own launch entry or `rclpy` script.
`setup.py`'s `console_scripts` registers `flask`, `handler`, `nav`,
`map_relay`, and `nav_relay` — `battery.py`'s `main()` isn't one of them, so
`ros2 run openamr_ui_package battery` doesn't exist. It also has no
`if __name__ == "__main__":` guard at the bottom of the file, so running
`python3 battery.py` directly does nothing either — `main()` is defined but
never called. If it were wired up, it would read a serial port
(`/dev/ttyUSB0`) for battery data and publish `Float32` on `battery_status`;
absent the serial device (the common case off a real battery-equipped
robot), it falls back to a simulated battery that slowly drains from 100,
purely so the Status page's battery panel
([Lesson 06](06-the-pages.md#status--infopagejsx)) has *something* to show
during development. This is the concrete mechanism behind that page's "no
battery data just means no node is publishing it" behavior — currently,
that's every deployment, since nothing starts this node yet.

## Try it

From the launch files, list which nodes start during normal UI bringup and
which require `physnode_launch.py`. Then locate where the installed React
bundle is served from.

**You're ready to continue when:** you can predict which page features remain
available when the optional map/route helpers are not running.

## Next

[Lesson 06 — A Tour of Every Page](06-the-pages.md) picks the frontend side
back up, covering what each page shows and which of these backend nodes and
topics it depends on.

---

[← Lesson 04](04-data-flow-and-relays.md) · [Lesson index](README.md) ·
[Next: Lesson 06 →](06-the-pages.md)
