# Lesson 08 — The Map and Route File Model

[Lesson 07](07-ui-components.md) covered the Route page's modals as UI
pieces. This lesson covers the file model underneath them — the concept that
makes the Route page's `Group`/`Map`/`Route` header make sense, and the one
most likely to confuse someone debugging "why does my saved route not show
up." It's owned entirely by
[`folders_handler.py`](05-backend-nodes-in-detail.md#folders_handlerpy--the-node-behind-the-route-page)
on the ROS side.

## Three levels: group → map → route

Everything is organized in a fixed three-level hierarchy:

```text
Group        (e.g. "Warehouse", "Welcome")
  Map          (e.g. "FloorA", "Start" — one saved occupancy grid)
    Route        (e.g. "MorningPatrol" — one saved waypoint sequence)
```

A **group** is a folder-level container — typically a building or site. A
**map** is one saved occupancy grid inside a group — typically one floor or
room layout. A **route** is one saved, named sequence of waypoints that only
makes sense for the specific map it was drawn on. This is the concept behind
[Lesson 07](07-ui-components.md#the-route-page-modals)'s note that a route
drawn for one map isn't valid on another — the hierarchy enforces that by
construction: routes physically live *inside* their map's folder, not
alongside it.

## Where the files actually live

```text
ros2/src/openamr_ui_package/maps/<group>/<map>.yaml       # standard ROS map_server YAML
ros2/src/openamr_ui_package/maps/<group>/<map>.png        # the occupancy grid image
ros2/src/openamr_ui_package/maps/<group>/<map>_ros.yaml   # per-map launch parameter override
ros2/src/openamr_ui_package/paths/<group>/<map>/<route>.csv
```

See [`maps/README.md`](../../ros2/src/openamr_ui_package/maps/README.md) and
[`paths/README.md`](../../ros2/src/openamr_ui_package/paths/README.md) for
the short folder-level notes. Renaming a map (`folders_handler.py`'s
`rename_map_func`) has to touch all three map files *and* move the matching
route folder — a good illustration of why this is a dedicated backend node
rather than the frontend just calling `os.rename` through some generic file
API: keeping four related paths in sync is exactly the kind of bookkeeping
you want one owner responsible for.

Each row in a route CSV is one waypoint: position (`x,y,z`), orientation as
a quaternion (`x,y,z,w`), three reserved numeric fields, and a trailing
"purpose" marker — eleven comma-separated values in total. The Route page
and `folders_handler.py` are the only things that need to agree on this
exact layout; nothing else in the UI reads route CSVs directly.

## The active context: one file, always current

Only one map and one route can be "active" at a time — the ones currently
shown on the Route page and driven by
[`waypoint_nav.py`](05-backend-nodes-in-detail.md#waypoint_navpy--a-second-subscriber-on-the-same-topic).
That state lives in exactly one file:

```text
ros2/src/openamr_ui_package/param/current_map_route.yaml
```

with two keys, `map_file` and `route_file`, each holding a full filesystem
path. Every operation — `Change`, `Save`, `Delete`, opening the Route page at
all — reads or rewrites this one file. The `Group`/`Map`/`Route` names shown
in the Route page header aren't stored anywhere separately; they're derived
by splitting the last two or three segments off whichever full path is
currently in this file (see `get_paths()` in `folders_handler.py`).

Worth knowing if a map or route keeps unexpectedly resetting: on startup,
`folders_handler.py` checks whether the stored `map_file`/`route_file` paths
still exist on disk, and also resets them if the path contains the literal
string `"darkadius"` — a legacy check left over from a stale absolute path
baked in from a different development machine. Either condition falls back
to a built-in default (`Welcome/Start`). If your active map/route keeps
reverting to `Welcome/Start` for no obvious reason, check
`current_map_route.yaml` for a stale or foreign absolute path before
assuming something else is broken.

## Next

[Lesson 09 — Blockly Visual Programming](09-blockly-programming.md) is the
deep dive on Blocks, the same way Lesson 07 was the deep dive on
the components behind Map and Route.
