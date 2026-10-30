# Maps

This folder holds the example maps shipped with the package. Each map normally
includes a ROS map YAML file and its image file. On first start the UI copies
them to its maps directory and from then on reads and saves maps only there;
a file that already exists there is never overwritten, so rebuilding or
reinstalling the package keeps saved maps.

The maps directory is `OPENAMROBOT_MAPS_DIR` if that environment variable is
set, otherwise `~/.openamr_ui/maps/`. Docker Compose sets it to `/data/maps`
on the `openamrobot_maps` volume. For the platform map_server to load the
maps the UI saves, the platform must mount the same `openamrobot_maps` volume
at `/data/maps`; that is a pending platform-sw change. Routes stay in
`~/.openamr_ui/paths/`.

When `OPENAMROBOT_MAPS_DIR` is first set on an installation whose
`~/.openamr_ui/maps/` already has maps, the UI copies those maps into it once
(marker `.maps_migrated`). Nothing is moved or deleted: the old folder stays
as a backup. A map whose name already exists in the new folder is skipped and
logged, and both copies are kept. A map whose image is outside the old maps
folder (an absolute path elsewhere, a `..` path or a symlink out) is skipped
and logged too; it stays only in the old folder. Skipped maps are not retried
on later starts. If the currently selected map is one whose name already
exists in the new folder, the migration is blocked for it: the selection stays
on the old folder, an error says which map collides, and the migration runs
again on every start until one of the two maps is renamed or removed.
A map whose image is missing or unreadable is not migrated either, and the
migration is not marked done: restore the image and restart, and the map is
copied then. If that map is the selected one, the selection stays on the old
folder until then.

The main Nav2 map server belongs to the robot or simulation workspace. This UI
workspace can store and manage map files, but it should not start a second map
server when the platform stack already owns `/map_server`.

Use the root `README.md` for the current build and run workflow.
