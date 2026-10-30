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

When `OPENAMROBOT_MAPS_DIR` is set on an installation whose
`~/.openamr_ui/maps/` already has maps, the UI copies those maps into it.
Nothing is moved, deleted or overwritten: the old folder stays as a backup.
A map is skipped, with its reason logged, when its image is outside the old
maps folder (an absolute path elsewhere, a `..` path or a symlink out), when
its image is missing or unreadable, or when its name already exists in the
new folder. While any map is skipped, the migration is not marked done
(marker `.maps_migrated`): it runs again on every start and logs each skipped
map until it is fixed or removed. If the selected map is skipped, the
selection stays on the old folder until then. Once nothing is skipped, the
marker is written and the migration does not run again.

The main Nav2 map server belongs to the robot or simulation workspace. This UI
workspace can store and manage map files, but it should not start a second map
server when the platform stack already owns `/map_server`.

Use the root `README.md` for the current build and run workflow.
