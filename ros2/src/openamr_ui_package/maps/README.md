# Maps

This folder holds the example maps shipped with the package. Each map normally
includes a ROS map YAML file and its image file. On first start the UI copies
them to its maps directory and from then on reads and saves maps only there;
a file that already exists there is never overwritten, so rebuilding or
reinstalling the package keeps saved maps.

The maps directory is `OPENAMROBOT_MAPS_DIR` if that environment variable is
set, otherwise `~/.openamr_ui/maps/`. Docker Compose sets it to `/data/maps`
on the `openamrobot_maps` volume, which the platform container mounts at the
same path so its map_server can load the maps the UI saves. Routes stay in
`~/.openamr_ui/paths/`.

The main Nav2 map server belongs to the robot or simulation workspace. This UI
workspace can store and manage map files, but it should not start a second map
server when the platform stack already owns `/map_server`.

Use the root `README.md` for the current build and run workflow.
