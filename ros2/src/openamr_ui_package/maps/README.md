# Maps

This folder holds the example maps shipped with the package. Each map normally
includes a ROS map YAML file and its image file. On first start the UI copies
them to `~/.openamr_ui/maps/`, the same data directory as recordings, and from
then on reads and saves maps only there; a file that already exists there is
never overwritten, so rebuilding or reinstalling the package keeps saved maps.

The main Nav2 map server belongs to the robot or simulation workspace. This UI
workspace can store and manage map files, but it should not start a second map
server when the platform stack already owns `/map_server`.

Use the root `README.md` for the current build and run workflow.
