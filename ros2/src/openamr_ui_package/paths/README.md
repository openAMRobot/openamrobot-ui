# Routes

This folder holds the example route CSV files shipped with the package. On
first start the UI copies them to `~/.openamr_ui/paths/`, the same data
directory as recordings, and from then on reads and saves routes only there;
a file that already exists there is never overwritten.

Routes are organized by map group and map name. Route files are read and written
by the optional helper nodes launched from `launch/physnode_launch.py`.

Use the route page in the web UI for normal route creation, editing, selection,
renaming, and deletion.
