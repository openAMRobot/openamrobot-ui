# Launch Files

Current launch files:

- `new_ui_launch.py`: main UI package launch. Starts Flask, rosbridge, rosapi,
  web video server, `map_relay`, and `nav_relay`.
- `physnode_launch.py`: optional helper launch for map/route file operations and
  waypoint route following.
- `map_server_launch.py`: deprecated compatibility launch, namespaced under
  `ui_legacy`.

The other launch files are legacy or compatibility helpers. For normal use, see
the root `README.md` and prefer `openamr_ui_bringup ui.launch.py`.
