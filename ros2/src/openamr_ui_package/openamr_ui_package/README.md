# Python Package

This folder contains the Python ROS nodes for `openamr_ui_package`.

- `flask_app.py`: Flask server for the compiled React app.
- `map_relay.py`: `/map` to `/ui/map` QoS relay.
- `nav_relays.py`: AMCL and action-status relays.
- `folders_handler.py`: map, group, route, and waypoint file operations.
- `waypoint_nav.py`: optional waypoint route-following helper.
- `static/app/`: generated React production build copied from `web/build/`.
