# OpenAMR UI Package

This ROS 2 package contains the Flask server, rosbridge launch integration,
camera web streaming launch integration, map/navigation relays, and optional
map/route helper nodes used by the OpenAMR browser UI.

The top-level `../../../README.md` is the source of truth for workspace setup,
build commands, run commands, ports, topics, and troubleshooting.

## Launch Files

- `launch/new_ui_launch.py`: starts the web UI services and browser-friendly
  relays.
- `launch/physnode_launch.py`: starts optional map/route file-operation and
  waypoint route-following helper nodes.
- `launch/map_server_launch.py`: deprecated compatibility launch, namespaced
  under `ui_legacy` to avoid conflicting with the platform map server.
- The remaining launch files are legacy or compatibility launch helpers.

## Runtime Pieces

- `flask_app.py`: serves the compiled React app.
- `map_relay.py`: republishes `/map` to `/ui/map` with browser-friendly QoS.
- `nav_relays.py`: republishes AMCL and navigation/docking action status under
  `/ui/*`.
- `folders_handler.py`: handles map, group, route, and waypoint file commands.
- `waypoint_nav.py`: optional route-following helper using Nav2 Simple
  Commander.

## Static App

The React source lives in the repository-level `web/` directory. Production
assets are built and copied into:

```text
openamr_ui_package/static/app/
```

Use the repository-level scripts:

```bash
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
```
