# Parameters

This folder contains ROS parameter files for the UI package.

- `config.yaml`: active UI service configuration, including Flask, rosbridge,
  and web video server ports.
- `current_map_route.yaml`: tracks the active map and route used by helper
  nodes.
- `move_base/`: legacy parameters, referenced only by the legacy
  `launch/move_base_launch.py`. Delete together with that launch file.

The root `README.md` documents the current ports and launch flow.
