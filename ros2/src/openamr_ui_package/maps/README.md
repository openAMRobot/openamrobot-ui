# Maps

This folder stores UI-managed map groups. Each map normally includes a ROS map
YAML file and its image file.

The main Nav2 map server belongs to the robot or simulation workspace. This UI
workspace can store and manage map files, but it should not start a second map
server when the platform stack already owns `/map_server`.

Use the root `README.md` for the current build and run workflow.
