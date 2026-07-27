# Submodules

Shared Python helpers for ROS nodes go here when multiple nodes need the same
logic. Currently holds one file, `nodechecker.py` — a small
`map_server_check()` helper that greps `ros2 node list` for
`/lifecycle_manager` — but nothing in this package imports it yet, so treat
it as a starting point rather than active, wired-in logic.
