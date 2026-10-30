# Launch Files

Current launch files:

- `new_ui_launch.py`: main UI package launch. Starts Flask, rosbridge, rosapi,
  web video server, `map_relay`, and `nav_relay`. For a manual launch it can
  load `../.env` (a package-scoped, gitignored file — see
  `../.env.example`) and inject those values only into `flask_app` through
  `additional_env`. It also accepts `ANTHROPIC_API_KEY` from the parent
  process, which is the recommended Docker Compose path. Real `.env` files
  are excluded from Docker build contexts so API keys are not baked into
  images. If neither source provides a key, `/api/voice-plan` returns 500 and
  the launch logs explain how to configure it.
- `physnode_launch.py`: optional helper launch for map/route file operations and
  waypoint route following.
- No launch file here starts SLAM, AMCL, the Nav2 controller, planner or
  navigator, or a map_server; those belong to the robot platform
  (openamrobot-ui#32). The legacy mapping and navigation launches
  (openamrobot-ui#52) and `map_server_launch.py` (openamrobot-ui#41) are
  deleted.

For normal use, see the root `README.md` and prefer
`openamr_ui_bringup ui.launch.py`.
