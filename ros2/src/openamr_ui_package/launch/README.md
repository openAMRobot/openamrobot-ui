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
- `map_server_launch.py`: deprecated compatibility launch, namespaced under
  `ui_legacy`.
- No launch file here starts SLAM, AMCL, map_server or the Nav2 controller,
  planner or navigator; those belong to the robot platform
  (openamrobot-ui#32).

For normal use, see the root `README.md` and prefer
`openamr_ui_bringup ui.launch.py`.
