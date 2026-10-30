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
- `mapping_launch.py` (includes `gmapping_launch.py` + `move_base_launch.py`)
  and `navigation_launch.py` (includes `move_base_launch.py` +
  `amcl_launch.py`): legacy. They start a second SLAM, AMCL and Nav2
  controller stack next to the platform's own (openamrobot-ui#32). Do not
  launch them. They stay only while `folders_handler.py` still names them and
  are to be deleted, with `../param/move_base/`, once no code references them.
  `amcl_launch.py`, `gmapping_launch.py` and `move_base_launch.py` are only
  the pieces those two include.
- The platform owns `map_server`; no launch file here starts one.

For normal use, see the root `README.md` and prefer
`openamr_ui_bringup ui.launch.py`.
