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
- `mapping_launch.py` (includes `gmapping_launch.py` + `move_base_launch.py`)
  and `navigation_launch.py` (includes `move_base_launch.py` +
  `amcl_launch.py`): **not legacy** — `folders_handler.py`'s `build_map_func`/
  `save_map_func` launch these directly, and they're what the web UI's Maps
  page **Start mapping**/**Save current map** buttons actually trigger.
  `amcl_launch.py`, `gmapping_launch.py`, and `move_base_launch.py` on their
  own are just the pieces those two compose — not meant to be launched
  standalone.

For normal use, see the root `README.md` and prefer
`openamr_ui_bringup ui.launch.py`.
