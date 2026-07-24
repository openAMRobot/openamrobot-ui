# Launch Files

Current launch files:

- `new_ui_launch.py`: main UI package launch. Starts Flask, rosbridge, rosapi,
  web video server, `map_relay`, and `nav_relay`. Also loads
  `../.env` (a package-scoped, gitignored file — see `../.env.example`) and
  injects its keys (e.g. `ANTHROPIC_API_KEY`) only into the `flask_app`
  node's process via `additional_env`, so LLM API keys don't need to be
  exported in the parent shell. If `.env` is missing, a `LogInfo` warning is
  printed and `/api/voice-plan` will return 500 until it's created.
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
