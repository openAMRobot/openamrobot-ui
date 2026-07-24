import os
import json
import re
import signal
import subprocess
import time
import ipaddress
import urllib.request
import urllib.error
from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from flask import Flask, abort, jsonify, request, send_file, send_from_directory
from ament_index_python.packages import get_package_share_directory
from werkzeug.exceptions import HTTPException

import xacro

# ─────────────────────────────────────────────────────────────────────────
# AUTH_MODE — open-source access model
#
# This UI must stay fully usable with zero authentication out of the box
# (open-source, self-hosted, offline-first — see project design notes).
# AUTH_MODE is an opt-in deployment switch for maintainers who need to lock
# down a shared/classroom/lab robot, not a feature this app forces on
# anyone:
#   open     - default. No login. (only implemented mode right now)
#   local    - backend-validated credentials/sessions for shared installs.
#              NOT IMPLEMENTED YET: there is no user store, session/cookie
#              handling, or password hashing in this codebase. Requesting
#              it does not silently no-op into "unprotected" — it falls
#              back to open and says so loudly (see AUTH_MODE_WARNING),
#              because a maintainer believing they enabled auth when they
#              didn't is worse than the current, honest "wide open" state.
#   external - future OIDC/OAuth or authenticated-reverse-proxy support.
#              NOT IMPLEMENTED YET, same reasoning as local.
# Whichever mode is eventually real, it must be enforced here in the
# backend — hiding frontend routes/buttons is never authorization.
# ─────────────────────────────────────────────────────────────────────────
VALID_AUTH_MODES = {"open", "local", "external"}
IMPLEMENTED_AUTH_MODES = {"open"}

REQUESTED_AUTH_MODE = os.environ.get("AUTH_MODE", "open").strip().lower()
if REQUESTED_AUTH_MODE not in VALID_AUTH_MODES:
    print(
        f"[openamr_ui] WARNING: unknown AUTH_MODE={REQUESTED_AUTH_MODE!r}; "
        "falling back to 'open'. Valid values: open, local, external.",
        flush=True,
    )
    REQUESTED_AUTH_MODE = "open"

if REQUESTED_AUTH_MODE in IMPLEMENTED_AUTH_MODES:
    AUTH_MODE = REQUESTED_AUTH_MODE
    AUTH_MODE_WARNING = None
else:
    AUTH_MODE = "open"
    AUTH_MODE_WARNING = (
        f"AUTH_MODE={REQUESTED_AUTH_MODE!r} was requested but is not implemented "
        "in this version yet — running with AUTH_MODE=open (no authentication)."
    )
    print(f"[openamr_ui] WARNING: {AUTH_MODE_WARNING}", flush=True)


def is_local_address(addr):
    """True if addr is a loopback/private/link-local IP (i.e. "this network"),
    used only to warn operators in AUTH_MODE=open — not an access control."""
    if not addr:
        return False
    try:
        ip = ipaddress.ip_address(addr)
    except ValueError:
        return False
    if isinstance(ip, ipaddress.IPv6Address) and ip.ipv4_mapped:
        ip = ip.ipv4_mapped
    return bool(ip.is_private or ip.is_loopback or ip.is_link_local)

# React build is installed to share/openamr_ui_package/static/app/ by setup.py
SHARE_DIR = get_package_share_directory("openamr_ui_package")
REACT_BUILD_DIR = os.path.join(SHARE_DIR, "app")
REACT_STATIC_DIR = os.path.join(REACT_BUILD_DIR, "static")
REACT_ROS_DIR = os.path.join(REACT_BUILD_DIR, "ros")

# Vendored URDF/Xacro + meshes for the Robot Description page, installed to
# share/openamr_ui_package/robot_description/openamrobot/ by setup.py.
# Source of truth: openAMRobot/openamr-platform-sw's openamrobot_description
# ROS package. Kept as a data subfolder here (not a second ROS package of the
# same name) so this UI workspace never collides with the real robot-software
# workspace if the two are ever colcon-built together.
ROBOT_DESC_NAME = "openamrobot"
ROBOT_DESC_DIR = os.path.join(SHARE_DIR, "robot_description", ROBOT_DESC_NAME)
ROBOT_DESC_XACRO = os.path.join(ROBOT_DESC_DIR, "urdf", "robo_urdf.urdf.xacro")
_robot_urdf_cache = {"mtime": None, "xml": None}

# Make Flask serve /static/* from the CRA build folder
app = Flask(__name__, static_folder=REACT_STATIC_DIR, static_url_path="/static")

PROGRAM_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _.-]{0,63}$")
BLOCK_PROGRAMS_DIR = os.path.join(
    os.path.expanduser("~"), ".openamr_ui", "block_programs"
)
BLOCK_LOCATIONS_FILE = os.path.join(
    os.path.expanduser("~"), ".openamr_ui", "block_locations.json"
)
BLOCK_RUN_HISTORY_FILE = os.path.join(
    os.path.expanduser("~"), ".openamr_ui", "block_run_history.json"
)
RECORDING_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _.-]{0,63}$")
RECORDINGS_DIR = os.path.join(os.path.expanduser("~"), ".openamr_ui", "recordings")
RECORDINGS_INDEX_FILE = os.path.join(RECORDINGS_DIR, "index.json")
TOPIC_NAME_RE = re.compile(r"^/[A-Za-z0-9_/]{1,255}$")

# In-process only — this Flask server runs as a single Werkzeug process
# (threaded=True, not multi-worker), so module-level state is safe here.
# Neither survives a Flask restart: the real `ros2 bag` OS process would
# keep running orphaned if that happened mid-recording/replay (flagged,
# not solved — see _reconcile_recordings_on_startup below).
_recording = {"proc": None, "id": None, "name": None, "started_at": None, "topics": None}
_replay = {"proc": None, "id": None, "started_at": None, "paused": False, "rate": 1.0}
DEFAULT_BLOCK_LOCATIONS = {
    "Home": {"x": 0, "y": 0, "yaw": 0},
    "Charging Station": {"x": 0.5, "y": 0, "yaw": 0},
    "Pickup Point": {"x": 2, "y": 1, "yaw": 1.57},
    "Dropoff Point": {"x": 0, "y": 2, "yaw": 3.14},
}

# Mirrors web/src/shared/constants/index.js AppConfig so the model doesn't
# propose speeds the UI's own plan validation will just reject.
MAX_LINEAR_SPEED = 0.2
MAX_ANGULAR_SPEED = 2

ANTHROPIC_API_URL = "https://api.anthropic.com/v1/messages"
ANTHROPIC_MODEL = "claude-sonnet-5"
ANTHROPIC_VERSION = "2023-06-01"

ACTION_TYPES = {
    "navigate",
    "navigate_named",
    "wait",
    "set_speed",
    "drive_for",
    "rotate_for",
    "stop_movement",
    "wait_nav_complete",
    "repeat",
    "battery_below",
    "log",
    "set_mode",
    "dock",
    "undock",
    "stop",
}

# JSON Schema for the tool Claude must call. Mirrors the action union already
# implemented client-side in web/src/features/blocks/blockDefinitions.js
# (blockToAction / planToWorkspace).
ACTION_SCHEMA = {
    "$defs": {
        "action": {
            "oneOf": [
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "navigate"},
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "yaw": {"type": "number", "description": "radians"},
                    },
                    "required": ["type", "x", "y", "yaw"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "navigate_named"},
                        "location": {"type": "string"},
                    },
                    "required": ["type", "location"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "wait"},
                        "seconds": {"type": "number"},
                    },
                    "required": ["type", "seconds"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "set_speed"},
                        "linear": {"type": "number"},
                        "angular": {"type": "number"},
                    },
                    "required": ["type", "linear", "angular"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "drive_for"},
                        "linear": {"type": "number"},
                        "seconds": {"type": "number"},
                    },
                    "required": ["type", "linear", "seconds"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "rotate_for"},
                        "angular": {"type": "number"},
                        "seconds": {"type": "number"},
                    },
                    "required": ["type", "angular", "seconds"],
                },
                {
                    "type": "object",
                    "properties": {"type": {"const": "stop_movement"}},
                    "required": ["type"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "wait_nav_complete"},
                        "timeout": {"type": "number"},
                    },
                    "required": ["type", "timeout"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "repeat"},
                        "times": {"type": "integer", "minimum": 1},
                        "actions": {
                            "type": "array",
                            "items": {"$ref": "#/$defs/action"},
                        },
                    },
                    "required": ["type", "times", "actions"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "battery_below"},
                        "percent": {"type": "number"},
                        "actions": {
                            "type": "array",
                            "items": {"$ref": "#/$defs/action"},
                        },
                    },
                    "required": ["type", "percent", "actions"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "log"},
                        "message": {"type": "string"},
                    },
                    "required": ["type", "message"],
                },
                {
                    "type": "object",
                    "properties": {
                        "type": {"const": "set_mode"},
                        "mode": {"enum": ["autonomous", "manual", "idle"]},
                    },
                    "required": ["type", "mode"],
                },
                {
                    "type": "object",
                    "properties": {"type": {"const": "dock"}},
                    "required": ["type"],
                },
                {
                    "type": "object",
                    "properties": {"type": {"const": "undock"}},
                    "required": ["type"],
                },
                {
                    "type": "object",
                    "properties": {"type": {"const": "stop"}},
                    "required": ["type"],
                },
            ]
        }
    },
    "type": "object",
    "properties": {
        "actions": {"type": "array", "items": {"$ref": "#/$defs/action"}},
    },
    "required": ["actions"],
}


def build_voice_plan_system_prompt(locations):
    location_names = ", ".join(sorted(locations.keys())) or "(none saved yet)"
    return (
        "You translate a spoken command for a mobile robot into a structured "
        "action plan by calling the build_robot_plan tool. Only use the action "
        "types defined in the tool schema. Prefer navigate_named over navigate "
        "when the command refers to one of the known named locations: "
        f"{location_names}. Keep set_speed/drive_for linear speeds within "
        f"+/-{MAX_LINEAR_SPEED} m/s and set_speed/rotate_for angular speeds "
        f"within +/-{MAX_ANGULAR_SPEED} rad/s. If the command is ambiguous or "
        "unsafe, produce the closest reasonable safe interpretation rather than "
        "refusing. Do not add actions the command didn't ask for."
    )


def call_anthropic_voice_plan(transcript, locations):
    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        abort(500, "ANTHROPIC_API_KEY is not set on the server.")

    payload = {
        "model": ANTHROPIC_MODEL,
        "max_tokens": 2048,
        "system": build_voice_plan_system_prompt(locations),
        "messages": [{"role": "user", "content": transcript}],
        "tools": [
            {
                "name": "build_robot_plan",
                "description": "Return the sequence of robot actions for the spoken command.",
                "input_schema": ACTION_SCHEMA,
            }
        ],
        "tool_choice": {"type": "tool", "name": "build_robot_plan"},
    }

    request_obj = urllib.request.Request(
        ANTHROPIC_API_URL,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={
            "Content-Type": "application/json",
            "x-api-key": api_key,
            "anthropic-version": ANTHROPIC_VERSION,
        },
    )

    try:
        with urllib.request.urlopen(request_obj, timeout=30) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", errors="ignore")
        abort(502, f"Claude API request failed ({error.code}): {detail[:200]}")
    except urllib.error.URLError as error:
        abort(502, f"Could not reach Claude API: {error.reason}")

    for block in body.get("content", []):
        if block.get("type") == "tool_use" and block.get("name") == "build_robot_plan":
            return block.get("input", {}).get("actions", [])

    abort(502, "Claude did not return a robot plan.")


def generate_voice_plan_actions(transcript, locations):
    return call_anthropic_voice_plan(transcript, locations)


def sanitize_plan_actions(actions):
    if not isinstance(actions, list):
        return []

    sanitized = []
    for action in actions:
        if not isinstance(action, dict) or action.get("type") not in ACTION_TYPES:
            continue

        clean = dict(action)
        if clean["type"] in ("repeat", "battery_below"):
            clean["actions"] = sanitize_plan_actions(clean.get("actions"))
        sanitized.append(clean)

    return sanitized


def ensure_block_programs_dir():
    os.makedirs(BLOCK_PROGRAMS_DIR, exist_ok=True)


def ensure_openamr_data_dir():
    os.makedirs(os.path.dirname(BLOCK_LOCATIONS_FILE), exist_ok=True)


def program_path(name: str):
    if not PROGRAM_NAME_RE.match(name):
        abort(
            400,
            "Program names must be 1-64 characters and may use letters, numbers, spaces, dots, underscores, or hyphens.",
        )
    return os.path.join(BLOCK_PROGRAMS_DIR, f"{name}.json")


def validate_name(name: str, entity: str):
    if not PROGRAM_NAME_RE.match(name):
        abort(
            400,
            f"{entity} names must be 1-64 characters and may use letters, numbers, spaces, dots, underscores, or hyphens.",
        )


def parse_float(value, field: str):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        abort(400, f"{field} must be a number.")

    return parsed


def ensure_recordings_dir():
    os.makedirs(RECORDINGS_DIR, exist_ok=True)


def read_recordings_index():
    if not os.path.exists(RECORDINGS_INDEX_FILE):
        return []
    try:
        with open(RECORDINGS_INDEX_FILE, "r", encoding="utf-8") as index_file:
            return json.load(index_file)
    except (OSError, json.JSONDecodeError):
        return []


def write_recordings_index(entries):
    ensure_recordings_dir()
    with open(RECORDINGS_INDEX_FILE, "w", encoding="utf-8") as index_file:
        json.dump(entries, index_file, indent=2, sort_keys=True)


def dir_size_bytes(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for filename in files:
            try:
                total += os.path.getsize(os.path.join(root, filename))
            except OSError:
                pass
    return total


def validate_recording_name(name: str):
    if not RECORDING_NAME_RE.match(name or ""):
        abort(
            400,
            "Recording names must be 1-64 characters and may use letters, numbers, spaces, dots, underscores, or hyphens.",
        )


def validate_topics(topics):
    if topics is None:
        return None
    if not isinstance(topics, list) or not topics:
        abort(400, "topics must be a non-empty array of topic names, or omitted to record everything.")
    for topic in topics:
        if not isinstance(topic, str) or not TOPIC_NAME_RE.match(topic):
            abort(400, f"Invalid topic name: {topic!r}")
    return topics


def _reconcile_recordings_on_startup():
    """Any index entry still marked "recording" means Flask restarted
    mid-recording — the real ros2 bag process, if it's even still alive, is
    now orphaned and un-trackable (a fresh Python process has no handle to
    it). Don't guess whether it's fine; mark it honestly as interrupted."""
    entries = read_recordings_index()
    changed = False
    for entry in entries:
        if entry.get("status") == "recording":
            entry["status"] = "interrupted"
            print(
                f"[openamr_ui_package] WARNING: recording '{entry.get('name')}' was still "
                "marked active at startup — Flask must have restarted mid-recording. "
                "Marked interrupted; check whether a leftover ros2 bag process is still running."
            )
            changed = True
    if changed:
        write_recordings_index(entries)


_reconcile_recordings_on_startup()


def read_program_file(path: str):
    with open(path, "r", encoding="utf-8") as program_file:
        return json.load(program_file)


def read_block_locations():
    if not os.path.exists(BLOCK_LOCATIONS_FILE):
        return DEFAULT_BLOCK_LOCATIONS.copy()

    try:
        with open(BLOCK_LOCATIONS_FILE, "r", encoding="utf-8") as locations_file:
            data = json.load(locations_file)
    except (OSError, json.JSONDecodeError):
        return DEFAULT_BLOCK_LOCATIONS.copy()

    if not isinstance(data, dict):
        return DEFAULT_BLOCK_LOCATIONS.copy()

    locations = {}
    for name, pose in data.items():
        if not isinstance(name, str) or not PROGRAM_NAME_RE.match(name):
            continue
        if not isinstance(pose, dict):
            continue

        try:
            locations[name] = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose["yaw"]),
            }
        except (KeyError, TypeError, ValueError):
            continue

    return locations or DEFAULT_BLOCK_LOCATIONS.copy()


def write_block_locations(locations):
    ensure_openamr_data_dir()
    with open(BLOCK_LOCATIONS_FILE, "w", encoding="utf-8") as locations_file:
        json.dump(locations, locations_file, indent=2, sort_keys=True)


def read_run_history():
    if not os.path.exists(BLOCK_RUN_HISTORY_FILE):
        return []

    try:
        with open(BLOCK_RUN_HISTORY_FILE, "r", encoding="utf-8") as history_file:
            data = json.load(history_file)
    except (OSError, json.JSONDecodeError):
        return []

    return data if isinstance(data, list) else []


def write_run_history(history):
    ensure_openamr_data_dir()
    with open(BLOCK_RUN_HISTORY_FILE, "w", encoding="utf-8") as history_file:
        json.dump(history[:100], history_file, indent=2, sort_keys=True)


@app.after_request
def add_api_headers(response):
    # Allows React dev server on localhost:3000 to call the Flask API on 5050.
    response.headers.setdefault("Access-Control-Allow-Origin", "*")
    response.headers.setdefault("Access-Control-Allow-Headers", "Content-Type")
    response.headers.setdefault(
        "Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS"
    )
    return response


@app.errorhandler(HTTPException)
def handle_http_error(error):
    if request.path.startswith("/api/"):
        return (
            jsonify(
                {
                    "code": error.code,
                    "message": error.description,
                }
            ),
            error.code,
        )
    return error


@app.route("/api/auth/status", methods=["GET"])
def auth_status():
    remote_addr = request.remote_addr
    return jsonify(
        {
            "mode": AUTH_MODE,
            "requestedMode": REQUESTED_AUTH_MODE,
            "implemented": REQUESTED_AUTH_MODE in IMPLEMENTED_AUTH_MODES,
            "warning": AUTH_MODE_WARNING,
            "remoteAddr": remote_addr,
            "isLocalNetwork": is_local_address(remote_addr),
        }
    )


@app.route("/api/block-programs", methods=["GET"])
def list_block_programs():
    ensure_block_programs_dir()
    programs = []

    for filename in sorted(os.listdir(BLOCK_PROGRAMS_DIR)):
        if not filename.endswith(".json"):
            continue

        path = os.path.join(BLOCK_PROGRAMS_DIR, filename)
        name = filename[:-5]
        try:
            data = read_program_file(path)
            name = data.get("name", name)
        except (OSError, json.JSONDecodeError):
            pass

        programs.append(
            {
                "name": name,
                "updated_at": datetime.fromtimestamp(
                    os.path.getmtime(path), timezone.utc
                ).isoformat(),
            }
        )

    return jsonify({"programs": programs})


@app.route("/api/block-programs/<path:name>", methods=["GET"])
def get_block_program(name: str):
    path = program_path(name)
    if not os.path.exists(path):
        abort(404, "Block program not found")

    return jsonify(read_program_file(path))


@app.route("/api/block-programs/<path:name>", methods=["POST", "OPTIONS"])
def save_block_program(name: str):
    if request.method == "OPTIONS":
        return ("", 204)

    payload = request.get_json(silent=True) or {}
    workspace = payload.get("workspace")
    plan = payload.get("plan", [])

    if not isinstance(workspace, dict):
        abort(400, "Request body must include a Blockly workspace object.")

    ensure_block_programs_dir()
    saved_at = datetime.now(timezone.utc).isoformat()
    data = {
        "name": name,
        "saved_at": saved_at,
        "workspace": workspace,
        "plan": plan if isinstance(plan, list) else [],
    }

    with open(program_path(name), "w", encoding="utf-8") as program_file:
        json.dump(data, program_file, indent=2, sort_keys=True)

    return jsonify(data), 201


@app.route("/api/block-programs/<path:name>", methods=["DELETE", "OPTIONS"])
def delete_block_program(name: str):
    if request.method == "OPTIONS":
        return ("", 204)

    path = program_path(name)
    if not os.path.exists(path):
        abort(404, "Block program not found")

    os.remove(path)
    return jsonify({"deleted": name})


@app.route("/api/block-locations", methods=["GET"])
def list_block_locations():
    return jsonify({"locations": read_block_locations()})


@app.route("/api/block-locations/<path:name>", methods=["POST", "OPTIONS"])
def save_block_location(name: str):
    if request.method == "OPTIONS":
        return ("", 204)

    validate_name(name, "Location")
    payload = request.get_json(silent=True) or {}
    location = {
        "x": parse_float(payload.get("x"), "x"),
        "y": parse_float(payload.get("y"), "y"),
        "yaw": parse_float(payload.get("yaw"), "yaw"),
    }

    locations = read_block_locations()
    locations[name] = location
    write_block_locations(locations)

    return jsonify({"name": name, "location": location, "locations": locations}), 201


@app.route("/api/block-locations/<path:name>", methods=["DELETE", "OPTIONS"])
def delete_block_location(name: str):
    if request.method == "OPTIONS":
        return ("", 204)

    validate_name(name, "Location")
    locations = read_block_locations()
    if name not in locations:
        abort(404, "Block location not found")

    del locations[name]
    write_block_locations(locations)

    return jsonify({"deleted": name, "locations": locations})


@app.route("/api/block-run-history", methods=["GET"])
def list_block_run_history():
    return jsonify({"history": read_run_history()})


@app.route("/api/block-run-history", methods=["POST", "OPTIONS"])
def save_block_run_history():
    if request.method == "OPTIONS":
        return ("", 204)

    payload = request.get_json(silent=True) or {}
    status = payload.get("status")
    if status not in {"success", "failed", "stopped"}:
        abort(400, "Run history status must be success, failed, or stopped.")

    entry = {
        "program_name": str(payload.get("program_name") or "Untitled Program"),
        "status": status,
        "started_at": str(payload.get("started_at") or ""),
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "duration_ms": int(payload.get("duration_ms") or 0),
        "steps_total": int(payload.get("steps_total") or 0),
        "steps_completed": int(payload.get("steps_completed") or 0),
        "error_message": str(payload.get("error_message") or ""),
    }

    history = [entry, *read_run_history()]
    write_run_history(history)

    return jsonify({"entry": entry, "history": history[:100]}), 201


@app.route("/api/block-run-history", methods=["DELETE", "OPTIONS"])
def clear_block_run_history():
    if request.method == "OPTIONS":
        return ("", 204)

    write_run_history([])
    return jsonify({"history": []})


@app.route("/api/voice-plan", methods=["POST", "OPTIONS"])
def create_voice_plan():
    if request.method == "OPTIONS":
        return ("", 204)

    payload = request.get_json(silent=True) or {}
    transcript = str(payload.get("transcript") or "").strip()
    if not transcript:
        abort(400, "Request body must include a non-empty transcript.")

    locations = payload.get("locations")
    if not isinstance(locations, dict):
        locations = {}

    raw_actions = generate_voice_plan_actions(transcript, locations)
    plan = sanitize_plan_actions(raw_actions)

    return jsonify({"plan": plan, "transcript": transcript})


def get_robot_description_urdf_xml():
    """Xacro-process the vendored robot description, cached by file mtime.

    Re-reads gazebo_control.xacro's mtime too since it's xacro:included by
    the main file and edits there should also invalidate the cache.
    """
    if not os.path.exists(ROBOT_DESC_XACRO):
        abort(404, "Robot description xacro not found on this install.")

    included = os.path.join(os.path.dirname(ROBOT_DESC_XACRO), "gazebo_control.xacro")
    try:
        mtime = (
            os.path.getmtime(ROBOT_DESC_XACRO),
            os.path.getmtime(included) if os.path.exists(included) else 0,
        )
    except OSError as error:
        abort(500, f"Could not stat robot description files: {error}")

    if _robot_urdf_cache["mtime"] == mtime and _robot_urdf_cache["xml"] is not None:
        return _robot_urdf_cache["xml"]

    try:
        doc = xacro.process_file(ROBOT_DESC_XACRO)
        xml_text = doc.toxml()
    except Exception as error:  # xacro raises plain Exception/xml errors
        abort(500, f"Xacro processing failed: {error}")

    _robot_urdf_cache["mtime"] = mtime
    _robot_urdf_cache["xml"] = xml_text
    return xml_text


@app.route("/api/robot-description/manifest", methods=["GET"])
def robot_description_manifest():
    xacro_exists = os.path.exists(ROBOT_DESC_XACRO)
    return jsonify(
        {
            "name": "robo_urdf",
            "displayName": "OpenAMRobot",
            "package": "openamrobot_description",
            "sourceRepo": "openAMRobot/openamr-platform-sw",
            "available": xacro_exists,
            "urdfUrl": "/api/robot-description/urdf",
            "assetBaseUrl": "/api/robot-description/assets",
            "packages": {"openamrobot_description": "/api/robot-description/assets"},
        }
    )


@app.route("/api/robot-description/urdf", methods=["GET"])
def robot_description_urdf():
    xml_text = get_robot_description_urdf_xml()
    return app.response_class(xml_text, mimetype="application/xml")


@app.route("/api/robot-description/assets/<path:filename>", methods=["GET"])
def robot_description_assets(filename: str):
    # filename comes straight from the URL path — normalize and confirm the
    # resolved path stays inside ROBOT_DESC_DIR before serving anything.
    requested = os.path.normpath(os.path.join(ROBOT_DESC_DIR, filename))
    if not requested.startswith(os.path.join(ROBOT_DESC_DIR, "")):
        abort(404, "Asset not found.")
    if not os.path.isfile(requested):
        abort(404, "Asset not found.")

    rel_dir = os.path.dirname(filename)
    rel_name = os.path.basename(filename)
    return send_from_directory(os.path.join(ROBOT_DESC_DIR, rel_dir), rel_name)


@app.route("/api/devices/serial-ports", methods=["GET"])
def list_serial_ports():
    """Real serial ports currently present on this host (USB/Pi-attached).

    This is genuine detection, not a stand-in for full USB/CAN plug-and-play
    support: it only sees serial devices on the machine running this Flask
    process, via pyserial's udev-backed enumeration. Every Linux box also
    exposes /dev/ttyS0-31 legacy platform serial ports whether or not
    anything is attached to them; pyserial reports hwid "n/a" for those, so
    they're filtered out here to avoid a permanently-populated fake list.
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        return jsonify({"ports": [], "supported": False})

    ports = [
        {
            "device": port.device,
            "description": port.description if port.description != "n/a" else None,
            "manufacturer": port.manufacturer,
            "vid": port.vid,
            "pid": port.pid,
        }
        for port in list_ports.comports()
        if port.hwid and port.hwid != "n/a"
    ]
    return jsonify({"ports": ports, "supported": True})


def _recording_alive():
    return _recording["proc"] is not None and _recording["proc"].poll() is None


def _replay_alive():
    return _replay["proc"] is not None and _replay["proc"].poll() is None


def _finalize_recording(interrupted=False):
    """Common cleanup for a recording that has stopped, one way or another
    — clean Stop request, the subprocess exiting on its own (duration/size
    limit), or a leftover marked interrupted at startup."""
    entries = read_recordings_index()
    for entry in entries:
        if entry.get("id") == _recording["id"]:
            entry["status"] = "interrupted" if interrupted else "complete"
            entry["endedAt"] = datetime.now(timezone.utc).isoformat()
            bag_path = os.path.join(RECORDINGS_DIR, entry["id"])
            entry["sizeBytes"] = dir_size_bytes(bag_path) if os.path.isdir(bag_path) else 0
            break
    write_recordings_index(entries)
    _recording.update({"proc": None, "id": None, "name": None, "started_at": None, "topics": None})


@app.route("/api/recordings", methods=["GET"])
def list_recordings():
    # Reap a recording that exited on its own (e.g. hit a size/duration
    # limit) since the last time anyone asked.
    if _recording["id"] and not _recording_alive():
        _finalize_recording()
    return jsonify({"recordings": read_recordings_index()})


@app.route("/api/recordings/status", methods=["GET"])
def recordings_status():
    if _recording["id"] and not _recording_alive():
        _finalize_recording()
    if _replay["id"] and not _replay_alive():
        _replay.update({"proc": None, "id": None, "started_at": None, "paused": False, "rate": 1.0})

    recording = None
    if _recording["id"]:
        recording = {
            "id": _recording["id"],
            "name": _recording["name"],
            "startedAt": _recording["started_at"],
            "topics": _recording["topics"],
        }

    replay = None
    if _replay["id"]:
        replay = {
            "id": _replay["id"],
            "startedAt": _replay["started_at"],
            "paused": _replay["paused"],
            "rate": _replay["rate"],
        }

    return jsonify({"recording": recording, "replay": replay})


@app.route("/api/recordings/start", methods=["POST", "OPTIONS"])
def start_recording():
    if request.method == "OPTIONS":
        return ("", 204)

    if _recording["id"] and _recording_alive():
        abort(409, "A recording is already in progress — stop it before starting another.")

    payload = request.get_json(silent=True) or {}
    name = str(payload.get("name") or "").strip()
    validate_recording_name(name)
    topics = validate_topics(payload.get("topics"))
    description = str(payload.get("description") or "").strip()[:500]

    ensure_recordings_dir()
    recording_id = f"{re.sub(r'[^A-Za-z0-9]+', '_', name).strip('_') or 'recording'}_{int(time.time())}"
    bag_path = os.path.join(RECORDINGS_DIR, recording_id)

    cmd = ["ros2", "bag", "record", "-o", bag_path, "--disable-keyboard-controls"]
    cmd += topics if topics else ["--all-topics"]

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        abort(500, "ros2 command not found on this backend — is the ROS environment sourced?")

    started_at = datetime.now(timezone.utc).isoformat()
    _recording.update(
        {"proc": proc, "id": recording_id, "name": name, "started_at": started_at, "topics": topics}
    )

    entries = read_recordings_index()
    entries.append(
        {
            "id": recording_id,
            "name": name,
            "description": description,
            "topics": topics,
            "status": "recording",
            "startedAt": started_at,
            "endedAt": None,
            "sizeBytes": 0,
        }
    )
    write_recordings_index(entries)

    return jsonify({"id": recording_id, "name": name, "startedAt": started_at}), 201


@app.route("/api/recordings/stop", methods=["POST", "OPTIONS"])
def stop_recording():
    if request.method == "OPTIONS":
        return ("", 204)

    if not _recording["id"]:
        abort(409, "No recording is in progress.")

    if _recording_alive():
        _recording["proc"].send_signal(signal.SIGINT)
        try:
            _recording["proc"].wait(timeout=10)
        except subprocess.TimeoutExpired:
            _recording["proc"].kill()

    _finalize_recording()
    return jsonify({"stopped": True})


@app.route("/api/recordings/<recording_id>", methods=["DELETE", "OPTIONS"])
def delete_recording(recording_id):
    if request.method == "OPTIONS":
        return ("", 204)

    if _recording["id"] == recording_id or _replay["id"] == recording_id:
        abort(409, "That recording is currently active — stop it first.")

    entries = read_recordings_index()
    remaining = [entry for entry in entries if entry.get("id") != recording_id]
    if len(remaining) == len(entries):
        abort(404, "Recording not found.")

    bag_path = os.path.join(RECORDINGS_DIR, recording_id)
    if os.path.isdir(bag_path):
        import shutil

        shutil.rmtree(bag_path, ignore_errors=True)

    write_recordings_index(remaining)
    return jsonify({"deleted": recording_id})


@app.route("/api/recordings/<recording_id>/download", methods=["GET"])
def download_recording(recording_id):
    # A rosbag is a directory (metadata.yaml + one or more .db3/.mcap files),
    # so we can't hand back a single file directly — zip the whole bag dir to a
    # temp archive and stream that. The recording must have finished; a bag
    # that's still being written isn't safe to archive.
    entries = read_recordings_index()
    entry = next((e for e in entries if e.get("id") == recording_id), None)
    if not entry:
        abort(404, "Recording not found.")
    if _recording["id"] == recording_id and _recording_alive():
        abort(409, "That recording is still in progress — stop it before downloading.")

    bag_path = os.path.join(RECORDINGS_DIR, recording_id)
    if not os.path.isdir(bag_path):
        abort(404, "Recording files are missing on disk.")

    import shutil
    import tempfile

    tmp_base = os.path.join(tempfile.gettempdir(), f"{recording_id}")
    archive_path = shutil.make_archive(tmp_base, "zip", root_dir=bag_path)

    response = send_file(
        archive_path,
        as_attachment=True,
        download_name=f"{recording_id}.zip",
        mimetype="application/zip",
    )

    # Delete the temp archive once the response has been fully sent — we only
    # needed it to stream; keeping it would leak disk on every download.
    @response.call_on_close
    def _cleanup():
        try:
            os.remove(archive_path)
        except OSError:
            pass

    return response


@app.route("/api/recordings/<recording_id>/replay/start", methods=["POST", "OPTIONS"])
def start_replay(recording_id):
    if request.method == "OPTIONS":
        return ("", 204)

    if _replay["id"] and _replay_alive():
        abort(409, "A replay is already in progress — stop it before starting another.")

    entries = read_recordings_index()
    entry = next((e for e in entries if e.get("id") == recording_id), None)
    if not entry:
        abort(404, "Recording not found.")
    if entry.get("status") not in ("complete", "interrupted"):
        abort(409, "That recording hasn't finished yet.")

    bag_path = os.path.join(RECORDINGS_DIR, recording_id)
    if not os.path.isdir(bag_path):
        abort(404, "Recording files are missing on disk.")

    payload = request.get_json(silent=True) or {}
    rate = parse_float(payload.get("rate", 1.0), "rate")
    if rate <= 0 or rate > 10:
        abort(400, "rate must be between 0 and 10.")

    cmd = ["ros2", "bag", "play", bag_path, "--disable-keyboard-controls", "-r", str(rate)]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        abort(500, "ros2 command not found on this backend — is the ROS environment sourced?")

    started_at = datetime.now(timezone.utc).isoformat()
    _replay.update(
        {"proc": proc, "id": recording_id, "started_at": started_at, "paused": False, "rate": rate}
    )
    return jsonify({"id": recording_id, "startedAt": started_at, "rate": rate}), 201


@app.route("/api/recordings/replay/stop", methods=["POST", "OPTIONS"])
def stop_replay():
    if request.method == "OPTIONS":
        return ("", 204)

    if not _replay["id"]:
        abort(409, "No replay is in progress.")

    if _replay_alive():
        if _replay["paused"]:
            _replay["proc"].send_signal(signal.SIGCONT)
        _replay["proc"].send_signal(signal.SIGINT)
        try:
            _replay["proc"].wait(timeout=10)
        except subprocess.TimeoutExpired:
            _replay["proc"].kill()

    _replay.update({"proc": None, "id": None, "started_at": None, "paused": False, "rate": 1.0})
    return jsonify({"stopped": True})


@app.route("/api/recordings/replay/pause", methods=["POST", "OPTIONS"])
def pause_replay():
    if request.method == "OPTIONS":
        return ("", 204)
    if not _replay["id"] or not _replay_alive():
        abort(409, "No replay is in progress.")
    if not _replay["paused"]:
        _replay["proc"].send_signal(signal.SIGSTOP)
        _replay["paused"] = True
    return jsonify({"paused": True})


@app.route("/api/recordings/replay/resume", methods=["POST", "OPTIONS"])
def resume_replay():
    if request.method == "OPTIONS":
        return ("", 204)
    if not _replay["id"] or not _replay_alive():
        abort(409, "No replay is in progress.")
    if _replay["paused"]:
        _replay["proc"].send_signal(signal.SIGCONT)
        _replay["paused"] = False
    return jsonify({"paused": False})


@app.route("/ros/<path:filename>")
def serve_ros_libs(filename: str):
    # Serve legacy ROS web libs placed under build/ros/
    return send_from_directory(REACT_ROS_DIR, filename)


@app.route("/", defaults={"path": ""})
@app.route("/<path:path>")
def serve_spa(path: str):
    # If someone requests a real file that exists in the build root (e.g., favicon.ico)
    requested = os.path.join(REACT_BUILD_DIR, path)
    if path and os.path.exists(requested) and os.path.isfile(requested):
        return send_from_directory(REACT_BUILD_DIR, path)

    # Otherwise return React index.html (SPA routing)
    return send_from_directory(REACT_BUILD_DIR, "index.html")


class ParamFlask(Node):
    def __init__(self):
        super().__init__("flask")
        self.declare_parameter("appAddress", "127.0.0.1")
        self.declare_parameter("portApp", 5050)


# Optional HTTPS support, mainly so the microphone works on browsers other
# than localhost (Chrome/Safari refuse getUserMedia/SpeechRecognition on
# plain HTTP LAN origins). Point these at an mkcert-issued cert/key to enable
# it; if either file is missing, the server falls back to plain HTTP exactly
# as before.
SSL_CERT_FILE = os.environ.get(
    "OPENAMR_UI_SSL_CERT",
    os.path.join(os.path.expanduser("~"), ".openamr_ui", "certs", "cert.pem"),
)
SSL_KEY_FILE = os.environ.get(
    "OPENAMR_UI_SSL_KEY",
    os.path.join(os.path.expanduser("~"), ".openamr_ui", "certs", "key.pem"),
)


def main():
    rclpy.init()
    node = ParamFlask()

    host = node.get_parameter("appAddress").get_parameter_value().string_value
    port = node.get_parameter("portApp").get_parameter_value().integer_value

    ssl_context = None
    if os.path.exists(SSL_CERT_FILE) and os.path.exists(SSL_KEY_FILE):
        ssl_context = (SSL_CERT_FILE, SSL_KEY_FILE)
        node.get_logger().info(f"Serving HTTPS using {SSL_CERT_FILE}")

    # threaded=True so one slow/stuck connection (e.g. a browser probing with
    # a plain-HTTP request against the HTTPS port, or a stalled TLS
    # handshake) can't block every other client on this single process.
    app.run(host=host, port=port, debug=False, ssl_context=ssl_context, threaded=True)


if __name__ == "__main__":
    main()
