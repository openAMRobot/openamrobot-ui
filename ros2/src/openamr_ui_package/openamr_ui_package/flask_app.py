import os
import json
import re
import urllib.request
import urllib.error
from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from flask import Flask, abort, jsonify, request, send_from_directory
from ament_index_python.packages import get_package_share_directory
from werkzeug.exceptions import HTTPException

# React build is installed to share/openamr_ui_package/static/app/ by setup.py
SHARE_DIR = get_package_share_directory("openamr_ui_package")
REACT_BUILD_DIR = os.path.join(SHARE_DIR, "app")
REACT_STATIC_DIR = os.path.join(REACT_BUILD_DIR, "static")
REACT_ROS_DIR = os.path.join(REACT_BUILD_DIR, "ros")

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
