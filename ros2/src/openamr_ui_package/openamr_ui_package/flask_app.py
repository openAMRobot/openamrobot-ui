import os
import json
import re
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


def main():
    rclpy.init()
    node = ParamFlask()

    host = node.get_parameter("appAddress").get_parameter_value().string_value
    port = node.get_parameter("portApp").get_parameter_value().integer_value

    app.run(host=host, port=port, debug=False)


if __name__ == "__main__":
    main()
