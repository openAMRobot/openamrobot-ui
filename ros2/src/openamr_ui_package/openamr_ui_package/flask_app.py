import os
import rclpy
from rclpy.node import Node
from flask import Flask, send_from_directory
from ament_index_python.packages import get_package_share_directory

# React build is installed to share/openamr_ui_package/static/app/ by setup.py
SHARE_DIR = get_package_share_directory("openamr_ui_package")
REACT_BUILD_DIR = os.path.join(SHARE_DIR, "app")
REACT_STATIC_DIR = os.path.join(REACT_BUILD_DIR, "static")
REACT_ROS_DIR = os.path.join(REACT_BUILD_DIR, "ros")

# Make Flask serve /static/* from the CRA build folder
app = Flask(__name__, static_folder=REACT_STATIC_DIR, static_url_path="/static")


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
