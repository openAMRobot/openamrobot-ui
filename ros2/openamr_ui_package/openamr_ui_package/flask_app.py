import os
import rclpy
from rclpy.node import Node
from flask import Flask, send_from_directory

# ---------------------------------------------------------------------
# Professional P1: Serve built React app from:
#   openamr_ui_package/static/app/
#
# React build layout (CRA):
#   static/app/index.html
#   static/app/static/js/main.<hash>.js
#   static/app/static/css/main.<hash>.css
#
# Browser requests by default:
#   /static/js/...
#   /static/css/...
#
# So we explicitly map:
#   URL /static/<...>  ->  filesystem static/app/static/<...>
# ---------------------------------------------------------------------

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# openamr_ui_package/static/
PKG_STATIC_DIR = os.path.join(BASE_DIR, "static")

# openamr_ui_package/static/app/   (React build root)
REACT_BUILD_DIR = os.path.join(PKG_STATIC_DIR, "app")

# openamr_ui_package/static/app/static/   (hashed assets)
REACT_ASSETS_DIR = os.path.join(REACT_BUILD_DIR, "static")

# Create Flask app without using Flask's default static handling.
# We'll provide our own /static route pointing at REACT_ASSETS_DIR.
app = Flask(__name__)


@app.route("/static/<path:filename>")
def react_assets(filename: str):
    # Serves:
    #   /static/js/...  -> REACT_ASSETS_DIR/js/...
    #   /static/css/... -> REACT_ASSETS_DIR/css/...
    return send_from_directory(REACT_ASSETS_DIR, filename)


@app.route("/", defaults={"path": ""})
@app.route("/<path:path>")
def spa(path: str):
    # Serve React index.html for SPA routes like:
    #   /, /route, /control, /info, etc.
    #
    # If you later want to also serve non-static files from REACT_BUILD_DIR
    # (like /asset-manifest.json), add explicit routes for them.
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

    # Production note: Flask dev server is OK for now; for production use gunicorn/uvicorn.
    app.run(host=host, port=port, debug=False)


if __name__ == "__main__":
    main()
