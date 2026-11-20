#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEB_BUILD_DIR="${REPO_ROOT}/web/build"

ROS_APP_DIR="${REPO_ROOT}/ros2/src/openamr_ui_package/openamr_ui_package/static/app"

if [ ! -f "${WEB_BUILD_DIR}/index.html" ]; then
  echo "ERROR: web build not found: ${WEB_BUILD_DIR}"
  echo "Run: bash scripts/build_frontend.sh"
  exit 1
fi

mkdir -p "${ROS_APP_DIR}"

echo "[sync] Cleaning existing ROS package React build..."
find "${ROS_APP_DIR}" -mindepth 1 ! -name README.md -exec rm -rf {} +

echo "[sync] Copying full React build into ROS2 package static/app/ ..."
cp -a "${WEB_BUILD_DIR}/." "${ROS_APP_DIR}/"

# build-info.json ties the served bundle to a commit and to the exact main
# JS file. flask_app.py refuses to serve index.html when this file is
# missing or the main JS does not match it (stale or partial copy).
python3 - "${ROS_APP_DIR}" "${REPO_ROOT}" <<'PY'
import hashlib, json, os, subprocess, sys
app_dir, repo = sys.argv[1], sys.argv[2]
manifest = json.load(open(os.path.join(app_dir, "asset-manifest.json")))
main_js = manifest["files"]["main.js"]
path = os.path.join(app_dir, main_js.lstrip("/"))
digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
def git(*args):
    try:
        return subprocess.run(["git", "-C", repo, *args], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return ""
commit = os.environ.get("OPENAMR_UI_COMMIT") or git("rev-parse", "HEAD") or "unknown"
dirty = bool(git("status", "--porcelain", "--", "web")) if commit != "unknown" else None
info = {"commit": commit, "web_dirty": dirty, "main_js": main_js, "main_js_sha256": digest}
json.dump(info, open(os.path.join(app_dir, "build-info.json"), "w"), indent=2, sort_keys=True)
print("[sync] build-info.json:", json.dumps(info, sort_keys=True))
PY

echo "[sync] OK: Synced frontend build to ${ROS_APP_DIR}"
