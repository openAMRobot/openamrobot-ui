#!/usr/bin/env bash
set -euo pipefail

# Legacy entrypoint kept for compatibility.
# Canonical run flow lives in ./scripts/
bash scripts/run_ui_backend.sh
