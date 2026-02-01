#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEB_DIR="${REPO_ROOT}/web"

if [ ! -d "${WEB_DIR}" ]; then
  echo "ERROR: web/ directory not found at: ${WEB_DIR}"
  exit 1
fi

cd "${WEB_DIR}"

echo "[build_frontend] Installing dependencies..."
if [ -f package-lock.json ]; then
  npm ci
else
  npm install
fi

echo "[build_frontend] Building production bundle..."
npm run build

if [ ! -d "${WEB_DIR}/build" ]; then
  echo "ERROR: build/ folder not found after npm run build"
  exit 1
fi

echo "[build_frontend] OK: build created at ${WEB_DIR}/build"
