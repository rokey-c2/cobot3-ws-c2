#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FRONTEND_DIR="$ROOT_DIR/frontend"

if ! command -v node >/dev/null 2>&1; then
    echo "[ERROR] node is not installed"
    echo "Ubuntu 24.04 example: sudo apt update && sudo apt install -y nodejs npm"
    exit 1
fi

if ! command -v npm >/dev/null 2>&1; then
    echo "[ERROR] npm is not installed"
    echo "Ubuntu 24.04 example: sudo apt update && sudo apt install -y npm"
    exit 1
fi

NODE_MAJOR="$(node -p "Number(process.versions.node.split('.')[0])")"
if [ "$NODE_MAJOR" -lt 18 ]; then
    echo "[ERROR] Node.js 18 or newer is required. Current: $(node -v)"
    exit 1
fi

echo "[Frontend] Node: $(node -v)"
echo "[Frontend] npm:  $(npm -v)"
cd "$FRONTEND_DIR"

if [ -f package-lock.json ]; then
    echo "[Frontend] package-lock.json found -> npm ci"
    npm ci
else
    echo "[Frontend] package-lock.json not found -> npm install"
    npm install
    echo "[Frontend] package-lock.json was generated. Commit it to Git for reproducible installs."
fi

echo "[Frontend] setup complete"
