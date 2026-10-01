#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FRONTEND_DIR="$ROOT_DIR/frontend"

if ! command -v node >/dev/null 2>&1; then
    echo "[ERROR] node is not installed"
    echo "Install Node.js 20.19+ or 22.12+ with npm. With nvm: nvm install 24 && nvm use 24"
    exit 1
fi

if ! command -v npm >/dev/null 2>&1; then
    echo "[ERROR] npm is not installed"
    echo "Install npm with a compatible Node.js release. With nvm: nvm install 24 && nvm use 24"
    exit 1
fi

if ! node -e 'const [major, minor] = process.versions.node.split(".").map(Number); process.exit((major === 20 && minor >= 19) || (major === 22 && minor >= 12) || major > 22 ? 0 : 1)'; then
    echo "[ERROR] Node.js 20.19+ or 22.12+ is required. Current: $(node -v)"
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
