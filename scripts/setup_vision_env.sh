#!/usr/bin/env bash
# P3020 vision dependencies are installed in a project venv without modifying
# the system Python. --system-site-packages keeps ROS2 apt packages visible.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$ROOT_DIR/.venv"
REQUIREMENTS="$ROOT_DIR/requirements/vision.txt"

if ! command -v python3 >/dev/null 2>&1; then
    echo "[ERROR] python3 is not installed"
    exit 1
fi

if ! python3 -c 'import venv' >/dev/null 2>&1; then
    echo "[ERROR] Python venv module is not available"
    echo "Ubuntu 24.04 example: sudo apt install -y python3.12-venv"
    exit 1
fi

if [ -d "$VENV_DIR" ] && [ ! -x "$VENV_DIR/bin/python3" ]; then
    echo "[ERROR] Existing .venv is incomplete: $VENV_DIR"
    echo "Remove the broken .venv and run this script again."
    exit 1
fi

python3 -m venv --system-site-packages "$VENV_DIR"
"$VENV_DIR/bin/python3" -m pip install --upgrade pip
"$VENV_DIR/bin/python3" -m pip install -r "$REQUIREMENTS"

if ! "$VENV_DIR/bin/python3" -c 'import cv2' >/dev/null 2>&1; then
    echo "[ERROR] OpenCV for the laser HUD/MJPEG stream is not available"
    echo "Ubuntu 24.04 example: sudo apt install -y python3-opencv"
    exit 1
fi

echo ""
echo "[Vision] setup complete"
echo "Activate with: source $VENV_DIR/bin/activate"
