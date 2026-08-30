#!/usr/bin/env bash
# Install the non-ROS Python dependency for scripts/ros2_mqtt_adapter.py while
# keeping ROS2 packages from /opt/ros visible through --system-site-packages.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$ROOT_DIR/.venv"
REQUIREMENTS="$ROOT_DIR/requirements/ros2_adapter.txt"

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

echo ""
echo "[ROS2 MQTT Adapter] setup complete"
echo "Run with: $VENV_DIR/bin/python3 scripts/ros2_mqtt_adapter.py"
