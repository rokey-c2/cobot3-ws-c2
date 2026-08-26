#!/usr/bin/env bash
# Project dependency bootstrap for a PC that already has the large platform
# prerequisites installed (ROS2 Jazzy, Isaac Sim 5.1, Docker, Node/npm).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT_DIR"

echo "[1/4] Frontend dependencies"
"$ROOT_DIR/scripts/setup_frontend.sh"

echo ""
echo "[2/4] ROS2 dependencies"
"$ROOT_DIR/scripts/install_ros_dependencies.sh"

echo ""
echo "[3/4] Vision Python dependencies"
"$ROOT_DIR/scripts/setup_vision_env.sh"

echo ""
echo "[4/4] ROS2 MQTT Adapter Python dependencies"
"$ROOT_DIR/scripts/setup_adapter_env.sh"

echo ""
echo "Setup complete. Run ./scripts/check_environment.sh for a final prerequisite check."
