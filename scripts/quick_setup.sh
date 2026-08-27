#!/usr/bin/env bash
# One-time project bootstrap for a new team PC.
# Large platform prerequisites (Ubuntu/ROS2/Isaac Sim/Docker) are intentionally
# not installed here. This script installs project dependencies, builds ROS2,
# prepares env files, and builds Docker images.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_SYSTEM_DEPS=0

usage() {
    cat <<'USAGE'
Usage: ./scripts/quick_setup.sh [--install-system-deps]

Options:
  --install-system-deps  Install small Ubuntu helper packages used by this repo
                         (venv, OpenCV, rosdep, colcon, Node/npm).

Required beforehand:
  - Ubuntu 24.04
  - ROS 2 Jazzy at /opt/ros/jazzy
  - Isaac Sim 5.1 (default: ~/isaacsim)
  - Docker + Docker Compose plugin
USAGE
}

for arg in "$@"; do
    case "$arg" in
        --install-system-deps) INSTALL_SYSTEM_DEPS=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] unknown option: $arg"; usage; exit 2 ;;
    esac
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/IsaacSim-ros_workspaces/jazzy_ws}"

cd "$ROOT_DIR"

echo "============================================"
echo " COBOT3 NEW PC QUICK SETUP"
echo "============================================"
echo "Repository: $ROOT_DIR"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "[ERROR] ROS 2 Jazzy is not installed at /opt/ros/jazzy"
    exit 1
fi

if [ ! -x "$ISAAC_SIM_DIR/python.sh" ]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    echo "        If Isaac Sim is elsewhere:"
    echo "        ISAAC_SIM_DIR=/path/to/isaacsim ./scripts/quick_setup.sh"
    exit 1
fi

if [ ! -f "$ISAAC_ROS_WS/install/setup.bash" ]; then
    echo "[ERROR] Isaac Sim ROS Jazzy workspace not found: $ISAAC_ROS_WS/install/setup.bash"
    echo "        iw_hub_navigation is required by scripts/run_ros2.sh"
    echo "        If it is elsewhere:"
    echo "        ISAAC_ROS_WS=/path/to/jazzy_ws ./scripts/quick_setup.sh"
    exit 1
fi

if [ ! -f "$ROOT_DIR/models/parcel_box_yolo_model/best.onnx" ]; then
    echo "[ERROR] YOLO model missing: models/parcel_box_yolo_model/best.onnx"
    exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "[ERROR] Docker is not installed"
    exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
    echo "[ERROR] Docker Compose plugin is not available: docker compose"
    exit 1
fi

if [ "$INSTALL_SYSTEM_DEPS" = "1" ]; then
    echo "[SYSTEM] installing helper packages"
    sudo apt update
    sudo apt install -y \
        python3.12-venv \
        python3-opencv \
        python3-rosdep \
        python3-colcon-common-extensions \
        nodejs \
        npm
fi

missing=0
for command_name in python3 rosdep colcon node npm; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "[ERROR] missing command: $command_name"
        missing=1
    fi
done

if ! python3 -c 'import venv' >/dev/null 2>&1; then
    echo "[ERROR] Python venv module missing"
    echo "        sudo apt install -y python3.12-venv"
    missing=1
fi

if ! python3 -c 'import cv2' >/dev/null 2>&1; then
    echo "[ERROR] Python OpenCV missing"
    echo "        sudo apt install -y python3-opencv"
    missing=1
fi

if [ "$missing" != "0" ]; then
    echo
    echo "Run again with: ./scripts/quick_setup.sh --install-system-deps"
    exit 1
fi

NODE_MAJOR="$(node -p "Number(process.versions.node.split('.')[0])" 2>/dev/null || echo 0)"
if [ "$NODE_MAJOR" -lt 18 ]; then
    echo "[ERROR] Node.js 18+ required; current $(node -v)"
    exit 1
fi

if [ ! -f .env ]; then
    cp .env.example .env
    echo "[ENV] created .env from .env.example"
else
    echo "[ENV] keeping existing .env"
fi

if [ ! -f frontend/.env ]; then
    cp frontend/.env.example frontend/.env
    echo "[ENV] created frontend/.env from frontend/.env.example"
else
    echo "[ENV] keeping existing frontend/.env"
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "[ROSDEP] initializing rosdep"
    sudo rosdep init
fi

echo "[ROSDEP] updating index"
rosdep update

echo
echo "[SETUP] project dependencies"
./scripts/setup_all.sh

echo
echo "[BUILD] ROS2 workspace"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
(
    cd "$ROOT_DIR/ros2_ws"
    colcon build --symlink-install
)

echo
echo "[BUILD] Docker images"
if docker info >/dev/null 2>&1; then
    docker compose build
else
    sudo docker compose build
fi

echo
echo "[CHECK] final environment"
./scripts/check_environment.sh

echo
echo "============================================"
echo " SETUP COMPLETE"
echo "============================================"
echo "Start the full stack:"
echo "  ./scripts/start_all.sh --fresh-db"
echo
echo "Then trigger the real mission:"
echo "  ./scripts/start_mission.sh"
echo
echo "Or do both in one command:"
echo "  ./scripts/start_all.sh --fresh-db --mission"
