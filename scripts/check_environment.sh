#!/usr/bin/env bash
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/IsaacSim-ros_workspaces/jazzy_ws}"
FAILED=0

ok() { printf '[ OK ] %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1"; }
fail() { printf '[FAIL] %s\n' "$1"; FAILED=1; }

printf 'COBOT3 environment check\n'
printf 'Repository: %s\n' "$ROOT_DIR"
printf 'Expected ROS_DOMAIN_ID: 110\n\n'

if command -v git >/dev/null 2>&1; then ok "git $(git --version | awk '{print $3}')"; else fail "git not found"; fi
if command -v python3 >/dev/null 2>&1; then ok "python3 $(python3 --version 2>&1 | awk '{print $2}')"; else fail "python3 not found"; fi

if command -v docker >/dev/null 2>&1; then
    ok "docker installed"
    if docker compose version >/dev/null 2>&1; then ok "docker compose plugin"; else fail "docker compose plugin missing"; fi
else
    fail "docker not found"
fi

if command -v node >/dev/null 2>&1; then
    NODE_MAJOR="$(node -p "Number(process.versions.node.split('.')[0])" 2>/dev/null || echo 0)"
    if [ "$NODE_MAJOR" -ge 18 ]; then ok "Node $(node -v)"; else fail "Node 18+ required; current $(node -v)"; fi
else
    fail "node not found"
fi
if command -v npm >/dev/null 2>&1; then ok "npm $(npm -v)"; else fail "npm not found"; fi

if [ -f /opt/ros/jazzy/setup.bash ]; then ok "ROS 2 Jazzy"; else fail "ROS 2 Jazzy not found at /opt/ros/jazzy"; fi
if command -v rosdep >/dev/null 2>&1; then ok "rosdep installed"; else fail "rosdep not found"; fi
if command -v colcon >/dev/null 2>&1; then ok "colcon installed"; else fail "colcon not found"; fi

if [ -x "$ISAAC_SIM_DIR/python.sh" ]; then ok "Isaac Sim python.sh: $ISAAC_SIM_DIR/python.sh"; else fail "Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"; fi
if [ -f "$ISAAC_ROS_WS/install/setup.bash" ]; then ok "Isaac ROS Jazzy workspace: $ISAAC_ROS_WS"; else fail "Isaac ROS Jazzy workspace not found: $ISAAC_ROS_WS"; fi
if [ -d "$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/jazzy" ]; then ok "Isaac Sim ROS2 Jazzy bridge"; else warn "Isaac Sim ROS2 Jazzy bridge directory not found at expected path"; fi

if [ -f "$ROOT_DIR/.env" ]; then ok ".env created"; else warn ".env missing; quick_setup.sh will create it"; fi
if [ -f "$ROOT_DIR/frontend/.env" ]; then ok "frontend/.env created"; else warn "frontend/.env missing; quick_setup.sh will create it"; fi

if [ -f "$ROOT_DIR/backend/requirements.txt" ]; then ok "Backend requirements.txt"; else fail "backend/requirements.txt missing"; fi
if [ -f "$ROOT_DIR/frontend/package.json" ]; then ok "Frontend package.json"; else fail "frontend/package.json missing"; fi
if [ -f "$ROOT_DIR/frontend/package-lock.json" ]; then ok "Frontend package-lock.json"; else fail "frontend/package-lock.json missing"; fi
if [ -d "$ROOT_DIR/frontend/node_modules" ]; then ok "Frontend node_modules installed"; else warn "frontend/node_modules missing; run quick_setup.sh"; fi

if [ -f "$ROOT_DIR/requirements/vision.txt" ]; then ok "Vision requirements"; else fail "requirements/vision.txt missing"; fi
if [ -f "$ROOT_DIR/requirements/ros2-adapter.txt" ]; then ok "ROS2 MQTT adapter requirements"; else fail "requirements/ros2-adapter.txt missing"; fi
if [ -f "$ROOT_DIR/models/parcel_box_yolo_model/best.onnx" ]; then ok "YOLO model best.onnx"; else fail "YOLO model missing"; fi

if [ -x "$ROOT_DIR/.venv/bin/python3" ]; then
    ok "project .venv"
    if "$ROOT_DIR/.venv/bin/python3" -c 'import onnxruntime, cv2, paho.mqtt.client' >/dev/null 2>&1; then
        ok ".venv vision + MQTT imports"
    else
        fail ".venv missing onnxruntime/cv2/paho-mqtt; run quick_setup.sh"
    fi
else
    warn "project .venv missing; run quick_setup.sh"
fi

if [ -f "$ROOT_DIR/ros2_ws/install/setup.bash" ]; then ok "ROS2 workspace built"; else warn "ros2_ws/install/setup.bash missing; run quick_setup.sh"; fi

printf '\n'
if [ "$FAILED" -eq 0 ]; then
    echo "Environment check completed without blocking errors."
    exit 0
fi

echo "Environment check found blocking prerequisites. See messages above."
exit 1
