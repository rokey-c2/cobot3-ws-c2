#!/usr/bin/env bash
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/IsaacSim-ros_workspaces/jazzy_ws}"
FAILED=0

ok() { printf '[ OK ] %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1"; }
fail() { printf '[FAIL] %s\n' "$1"; FAILED=1; }

printf 'Control Tower environment check\n'
printf 'Repository: %s\n\n' "$ROOT_DIR"

if command -v git >/dev/null 2>&1; then ok "git $(git --version | awk '{print $3}')"; else fail "git not found"; fi
if command -v python3 >/dev/null 2>&1; then ok "python3 $(python3 --version 2>&1 | awk '{print $2}')"; else fail "python3 not found"; fi
if command -v docker >/dev/null 2>&1; then ok "docker installed"; else warn "docker not found (Backend/PostgreSQL/MQTT need Docker)"; fi

if command -v node >/dev/null 2>&1; then
    if node -e 'const [major, minor] = process.versions.node.split(".").map(Number); process.exit((major === 20 && minor >= 19) || (major === 22 && minor >= 12) || major > 22 ? 0 : 1)'; then
        ok "Node $(node -v)"
    else
        fail "Node 20.19+ or 22.12+ required; current $(node -v)"
    fi
else
    warn "node not found (Frontend needs Node 20.19+ or 22.12+)"
fi
if command -v npm >/dev/null 2>&1; then ok "npm $(npm -v)"; else warn "npm not found (Frontend setup cannot run)"; fi

if [ -f /opt/ros/jazzy/setup.bash ]; then ok "ROS 2 Jazzy"; else fail "ROS 2 Jazzy not found at /opt/ros/jazzy"; fi
if command -v rosdep >/dev/null 2>&1; then ok "rosdep installed"; else warn "rosdep not found"; fi
if command -v colcon >/dev/null 2>&1; then ok "colcon installed"; else warn "colcon not found"; fi

if [ -x "$ISAAC_SIM_DIR/python.sh" ]; then ok "Isaac Sim python.sh: $ISAAC_SIM_DIR/python.sh"; else fail "Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"; fi
if [ -f "$ISAAC_ROS_WS/install/setup.bash" ]; then ok "Isaac ROS Jazzy workspace: $ISAAC_ROS_WS"; else warn "Isaac ROS Jazzy workspace not found: $ISAAC_ROS_WS"; fi
if [ -d "$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/jazzy" ]; then ok "Isaac Sim ROS2 Jazzy bridge"; else warn "Isaac Sim ROS2 Jazzy bridge directory not found"; fi

if [ -f "$ROOT_DIR/backend/requirements.txt" ]; then ok "Backend requirements.txt"; else fail "backend/requirements.txt missing"; fi
if [ -f "$ROOT_DIR/frontend/package.json" ]; then ok "Frontend package.json"; else fail "frontend/package.json missing"; fi
if [ -f "$ROOT_DIR/frontend/package-lock.json" ]; then ok "Frontend package-lock.json"; else warn "frontend/package-lock.json missing; run npm install once and commit it"; fi
if [ -f "$ROOT_DIR/requirements/vision.txt" ]; then ok "Vision requirements"; else fail "requirements/vision.txt missing"; fi
if [ -f "$ROOT_DIR/requirements/ros2_adapter.txt" ]; then ok "ROS2 MQTT adapter requirements"; else fail "requirements/ros2_adapter.txt missing"; fi

printf '\n'
if [ "$FAILED" -eq 0 ]; then
    echo "Environment check completed without blocking errors."
    exit 0
fi

echo "Environment check found blocking prerequisites. See messages above."
exit 1
