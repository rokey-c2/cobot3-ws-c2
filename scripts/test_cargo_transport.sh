#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

echo "============================================"
echo " IW HUB PICKUP + DELIVERY TEST"
echo "============================================"
echo "[INFO] ROS2/Nav2/LiDAR logic are NOT used."
echo "[INFO] 1) rotate to +90 deg"
echo "[INFO] 2) move only along Y under cargo_pod"
echo "[INFO] 3) lift to authored physical maximum (~0.04 m)"
echo "[INFO] 4) carry cargo to x=1.30104, y=-0.06065"
echo "[INFO] Cargo PhysicsColliders are visible."
echo "============================================"

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" tests/test_cargo_y_dock_lift.py
