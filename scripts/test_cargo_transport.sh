#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

echo "============================================"
echo " IW HUB FIXED LIFT + DELIVERY V4"
echo "============================================"
echo "[INFO] ROS2/Nav2/LiDAR logic are NOT used."
echo "[INFO] 1) rotate to +90 deg"
echo "[INFO] 2) move along Y under cargo_pod"
echo "[INFO] 3) restore lift position mode / KP / KD"
echo "[INFO] 4) lift to physical max (~0.04 m)"
echo "[INFO] 5) carry cargo to x=1.30104, y=-0.06065"
echo "[INFO] Cargo PhysicsColliders are visible."
echo "============================================"

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" tests/test_cargo_y_dock_delivery_v4.py
