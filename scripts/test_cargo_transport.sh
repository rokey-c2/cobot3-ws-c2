#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

echo "============================================"
echo " IW HUB DOCK + LIFT DIAGNOSTIC + DELIVERY"
echo "============================================"
echo "[INFO] ROS2/Nav2/LiDAR logic are NOT used."
echo "[INFO] 1) rotate to +90 deg"
echo "[INFO] 2) move on x=10.5 lane under cargo_pod"
echo "[INFO] 3) verify lift itself moves to physical max"
echo "[INFO] 4) verify cargo actually rises"
echo "[INFO] 5) only then carry to x=1.30104, y=-0.06065"
echo "[INFO] Cargo PhysicsColliders are visible."
echo "============================================"

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" tests/test_cargo_y_dock_delivery_v2.py
