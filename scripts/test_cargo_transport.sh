#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

echo "============================================"
echo " CORRECTED IW HUB Y-AXIS DOCK + LIFT TEST"
echo "============================================"
echo "[INFO] ROS2/Nav2/LiDAR logic are NOT used."
echo "[INFO] Robot X lane: 10.53654"
echo "[INFO] 1) rotate to +90 deg"
echo "[INFO] 2) move only along Y to the computed under-pod pose"
echo "[INFO] 3) verify lift center against cargo center"
echo "[INFO] 4) lift up to 0.04 m"
echo "[INFO] Previous y=-2.0846 was not the lift-aligned pose."
echo "[INFO] Cargo PhysicsColliders are visible."
echo "============================================"

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" tests/test_cargo_y_dock_lift.py
