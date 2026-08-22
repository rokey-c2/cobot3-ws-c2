#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

echo "============================================"
echo " IW HUB Y-AXIS APPROACH + LIFT TEST"
echo "============================================"
echo "[INFO] ROS2/Nav2/LiDAR logic are NOT used."
echo "[INFO] 1) START -> rotate +90 deg"
echo "[INFO] 2) Move only on Y to y=-2.0846"
echo "[INFO] 3) Lift up to 0.04 m"
echo "[INFO] Screenshot X reference: 10.53654"
echo "[INFO] Cargo PhysicsColliders are visible."
echo "============================================"

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" tests/test_cargo_transport_simple.py
