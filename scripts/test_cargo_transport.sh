#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

echo "============================================"
echo " IW HUB SCRIPTED CARGO TRANSPORT TEST"
echo "============================================"
echo "[INFO] ROS2/Nav2/LiDAR are NOT started."
echo "[INFO] Sequence: START -> cargo -> lift up -> goal"
echo "[INFO] Goal: x=1.30104, y=-0.06065"
echo "============================================"

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" tests/test_cargo_transport.py
