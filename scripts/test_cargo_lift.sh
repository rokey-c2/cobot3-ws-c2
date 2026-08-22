#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

echo "============================================"
echo " IW HUB CARGO LIFT TEST"
echo "============================================"
echo "[INFO] ROS2/Nav2 is not started by this script."
echo "[INFO] Running only Isaac Sim physics + lift_joint test."
echo "============================================"

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" tests/test_cargo_lift.py
