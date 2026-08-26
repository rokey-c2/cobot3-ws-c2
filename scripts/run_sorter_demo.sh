#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

if [ ! -x "$ISAAC_SIM_DIR/python.sh" ]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" sorter_demo.py
