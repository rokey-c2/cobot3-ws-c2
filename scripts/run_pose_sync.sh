#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ ! -x "$ROOT_DIR/.venv/bin/python3" ]; then
    echo "[ERROR] .venv/bin/python3 not found"
    echo "[ERROR] Run the project environment setup first."
    exit 1
fi

echo "[POSE SYNC] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[POSE SYNC] Waiting for Isaac /amr_a/map_pose"
echo "[POSE SYNC] Canonical frame: map"

exec "$ROOT_DIR/.venv/bin/python3" \
    "$ROOT_DIR/scripts/pose_sync_manager.py"
