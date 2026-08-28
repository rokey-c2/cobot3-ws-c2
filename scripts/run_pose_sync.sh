#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-111}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Keep discovery on localhost -- every ROS 2 node in this project is on this
# host (the Docker stack is MQTT/HTTP only). Set DISABLE_ROS_LOCALHOST_ONLY=1
# to talk to another machine or a container.
if [ "${DISABLE_ROS_LOCALHOST_ONLY:-0}" != "1" ]; then
    export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
fi

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
