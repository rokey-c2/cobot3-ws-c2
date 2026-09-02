#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ ! -x "$ROOT_DIR/.venv/bin/python3" ]; then
    echo "[ERROR] .venv/bin/python3 not found"
    echo "[ERROR] Run ./scripts/setup_adapter_env.sh first."
    exit 1
fi

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "[CONTROL TOWER] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[CONTROL TOWER] Starting AMR + manual + process adapters (Nav2 launch not required)"

cleanup() {
    jobs -pr | xargs -r kill 2>/dev/null || true
    wait || true
}
trap cleanup EXIT INT TERM

"$ROOT_DIR/.venv/bin/python3" "${ROOT_DIR}/scripts/ros2_mqtt_adapter.py" &
"$ROOT_DIR/.venv/bin/python3" "${ROOT_DIR}/scripts/ros2_manual_mqtt_adapter.py" &
"$ROOT_DIR/.venv/bin/python3" "${ROOT_DIR}/scripts/process_mqtt_adapter.py" &

wait -n
