#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "[CONTROL TOWER] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[CONTROL TOWER] Starting AMR adapter + process adapter (Nav2 launch not required)"

cleanup() {
    jobs -pr | xargs -r kill 2>/dev/null || true
    wait || true
}
trap cleanup EXIT INT TERM

python3 "${ROOT_DIR}/scripts/ros2_mqtt_adapter.py" &
python3 "${ROOT_DIR}/scripts/process_mqtt_adapter.py" &

wait -n
