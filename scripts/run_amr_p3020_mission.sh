#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ "${USE_FASTDDS_WHITELIST:-0}" != "1" ]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

if [ -f "$ROOT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$ROOT_DIR/ros2_ws/install/setup.bash"
fi

if ! ros2 interface show logistics_interfaces/action/PickPlace >/dev/null 2>&1; then
    echo "[ERROR] logistics_interfaces/action/PickPlace is not built."
    echo "Run:"
    echo "  cd $ROOT_DIR/ros2_ws"
    echo "  colcon build --symlink-install"
    echo "  source install/setup.bash"
    exit 1
fi

echo "============================================"
echo " IW HUB -> P3020 FULL MISSION"
echo "============================================"
echo "[1] local rotate +90 deg"
echo "[2] local drive -> (9, -3)"
echo "[3] lift up -> PICKUP_DONE"
echo "[4] Nav2 delivery -> (1.1, -1.5)"
echo "[5] P3020 PickPlace action"
echo "[6] Nav2 return near cargo"
echo "[7] local dock -> (9, -3, yaw=90 deg)"
echo "[8] lift down + cargo pose verification"
echo "[9] local return -> (9, -6, yaw=0 deg)"
echo
echo "[TEST DEFAULT] simulate_p3020:=true"
echo "[MANUAL DELIVERY] add -p manual_delivery:=true to wait for the Dashboard arrival button"
echo "============================================"

exec python3 \
    "$ROOT_DIR/ros2_ws/src/amr_controller/amr_controller/amr_p3020_mission.py" \
    "$@"
