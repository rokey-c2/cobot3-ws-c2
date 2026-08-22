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
echo " AMR -> P3020 MISSION"
echo "============================================"
echo "[1] Nav2 PRE_DOCK"
echo "[2] local dock + lift"
echo "[3] Nav2 delivery: (1.30104, -0.06065)"
echo "[4] P3020 fixed-pose PickPlace"
echo "[5] lift down"
echo "[6] Nav2 home"
echo
echo "[TEST DEFAULT] simulate_p3020:=true"
echo "============================================"

exec python3 \
    "$ROOT_DIR/ros2_ws/src/amr_controller/amr_controller/amr_p3020_mission.py" \
    "$@"
