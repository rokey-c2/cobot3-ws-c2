#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"
ROS2_BRIDGE_DIR="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/jazzy"

if [ ! -x "$ISAAC_SIM_DIR/python.sh" ]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-111}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ "${USE_FASTDDS_WHITELIST:-0}" != "1" ]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

export PYTHONPATH="$ROS2_BRIDGE_DIR/rclpy"
export LD_LIBRARY_PATH="$ROS2_BRIDGE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$ROOT_DIR/isaac_sim"
exec "$ISAAC_SIM_DIR/python.sh" test_full_pipeline.py
