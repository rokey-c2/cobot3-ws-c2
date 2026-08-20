#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd "$ROOT_DIR/ros2_ws"

echo "[TODO] Add package.xml / setup.py or CMakeLists.txt before colcon build."
