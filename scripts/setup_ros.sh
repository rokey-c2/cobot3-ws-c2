#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd "$ROOT_DIR/ros2_ws"

echo "[ROS2] amr_controller 패키지 빌드 시작"

colcon build \
    --symlink-install \
    --packages-select amr_controller

source install/setup.bash

echo "[ROS2] 빌드 완료"
echo "[ROS2] Package: $(ros2 pkg prefix amr_controller)"
