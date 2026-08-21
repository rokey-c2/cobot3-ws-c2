#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd "$ROOT_DIR/ros2_ws"

if [ ! -f "install/setup.bash" ]; then
    echo "[ERROR] ROS 2 Workspace가 아직 빌드되지 않았습니다."
    echo "먼저 다음 명령을 실행하세요:"
    echo "  ./scripts/setup_ros.sh"
    exit 1
fi

source install/setup.bash

if ! ros2 pkg prefix amr_controller > /dev/null 2>&1; then
    echo "[ERROR] amr_controller 패키지가 빌드되지 않았습니다."
    echo "다음 명령을 다시 실행하세요:"
    echo "  ./scripts/setup_ros.sh"
    exit 1
fi

echo "[ROS2] /amr_a/goal_pose 자율주행 노드 시작"

exec ros2 launch \
    amr_controller \
    amr_navigation.launch.py \
    "$@"
