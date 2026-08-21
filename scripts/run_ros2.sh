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

required_packages=(
    amr_controller
    nav2_bringup
    nav2_collision_monitor
    nav2_regulated_pure_pursuit_controller
    nav2_smac_planner
    tf2_ros
)

for package_name in "${required_packages[@]}"; do
    if ! ros2 pkg prefix "$package_name" > /dev/null 2>&1; then
        echo "[ERROR] 필요한 ROS 2 패키지가 없습니다: $package_name"
        echo "다음을 설치하고 workspace를 다시 빌드하세요:"
        echo "  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup"
        echo "  ./scripts/setup_ros.sh"
        exit 1
    fi
done

echo "[ROS2] ForkliftB LiDAR/Nav2 장애물 회피 시작"
echo "[ROS2] 시작 전 /clock, /amr_a/scan, /amr_a/odom을 확인하세요."

exec ros2 launch \
    amr_controller \
    amr_nav2.launch.py \
    "$@"
