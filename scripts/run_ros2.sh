#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [ "${USE_FASTDDS_WHITELIST:-0}" != "1" ]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

if [ ! -f "$ROOT_DIR/ros2_ws/install/setup.bash" ]; then
    echo "[ERROR] ROS 2 workspace is not built."
    echo "Run first: ./scripts/setup_ros.sh"
    exit 1
fi
source "$ROOT_DIR/ros2_ws/install/setup.bash"

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
        echo "[ERROR] missing ROS 2 package: $package_name"
        echo "Install: sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup"
        exit 1
    fi
done

for topic_name in /clock /amr_a/scan /amr_a/odom; do
    echo "[ROS2] waiting for $topic_name"
    if ! timeout 10 ros2 topic echo "$topic_name" --once \
        --qos-reliability best_effort \
        --qos-durability volatile > /dev/null 2>&1; then
        echo "[ERROR] no message received from $topic_name"
        echo "Start Isaac Sim first: ./scripts/run_isaac.sh"
        exit 1
    fi
done

exec ros2 launch amr_controller amr_nav2.launch.py "$@"
