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

wait_for_publisher() {
    local topic_name="$1"

    echo "[ROS2] waiting for $topic_name"
    for _ in $(seq 1 40); do
        if ros2 topic info "$topic_name" 2>/dev/null | \
            grep -Eq 'Publisher count: [1-9][0-9]*'; then
            return 0
        fi
        sleep 0.5
    done

    echo "[ERROR] no publisher found for $topic_name"
    echo "Start Isaac Sim first: ./scripts/run_isaac.sh"
    return 1
}

wait_for_samples() {
    local topic_name="$1"
    local output

    echo "[ROS2] checking live samples on $topic_name"
    output="$(timeout 8 ros2 topic hz "$topic_name" --window 5 2>&1 || true)"
    if printf '%s\n' "$output" | grep -q 'average rate:'; then
        printf '%s\n' "$output" | grep -m1 'average rate:'
        return 0
    fi

    echo "[ERROR] publisher exists but no live samples arrived on $topic_name"
    echo "Keep Isaac Sim playing; do not start Nav2 with a silent LiDAR."
    return 1
}

wait_for_publisher /clock
wait_for_publisher /amr_a/scan
wait_for_publisher /amr_a/odom
wait_for_samples /amr_a/scan

exec ros2 launch amr_controller amr_nav2.launch.py "$@"
