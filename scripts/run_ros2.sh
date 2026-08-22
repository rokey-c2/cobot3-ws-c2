#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_FILE="$ROOT_DIR/isaac_sim/usd/enva_small_warehouse_p3020_marker/navigation/maps/warehouse_navigation.yaml"
PARAMS_FILE="$ROOT_DIR/isaac_sim/usd/enva_small_warehouse_p3020_marker/navigation/params/iw_hub_navigation_params_custom.yaml"

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

# NVIDIA IW Hub navigation is normally built in the Isaac Sim Jazzy workspace.
# Source it automatically when it is not already visible in this shell.
if ! ros2 pkg prefix iw_hub_navigation >/dev/null 2>&1; then
    ISAAC_JAZZY_SETUP="$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash"
    if [ -f "$ISAAC_JAZZY_SETUP" ]; then
        source "$ISAAC_JAZZY_SETUP"
    fi
fi

required_packages=(
    amr_controller
    iw_hub_navigation
    tf2_ros
)
for package_name in "${required_packages[@]}"; do
    if ! ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
        echo "[ERROR] missing ROS 2 package: $package_name"
        exit 1
    fi
done

if [ ! -f "$MAP_FILE" ]; then
    echo "[ERROR] map not found: $MAP_FILE"
    exit 1
fi
if [ ! -f "$PARAMS_FILE" ]; then
    echo "[ERROR] Nav2 params not found: $PARAMS_FILE"
    exit 1
fi

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
    return 1
}

wait_for_publisher /clock
wait_for_publisher /amr_a/scan_raw
wait_for_publisher /amr_a/odom
wait_for_samples /amr_a/scan_raw

BRIDGE_PIDS=()
cleanup() {
    for pid in "${BRIDGE_PIDS[@]:-}"; do
        kill "$pid" >/dev/null 2>&1 || true
    done
}
trap cleanup EXIT INT TERM

# Keep Isaac's raw scan untouched for diagnostics and publish only the filtered
# stream on /amr_a/scan, which is what AMCL/costmaps/collision_monitor consume.
ros2 run amr_controller scan_self_filter --ros-args -p use_sim_time:=true &
BRIDGE_PIDS+=("$!")

# Alias Isaac's namespaced ground-truth odometry into the single-robot Nav2 TF
# convention: odom -> base_link.
ros2 run amr_controller odom_tf_bridge --ros-args -p use_sim_time:=true &
BRIDGE_PIDS+=("$!")

wait_for_publisher /amr_a/scan
wait_for_samples /amr_a/scan

sleep 1

echo "[ROS2] starting NVIDIA IW Hub Nav2"
echo "[ROS2] map:    $MAP_FILE"
echo "[ROS2] params: $PARAMS_FILE"

ros2 launch iw_hub_navigation iw_hub_navigation.launch.py \
    map:="$MAP_FILE" \
    params_file:="$PARAMS_FILE" \
    "$@"
