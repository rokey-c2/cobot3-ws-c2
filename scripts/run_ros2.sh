#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_FILE="$ROOT_DIR/isaac_sim/usd/warehouse_final_final/navigation/maps/warehouse_navigation.yaml"

# Must match the verified Isaac spawn transform.
START_X="10.5"
START_Y="1.80122"

source /opt/ros/jazzy/setup.bash

unset GTK_PATH
unset GIO_MODULE_DIR

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ "${USE_FASTDDS_WHITELIST:-0}" != "1" ]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

if ! ros2 pkg prefix iw_hub_navigation >/dev/null 2>&1; then
    ISAAC_JAZZY_SETUP="$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash"
    if [ -f "$ISAAC_JAZZY_SETUP" ]; then
        source "$ISAAC_JAZZY_SETUP"
    fi
fi

if ! ros2 pkg prefix iw_hub_navigation >/dev/null 2>&1; then
    echo "[ERROR] iw_hub_navigation package not found"
    echo "Expected: $HOME/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash"
    exit 1
fi

if [ ! -f "$MAP_FILE" ]; then
    echo "[ERROR] map not found: $MAP_FILE"
    exit 1
fi

wait_for_publisher() {
    local topic_name="$1"

    echo "[ROS2] waiting for $topic_name"
    for _ in $(seq 1 60); do
        if ros2 topic info "$topic_name" 2>/dev/null | \
            grep -Eq 'Publisher count: [1-9][0-9]*'; then
            return 0
        fi
        sleep 0.5
    done

    echo "[ERROR] no publisher found for $topic_name"
    echo "[ERROR] Start Isaac Sim first with ./scripts/run_isaac_mission.sh"
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
wait_for_publisher /front_2d_lidar/scan
wait_for_publisher /back_2d_lidar/scan
wait_for_publisher /chassis/odom

wait_for_samples /front_2d_lidar/scan
wait_for_samples /back_2d_lidar/scan

printf '\n[ROS2] Default IW Hub Sensor topics are alive.\n'
printf '[ROS2] Starting NVIDIA iw_hub_navigation with the project map.\n'
printf '[ROS2] No custom LiDAR, scan filter, or odom TF bridge is used.\n\n'

ros2 launch iw_hub_navigation iw_hub_navigation.launch.py \
    map:="$MAP_FILE" \
    "$@" &
NAV2_PID=$!

cleanup() {
    kill "$NAV2_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[ROS2] waiting for AMCL"
for _ in $(seq 1 60); do
    amcl_state="$(ros2 lifecycle get /amcl 2>/dev/null || true)"
    if printf '%s\n' "$amcl_state" | grep -q 'active'; then
        break
    fi
    sleep 1
done

if ! printf '%s\n' "${amcl_state:-}" | grep -q 'active'; then
    echo "[ERROR] AMCL did not become active"
    wait "$NAV2_PID"
    exit 1
fi

sleep 1

echo "[ROS2] setting initial pose: ($START_X, $START_Y), yaw=0 deg"
ros2 topic pub --once \
    /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $START_X, y: $START_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

wait "$NAV2_PID"
