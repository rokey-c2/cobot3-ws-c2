#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_FILE="$ROOT_DIR/isaac_sim/usd/Parcel_Sorting_Map_real_real_final_final/navigation/maps/Parcel_Sorting_Map.yaml"

LOADED_FOOTPRINT="[[0.70, 0.55], [0.70, -0.55], [-0.80, -0.55], [-0.80, 0.55]]"

# About 3x the previous Nav2 travel speed (0.65 -> 1.95 m/s).
NAV2_DESIRED_LINEAR_SPEED="1.95"
NAV2_MAX_LINEAR_SPEED="2.40"
NAV2_MAX_LINEAR_ACCEL="2.40"
NAV2_MAX_LINEAR_DECEL="-3.00"

source /opt/ros/jazzy/setup.bash

unset GTK_PATH
unset GIO_MODULE_DIR

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-111}"
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

set_loaded_footprint() {
    echo "[ROS2] setting loaded cargo footprint: $LOADED_FOOTPRINT"
    for node_name in /local_costmap/local_costmap /global_costmap/global_costmap; do
        ready=0
        for _ in $(seq 1 30); do
            if ros2 param list "$node_name" 2>/dev/null | \
                sed 's/^ *//' | grep -qx 'footprint'; then
                ready=1
                break
            fi
            sleep 0.5
        done
        if [ "$ready" != "1" ]; then
            echo "[ERROR] footprint parameter not available on $node_name"
            return 1
        fi
        ros2 param set "$node_name" footprint "$LOADED_FOOTPRINT"
    done
}

set_param_when_ready() {
    local node_name="$1"
    local param_name="$2"
    local param_value="$3"

    for _ in $(seq 1 40); do
        if ros2 param list "$node_name" 2>/dev/null | \
            sed 's/^ *//' | grep -qx "$param_name"; then
            if ros2 param set "$node_name" "$param_name" "$param_value" >/dev/null; then
                echo "[ROS2] $node_name $param_name = $param_value"
                return 0
            fi
        fi
        sleep 0.5
    done

    echo "[WARN] could not apply parameter: $node_name $param_name"
    return 0
}

set_nav2_speed() {
    echo "[ROS2] applying ~3x Nav2 linear speed"
    set_param_when_ready \
        /controller_server FollowPath.desired_linear_vel \
        "$NAV2_DESIRED_LINEAR_SPEED"
    set_param_when_ready \
        /velocity_smoother max_velocity \
        "[$NAV2_MAX_LINEAR_SPEED, 0.0, 0.90]"
    set_param_when_ready \
        /velocity_smoother max_accel \
        "[$NAV2_MAX_LINEAR_ACCEL, 0.0, 1.50]"
    set_param_when_ready \
        /velocity_smoother max_decel \
        "[$NAV2_MAX_LINEAR_DECEL, 0.0, -1.80]"
}

wait_for_publisher /clock
wait_for_publisher /front_2d_lidar/scan
wait_for_publisher /back_2d_lidar/scan
wait_for_publisher /chassis/odom

wait_for_samples /front_2d_lidar/scan
wait_for_samples /back_2d_lidar/scan

printf '\n[ROS2] Default IW Hub Sensor topics are alive.\n'
printf '[ROS2] Starting NVIDIA iw_hub_navigation with the project map.\n'
printf '[ROS2] NVIDIA LiDAR pose/rate/resolution are unchanged.\n\n'

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

set_loaded_footprint
set_nav2_speed

echo "[ROS2] Nav2 is ready."
echo "[ROS2] /initialpose is owned by scripts/pose_sync_manager.py."
echo "[ROS2] Start ./scripts/run_pose_sync.sh to initialize AMCL from Isaac."

wait "$NAV2_PID"
