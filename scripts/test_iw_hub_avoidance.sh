#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

GOAL_X="1.0015223026275635"
GOAL_Y="0.018860459327697754"

source /opt/ros/jazzy/setup.bash
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

ACTION_NAME="/navigate_to_pose"
LIFECYCLE_NODE="/bt_navigator"

echo "[TEST] waiting for $LIFECYCLE_NODE to become active"
lifecycle_state=""
for _ in $(seq 1 60); do
    lifecycle_state="$(ros2 lifecycle get "$LIFECYCLE_NODE" 2>/dev/null || true)"
    if printf '%s\n' "$lifecycle_state" | grep -q "active"; then
        break
    fi
    sleep 1
done

if ! printf '%s\n' "$lifecycle_state" | grep -q "active"; then
    echo "[ERROR] Nav2 did not become active: $LIFECYCLE_NODE"
    exit 1
fi

if ! ros2 action list | grep -qx "$ACTION_NAME"; then
    echo "[ERROR] Nav2 action is not available: $ACTION_NAME"
    exit 1
fi

if ! timeout 10 ros2 topic echo /map --once \
    --qos-reliability reliable \
    --qos-durability transient_local >/dev/null 2>&1; then
    echo "[ERROR] no static map received on /map"
    exit 1
fi

for lidar_topic in /front_2d_lidar/scan /back_2d_lidar/scan; do
    scan_rate="$(timeout 8 ros2 topic hz "$lidar_topic" --window 5 2>&1 || true)"
    if ! printf '%s\n' "$scan_rate" | grep -q 'average rate:'; then
        echo "[ERROR] default IW Hub LiDAR has no live samples: $lidar_topic"
        exit 1
    fi
    printf '%s: ' "$lidar_topic"
    printf '%s\n' "$scan_rate" | grep -m1 'average rate:'
done

echo "[TEST] default IW Hub Sensor navigation"
echo "[TEST] goal=($GOAL_X, $GOAL_Y)"

exec ros2 action send_goal \
    "$ACTION_NAME" \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
    --feedback
