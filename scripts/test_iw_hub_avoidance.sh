#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
source "$ROOT_DIR/ros2_ws/install/setup.bash"

ACTION_NAME="/amr_a/navigate_to_pose"
LIFECYCLE_NODE="/amr_a/bt_navigator"

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
    echo "[ERROR] current lifecycle state: ${lifecycle_state:-unavailable}"
    echo "Check terminal 2 for lifecycle or TF errors."
    exit 1
fi

echo "[TEST] waiting for $ACTION_NAME"
for _ in $(seq 1 30); do
    if ros2 action list | grep -qx "$ACTION_NAME"; then
        break
    fi
    sleep 1
done

if ! ros2 action list | grep -qx "$ACTION_NAME"; then
    echo "[ERROR] Nav2 action is not available: $ACTION_NAME"
    echo "Run first: ./scripts/run_ros2.sh"
    exit 1
fi

ros2 topic pub --once \
    /amr_a/navigation_enabled \
    std_msgs/msg/Bool \
    "{data: true}"

echo "[TEST] start=(1.5, 0.0), obstacle=(3.5, 0.0), goal=(6.0, 0.0)"
exec ros2 action send_goal \
    "$ACTION_NAME" \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 6.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
    --feedback
