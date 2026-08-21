#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [ "${USE_FASTDDS_WHITELIST:-0}" != "1" ]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
fi
source "$ROOT_DIR/ros2_ws/install/setup.bash"

ACTION_NAME="/amr_a/navigate_to_pose"
LIFECYCLE_NODE="/amr_a/bt_navigator"

echo "[MISSION] waiting for active Nav2"
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
    --qos-durability transient_local > /dev/null 2>&1; then
    echo "[ERROR] no static map received on /map"
    exit 1
fi

echo "[MISSION] pickup=(1.5, 0.0), obstacle=(3.5, 0.0), place=(6.0, 0.0)"
echo "[MISSION] lift 0.00 -> 0.30 m, navigate, lift 0.30 -> 0.00 m"
exec ros2 run amr_controller container_mission --ros-args \
    -r __ns:=/amr_a \
    -p use_sim_time:=true \
    -p goal_x:=6.0 \
    -p goal_y:=0.0 \
    -p lift_height:=0.30 \
    -p lift_duration:=4.0
