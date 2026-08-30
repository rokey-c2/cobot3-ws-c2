#!/usr/bin/env bash
# Lightweight map_server + rviz2, deliberately WITHOUT AMCL/Nav2.
#
# Purpose: display Final.yaml's occupancy grid in rviz2 so the
# user can read off real-world (map-frame) coordinates for AMR/cargo/parcel
# placement. The full run_ros2.sh stack requires a live robot publishing
# /clock + lidar + odom and a valid initial pose to bring AMCL up -- none of
# that exists yet for the new map, so this script only brings up map_server.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_FILE="$ROOT_DIR/isaac_sim/usd/Final_Real_Map/navigation/maps/Final.yaml"
RVIZ_CONFIG="$ROOT_DIR/rviz/map_view.rviz"

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ "${USE_FASTDDS_WHITELIST:-0}" != "1" ]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

if [ ! -f "$MAP_FILE" ]; then
    echo "[ERROR] map not found: $MAP_FILE"
    exit 1
fi

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP_FILE" &
MAP_SERVER_PID=$!

cleanup() {
    kill "$MAP_SERVER_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[MAP_SERVER] waiting for lifecycle service"
for _ in $(seq 1 30); do
    if ros2 lifecycle get /map_server >/dev/null 2>&1; then
        break
    fi
    sleep 0.5
done

map_server_state() {
    ros2 lifecycle get /map_server 2>/dev/null | awk '{print $1}'
}

state="$(map_server_state)"
if [ "$state" = "unconfigured" ]; then
    ros2 lifecycle set /map_server configure || true
    state="$(map_server_state)"
fi
if [ "$state" != "active" ]; then
    ros2 lifecycle set /map_server activate || true
    state="$(map_server_state)"
fi

if [ "$state" != "active" ]; then
    echo "[ERROR] map_server did not reach active state (current: $state)"
    exit 1
fi

echo "[MAP_SERVER] active, publishing /map from $MAP_FILE"

if [ -f "$RVIZ_CONFIG" ]; then
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
else
    ros2 run rviz2 rviz2 &
fi
RVIZ_PID=$!

wait "$RVIZ_PID"
