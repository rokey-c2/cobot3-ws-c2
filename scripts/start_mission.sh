#!/usr/bin/env bash
# Start the real AMR -> P3020 integrated mission with simulation fallback off.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [ -f "$ROOT_DIR/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROOT_DIR/ros2_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

cd "$ROOT_DIR"

if [ "$#" -gt 0 ]; then
    exec ./scripts/run_amr_p3020_mission.sh "$@"
fi

exec ./scripts/run_amr_p3020_mission.sh \
    --ros-args \
    -p simulate_p3020:=false \
    -p pickup_x:=1.5 \
    -p pickup_y:=-2.0 \
    -p place_x:=-0.5 \
    -p place_y:=0.0
