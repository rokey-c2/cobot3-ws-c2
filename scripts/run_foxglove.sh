#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
FOXGLOVE_ADDRESS="${FOXGLOVE_ADDRESS:-0.0.0.0}"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

cd "$ROOT_DIR/ros2_ws"

if [ ! -f "install/setup.bash" ]; then
    echo "[ERROR] ROS 2 Workspace가 아직 빌드되지 않았습니다."
    echo "먼저 실행하세요: ./scripts/setup_ros.sh"
    exit 1
fi

source install/setup.bash

for package_name in foxglove_bridge amr_controller; do
    if ! ros2 pkg prefix "$package_name" > /dev/null 2>&1; then
        echo "[ERROR] 필요한 ROS 2 패키지가 없습니다: $package_name"
        if [ "$package_name" = "foxglove_bridge" ]; then
            echo "설치: sudo apt install ros-jazzy-foxglove-bridge"
        else
            echo "빌드: ./scripts/setup_ros.sh"
        fi
        exit 1
    fi
done

if ! [[ "$FOXGLOVE_PORT" =~ ^[0-9]+$ ]] \
    || [ "$FOXGLOVE_PORT" -lt 1 ] \
    || [ "$FOXGLOVE_PORT" -gt 65535 ]; then
    echo "[ERROR] FOXGLOVE_PORT는 1~65535 범위의 정수여야 합니다."
    exit 1
fi

echo "[FOXGLOVE] Bridge 시작"
echo "[FOXGLOVE] Local: ws://localhost:$FOXGLOVE_PORT"
echo "[FOXGLOVE] Remote: ws://<이 PC의 IP>:$FOXGLOVE_PORT"
echo "[FOXGLOVE] IP 확인: hostname -I"

exec ros2 launch \
    amr_controller \
    foxglove_bringup.launch.py \
    port:="$FOXGLOVE_PORT" \
    address:="$FOXGLOVE_ADDRESS" \
    "$@"
