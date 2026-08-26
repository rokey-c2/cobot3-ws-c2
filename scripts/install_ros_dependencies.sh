#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
ROS_SRC="$ROOT_DIR/ros2_ws/src"

if [ ! -f "$ROS_SETUP" ]; then
    echo "[ERROR] ROS 2 $ROS_DISTRO is not installed: $ROS_SETUP"
    echo "Install ROS 2 $ROS_DISTRO first, then run this script again."
    exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
    echo "[ERROR] rosdep is not installed"
    echo "Ubuntu example: sudo apt install -y python3-rosdep"
    exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
    echo "[ERROR] colcon is not installed"
    echo "Ubuntu example: sudo apt install -y python3-colcon-common-extensions"
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

echo "[ROS2] distro: $ROS_DISTRO"
echo "[ROS2] installing dependencies declared by package.xml files"
rosdep install \
    --from-paths "$ROS_SRC" \
    --ignore-src \
    -r -y \
    --rosdistro "$ROS_DISTRO"

echo "[ROS2] dependency setup complete"
echo "[ROS2] Note: NVIDIA iw_hub_navigation comes from the separate Isaac Sim ROS workspace."
