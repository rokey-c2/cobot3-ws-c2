#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"

ROS2_BRIDGE_DIR="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/jazzy"

ISAAC_RCLPY_DIR="$ROS2_BRIDGE_DIR/rclpy"
ISAAC_ROS_LIB_DIR="$ROS2_BRIDGE_DIR/lib"

if [ ! -x "$ISAAC_SIM_DIR/python.sh" ]; then
    echo "[ERROR] Isaac Sim python.sh를 찾을 수 없습니다."
    echo "확인할 경로: $ISAAC_SIM_DIR/python.sh"
    exit 1
fi

if [ ! -f "$ISAAC_RCLPY_DIR/rclpy/__init__.py" ]; then
    echo "[ERROR] Isaac Sim 내부 rclpy를 찾을 수 없습니다."
    echo "확인할 경로: $ISAAC_RCLPY_DIR"
    exit 1
fi

remove_system_ros_paths() {
    local original_value="$1"
    local cleaned_value=""
    local path_item

    IFS=':' read -ra path_items <<< "$original_value"

    for path_item in "${path_items[@]}"; do
        if [ -z "$path_item" ]; then
            continue
        fi

        if [[ "$path_item" == /opt/ros/* ]]; then
            continue
        fi

        if [ -z "$cleaned_value" ]; then
            cleaned_value="$path_item"
        else
            cleaned_value="$cleaned_value:$path_item"
        fi
    done

    printf '%s' "$cleaned_value"
}

# 현재 터미널에 /opt/ros/jazzy가 source돼 있어도
# Isaac Python 3.11에는 전달하지 않는다.
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH

export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 기존 환경에서 /opt/ros 경로만 제거한다.
# CUDA처럼 ROS 2와 관계없는 경로는 그대로 보존한다.
CLEAN_PYTHONPATH="$(
    remove_system_ros_paths "${PYTHONPATH:-}"
)"

CLEAN_LD_LIBRARY_PATH="$(
    remove_system_ros_paths "${LD_LIBRARY_PATH:-}"
)"

# Isaac Sim에 포함된 Python 3.11용 ROS 2 경로를 먼저 사용한다.
export PYTHONPATH="$ISAAC_RCLPY_DIR"
export LD_LIBRARY_PATH="$ISAAC_ROS_LIB_DIR"

if [ -n "$CLEAN_PYTHONPATH" ]; then
    export PYTHONPATH="$PYTHONPATH:$CLEAN_PYTHONPATH"
fi

if [ -n "$CLEAN_LD_LIBRARY_PATH" ]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$CLEAN_LD_LIBRARY_PATH"
fi

cd "$ROOT_DIR/isaac_sim"

exec "$ISAAC_SIM_DIR/python.sh" main.py
