# Shared environment for every Isaac Sim launcher in this directory.
# MEANT TO BE SOURCED, not executed:
#
#   source "$(dirname "${BASH_SOURCE[0]}")/_isaac_env.sh"
#   exec "$ISAAC_SIM_DIR/python.sh" main_mission.py
#
# After sourcing, the caller can rely on:
#   ROOT_DIR         repo root
#   ISAAC_SIM_DIR    Isaac Sim install (override with the env var of the same name)
#   the working directory is $ROOT_DIR/isaac_sim
#
# This file used to be copy-pasted into run_isaac_mission.sh / run_isaac.sh /
# run_map_only.sh / run_logic_test.sh / run_full_pipeline_test.sh; keeping one
# copy stops them from drifting apart (run_isaac.sh had silently lost the ROS
# bridge / domain setup this way).

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/isaacsim}"
ROS2_BRIDGE_DIR="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/jazzy"

if [ ! -x "$ISAAC_SIM_DIR/python.sh" ]; then
    echo "[ERROR] Isaac Sim python.sh not found: $ISAAC_SIM_DIR/python.sh"
    echo "Set ISAAC_SIM_DIR if Isaac Sim is installed somewhere else."
    exit 1
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-111}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Every ROS 2 participant in this project (Isaac bridge, Nav2, pose sync,
# control-tower adapters, arm action server, YOLO node) runs on this host --
# the Docker stack is MQTT/HTTP only, no ROS inside it. Restricting discovery
# to localhost stops FastDDS from enumerating docker0 / br-* / veth*
# interfaces every run, which otherwise burns CPU on multicast discovery and
# shared-memory segments. Set DISABLE_ROS_LOCALHOST_ONLY=1 if a ROS 2 node
# ever needs to talk to another machine or a container.
if [ "${DISABLE_ROS_LOCALHOST_ONLY:-0}" != "1" ]; then
    export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
fi

export PYTHONPATH="$ROS2_BRIDGE_DIR/rclpy"
export LD_LIBRARY_PATH="$ROS2_BRIDGE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$ROOT_DIR/isaac_sim"
