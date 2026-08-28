#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=111
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Keep discovery on localhost -- every ROS 2 node in this project is on this
# host (the Docker stack is MQTT/HTTP only). Set DISABLE_ROS_LOCALHOST_ONLY=1
# to talk to another machine or a container.
if [ "${DISABLE_ROS_LOCALHOST_ONLY:-0}" != "1" ]; then
    export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
fi

echo "ROS2 Jazzy configured"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
