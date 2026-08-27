#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ -n "${VISION_PYTHON:-}" ]; then
    PYTHON_BIN="$VISION_PYTHON"
elif [ -x "$ROOT_DIR/.venv/bin/python" ]; then
    PYTHON_BIN="$ROOT_DIR/.venv/bin/python"
else
    PYTHON_BIN="python3"
fi

if ! "$PYTHON_BIN" -c 'import cv2, numpy, rclpy' >/dev/null 2>&1; then
    echo "[ERROR] $PYTHON_BIN needs cv2, numpy and ROS2 rclpy."
    exit 1
fi

if ! "$PYTHON_BIN" -c 'import onnxruntime' >/dev/null 2>&1; then
    echo "[ERROR] onnxruntime is missing from $PYTHON_BIN."
    echo "        Use the project .venv or set VISION_PYTHON to the Python that has onnxruntime."
    exit 1
fi

PIDS=()
cleanup() {
    trap - INT TERM EXIT
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

start_node() {
    local label="$1"
    shift
    echo "[VISION] starting $label"
    "$@" &
    PIDS+=("$!")
}

resolve_amr_camera_topic() {
    if [ -n "${AMR_CAMERA_TOPIC:-}" ]; then
        printf '%s\n' "$AMR_CAMERA_TOPIC"
        return
    fi

    local candidates=(
        "/front_stereo_camera/left/image_raw"
        "/front_stereo_camera/left/image_rect_color"
        "/front_stereo_camera/left_rgb/image_raw"
    )
    local topics=""
    local candidate=""
    local discovered=""

    # Isaac's IW Hub camera graph can appear a moment after the mission READY
    # banner. Wait briefly and accept the topic names used across Isaac Sim
    # sensor asset revisions instead of hard-coding only one variant.
    for _ in $(seq 1 20); do
        topics="$(ros2 topic list 2>/dev/null || true)"

        for candidate in "${candidates[@]}"; do
            if grep -Fxq "$candidate" <<<"$topics"; then
                printf '%s\n' "$candidate"
                return
            fi
        done

        discovered="$(
            grep -E '^/front_stereo_camera/.+(image_raw|image_rect_color)$' <<<"$topics" \
                | head -n 1 || true
        )"
        if [ -n "$discovered" ]; then
            printf '%s\n' "$discovered"
            return
        fi

        sleep 0.5
    done

    # Subscribers may still connect later. Isaac Sim 5.x commonly uses this
    # topic for the left front stereo RGB camera.
    printf '%s\n' "/front_stereo_camera/left/image_raw"
}

cd "$ROOT_DIR"
AMR_CAMERA_TOPIC_RESOLVED="$(resolve_amr_camera_topic)"

echo "[VISION] AMR camera topic: $AMR_CAMERA_TOPIC_RESOLVED"

start_node "AMR front camera :8090" \
    "$PYTHON_BIN" isaac_sim/vision/top_view_stream_node.py \
    --ros-args \
    --remap __node:=control_tower_amr_camera_stream \
    -p image_topic:="$AMR_CAMERA_TOPIC_RESOLVED" \
    -p stream_port:=8090 \
    -p jpeg_quality:=78

start_node "Warehouse Top View :8092" \
    "$PYTHON_BIN" isaac_sim/vision/top_view_stream_node.py \
    --ros-args \
    --remap __node:=control_tower_top_view_stream \
    -p image_topic:=/top_view/rgb \
    -p stream_port:=8092 \
    -p jpeg_quality:=78

start_node "P3020 IN laser vision :8091" \
    "$PYTHON_BIN" isaac_sim/robots/p3020/vision/box_detector_node.py \
    --ros-args \
    --remap __node:=p3020_in_box_detector \
    -p image_topic:=/rgb \
    -p box_pixel_topic:=/box_pixel \
    -p annotated_image_topic:=/p3020/in/vision/image_annotated \
    -p stream_port:=8091

start_node "P3020 OUT laser vision :8093" \
    "$PYTHON_BIN" isaac_sim/robots/p3020/vision/box_detector_node.py \
    --ros-args \
    --remap __node:=p3020_out_box_detector \
    -p image_topic:=/arm_b/rgb \
    -p box_pixel_topic:=/arm_b/box_pixel \
    -p annotated_image_topic:=/p3020/out/vision/image_annotated \
    -p stream_port:=8093

echo "[VISION] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[VISION] AMR       http://<this-pc-ip>:8090/stream.mjpg"
echo "[VISION] P3020 IN  http://<this-pc-ip>:8091/stream.mjpg"
echo "[VISION] TOP VIEW  http://<this-pc-ip>:8092/stream.mjpg"
echo "[VISION] P3020 OUT http://<this-pc-ip>:8093/stream.mjpg"
echo "[VISION] Ctrl+C stops all four stream processes."

wait -n "${PIDS[@]}"
status=$?
echo "[VISION] one stream process exited (status=$status); stopping the group."
exit "$status"
