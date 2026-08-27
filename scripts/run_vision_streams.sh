#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

export ORT_INTRA_OP_NUM_THREADS="${ORT_INTRA_OP_NUM_THREADS:-2}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-2}"

AMR_STREAM_FPS="${AMR_STREAM_FPS:-6.0}"
TOP_VIEW_STREAM_FPS="${TOP_VIEW_STREAM_FPS:-5.0}"
P3020_DETECTION_FPS="${P3020_DETECTION_FPS:-5.0}"
P3020_STREAM_FPS="${P3020_STREAM_FPS:-6.0}"
VISION_JPEG_QUALITY="${VISION_JPEG_QUALITY:-65}"
VISION_CPU_THREADS="${VISION_CPU_THREADS:-2}"

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

cd "$ROOT_DIR"

start_node "AMR camera :8090" \
    "$PYTHON_BIN" isaac_sim/vision/top_view_stream_node.py \
    --ros-args \
    --remap __node:=control_tower_amr_camera_stream \
    -p image_topic:=/amr_a/camera/rgb \
    -p stream_port:=8090 \
    -p max_fps:="$AMR_STREAM_FPS" \
    -p max_width:=640 \
    -p jpeg_quality:="$VISION_JPEG_QUALITY"

start_node "Warehouse Top View :8092" \
    "$PYTHON_BIN" isaac_sim/vision/top_view_stream_node.py \
    --ros-args \
    --remap __node:=control_tower_top_view_stream \
    -p image_topic:=/top_view/rgb \
    -p stream_port:=8092 \
    -p max_fps:="$TOP_VIEW_STREAM_FPS" \
    -p max_width:=960 \
    -p jpeg_quality:="$VISION_JPEG_QUALITY"

start_node "P3020 IN laser vision :8091" \
    "$PYTHON_BIN" isaac_sim/robots/p3020/vision/box_detector_node.py \
    --ros-args \
    --remap __node:=p3020_in_box_detector \
    -p image_topic:=/rgb \
    -p box_pixel_topic:=/box_pixel \
    -p annotated_image_topic:=/p3020/in/vision/image_annotated \
    -p stream_port:=8091 \
    -p detection_fps:="$P3020_DETECTION_FPS" \
    -p stream_fps:="$P3020_STREAM_FPS" \
    -p stream_max_width:=640 \
    -p jpeg_quality:="$VISION_JPEG_QUALITY" \
    -p publish_annotated:=false \
    -p cpu_threads:="$VISION_CPU_THREADS"

start_node "P3020 OUT laser vision :8093" \
    "$PYTHON_BIN" isaac_sim/robots/p3020/vision/box_detector_node.py \
    --ros-args \
    --remap __node:=p3020_out_box_detector \
    -p image_topic:=/arm_b/rgb \
    -p box_pixel_topic:=/arm_b/box_pixel \
    -p annotated_image_topic:=/p3020/out/vision/image_annotated \
    -p stream_port:=8093 \
    -p detection_fps:="$P3020_DETECTION_FPS" \
    -p stream_fps:="$P3020_STREAM_FPS" \
    -p stream_max_width:=640 \
    -p jpeg_quality:="$VISION_JPEG_QUALITY" \
    -p publish_annotated:=false \
    -p cpu_threads:="$VISION_CPU_THREADS"

echo "[VISION] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[VISION] AMR       http://<this-pc-ip>:8090/stream.mjpg"
echo "[VISION] P3020 IN  http://<this-pc-ip>:8091/stream.mjpg"
echo "[VISION] TOP VIEW  http://<this-pc-ip>:8092/stream.mjpg"
echo "[VISION] P3020 OUT http://<this-pc-ip>:8093/stream.mjpg"
echo "[VISION] perf: AMR=${AMR_STREAM_FPS}fps TOP=${TOP_VIEW_STREAM_FPS}fps P3020 detect=${P3020_DETECTION_FPS}fps JPEG=${VISION_JPEG_QUALITY}"
echo "[VISION] ONNX threads/process=$VISION_CPU_THREADS (CUDA is auto-used when ONNX Runtime provides it)"
echo "[VISION] Ctrl+C stops all four stream processes."

wait -n "${PIDS[@]}"
status=$?
echo "[VISION] one stream process exited (status=$status); stopping the group."
exit "$status"
