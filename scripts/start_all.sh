#!/usr/bin/env bash
# Start the complete demo stack in the background with per-process log files.
# Nothing is written to ~/.bashrc. Runtime pid/log files live under .runtime/.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="$ROOT_DIR/.runtime"
PID_DIR="$RUNTIME_DIR/pids"
LOG_DIR="$RUNTIME_DIR/logs"
FRESH_DB=0
AUTO_MISSION=0

usage() {
    cat <<'USAGE'
Usage: ./scripts/start_all.sh [--fresh-db] [--mission]

Options:
  --fresh-db  Stop Docker and delete PostgreSQL/MQTT volumes before startup.
  --mission   Start the real AMR + P3020 mission automatically after readiness.

Examples:
  ./scripts/start_all.sh
  ./scripts/start_all.sh --fresh-db
  ./scripts/start_all.sh --fresh-db --mission
USAGE
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --fresh-db) FRESH_DB=1 ;;
        --mission) AUTO_MISSION=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] unknown option: $1"; usage; exit 2 ;;
    esac
    shift
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONUNBUFFERED=1
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/IsaacSim-ros_workspaces/jazzy_ws}"

mkdir -p "$PID_DIR" "$LOG_DIR"
cd "$ROOT_DIR"

compose() {
    if docker info >/dev/null 2>&1; then
        docker compose "$@"
    else
        sudo docker compose "$@"
    fi
}

is_running() {
    local name="$1"
    local pid_file="$PID_DIR/$name.pid"
    [ -f "$pid_file" ] || return 1
    local pid
    pid="$(cat "$pid_file" 2>/dev/null || true)"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

start_process() {
    local name="$1"
    shift
    local pid_file="$PID_DIR/$name.pid"
    local log_file="$LOG_DIR/$name.log"

    if is_running "$name"; then
        echo "[SKIP] $name already running (pid=$(cat "$pid_file"))"
        return 0
    fi

    rm -f "$pid_file"
    : > "$log_file"
    echo "[START] $name -> $log_file"

    nohup setsid "$@" >>"$log_file" 2>&1 < /dev/null &
    local pid=$!
    echo "$pid" > "$pid_file"
    sleep 0.5

    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[ERROR] $name exited during startup"
        tail -n 40 "$log_file" || true
        rm -f "$pid_file"
        return 1
    fi
}

wait_for_ros_publisher() {
    local topic="$1"
    local timeout_s="${2:-180}"
    local end=$((SECONDS + timeout_s))

    echo "[WAIT] publisher: $topic (timeout ${timeout_s}s)"
    while [ "$SECONDS" -lt "$end" ]; do
        if ros2 topic info "$topic" 2>/dev/null | grep -Eq 'Publisher count: [1-9][0-9]*'; then
            echo "[ OK ] $topic"
            return 0
        fi
        sleep 1
    done
    echo "[ERROR] timed out waiting for $topic"
    return 1
}

wait_for_action() {
    local action_name="$1"
    local timeout_s="${2:-120}"
    local end=$((SECONDS + timeout_s))

    echo "[WAIT] action: $action_name (timeout ${timeout_s}s)"
    while [ "$SECONDS" -lt "$end" ]; do
        if ros2 action list 2>/dev/null | grep -Fxq "$action_name"; then
            echo "[ OK ] $action_name"
            return 0
        fi
        sleep 1
    done
    echo "[ERROR] timed out waiting for $action_name"
    return 1
}

wait_for_log() {
    local name="$1"
    local pattern="$2"
    local timeout_s="${3:-180}"
    local log_file="$LOG_DIR/$name.log"
    local end=$((SECONDS + timeout_s))

    echo "[WAIT] $name: $pattern (timeout ${timeout_s}s)"
    while [ "$SECONDS" -lt "$end" ]; do
        if grep -Fq "$pattern" "$log_file" 2>/dev/null; then
            echo "[ OK ] $name ready"
            return 0
        fi
        if ! is_running "$name"; then
            echo "[ERROR] $name stopped before readiness"
            tail -n 60 "$log_file" || true
            return 1
        fi
        sleep 1
    done

    echo "[ERROR] timed out waiting for $name readiness"
    tail -n 60 "$log_file" || true
    return 1
}

require_file() {
    local path="$1"
    local help="$2"
    if [ ! -e "$path" ]; then
        echo "[ERROR] missing: $path"
        echo "        $help"
        exit 1
    fi
}

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "[ERROR] ROS 2 Jazzy not found"
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

require_file "$ROOT_DIR/.env" "Run ./scripts/quick_setup.sh first."
require_file "$ROOT_DIR/.venv/bin/python3" "Run ./scripts/quick_setup.sh first."
require_file "$ROOT_DIR/ros2_ws/install/setup.bash" "Run ./scripts/quick_setup.sh first."
require_file "$ROOT_DIR/frontend/node_modules" "Run ./scripts/quick_setup.sh first."
require_file "$ISAAC_ROS_WS/install/setup.bash" "Set ISAAC_ROS_WS to the Isaac ROS Jazzy workspace."

if [ "$FRESH_DB" = "1" ]; then
    echo "[DOCKER] resetting PostgreSQL/MQTT volumes"
    compose down -v --remove-orphans
fi

echo "[DOCKER] starting PostgreSQL + MQTT + Backend"
compose up -d --build
compose ps

start_process isaac \
    bash -c "cd '$ROOT_DIR' && exec ./scripts/run_isaac_mission.sh"

if ! wait_for_ros_publisher /clock 240; then
    echo "See: $LOG_DIR/isaac.log"
    exit 1
fi
wait_for_ros_publisher /front_2d_lidar/scan 120
wait_for_ros_publisher /back_2d_lidar/scan 120
wait_for_ros_publisher /chassis/odom 120

start_process nav2 \
    bash -c "cd '$ROOT_DIR' && source /opt/ros/jazzy/setup.bash && source '$ISAAC_ROS_WS/install/setup.bash' && exec ./scripts/run_ros2.sh"

start_process adapters \
    bash -c "cd '$ROOT_DIR' && exec ./scripts/run_control_tower_adapters.sh"

start_process p3020_action \
    bash -c "cd '$ROOT_DIR/ros2_ws' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec ros2 run arm_controller pick_place_action_server"

start_process vision \
    bash -c "cd '$ROOT_DIR' && exec ./scripts/run_vision_streams.sh"

start_process frontend \
    bash -c "cd '$ROOT_DIR/frontend' && exec npm run dev -- --host 0.0.0.0"

wait_for_log nav2 "[ROS2] Nav2 is ready." 240

start_process pose_sync \
    bash -c "cd '$ROOT_DIR' && exec ./scripts/run_pose_sync.sh"

wait_for_action /p3020/pick_place 120
wait_for_action /navigate_to_pose 120

# Give pose sync a short window to publish /initialpose before an optional
# automatic mission starts. The pose manager itself keeps running afterwards.
sleep 5

echo
echo "============================================"
echo " FULL STACK STARTED"
echo "============================================"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Frontend: http://localhost:5173"
echo "Backend:  http://localhost:8000"
echo "AMR MJPEG:        http://localhost:8090/stream.mjpg"
echo "P3020 IN MJPEG:   http://localhost:8091/stream.mjpg"
echo "TOP VIEW MJPEG:   http://localhost:8092/stream.mjpg"
echo "P3020 OUT MJPEG:  http://localhost:8093/stream.mjpg"
echo "Logs: $LOG_DIR"
echo "Status: ./scripts/status_all.sh"
echo "Stop:   ./scripts/stop_all.sh"

if [ "$AUTO_MISSION" = "1" ]; then
    start_process mission \
        bash -c "cd '$ROOT_DIR' && exec ./scripts/start_mission.sh"
    echo "Mission started. Log: $LOG_DIR/mission.log"
else
    echo "Mission is NOT started yet. Trigger it with:"
    echo "  ./scripts/start_mission.sh"
fi
