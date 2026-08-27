#!/usr/bin/env bash
# Stop processes started by scripts/start_all.sh. Docker volumes are preserved
# unless --volumes is explicitly requested.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_DIR="$ROOT_DIR/.runtime/pids"
REMOVE_VOLUMES=0

if [ "${1:-}" = "--volumes" ]; then
    REMOVE_VOLUMES=1
elif [ "$#" -gt 0 ]; then
    echo "Usage: ./scripts/stop_all.sh [--volumes]"
    exit 2
fi

compose() {
    if docker info >/dev/null 2>&1; then
        docker compose "$@"
    else
        sudo docker compose "$@"
    fi
}

stop_process() {
    local name="$1"
    local pid_file="$PID_DIR/$name.pid"

    if [ ! -f "$pid_file" ]; then
        echo "[SKIP] $name not tracked"
        return 0
    fi

    local pid
    pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
        echo "[SKIP] $name already stopped"
        rm -f "$pid_file"
        return 0
    fi

    echo "[STOP] $name (pid=$pid)"
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true

    for _ in $(seq 1 20); do
        if ! kill -0 "$pid" 2>/dev/null; then
            rm -f "$pid_file"
            return 0
        fi
        sleep 0.25
    done

    echo "[KILL] $name did not exit gracefully"
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    rm -f "$pid_file"
}

cd "$ROOT_DIR"
mkdir -p "$PID_DIR"

# Reverse dependency order. Isaac is intentionally last.
for name in mission frontend vision p3020_action pose_sync adapters nav2 isaac; do
    stop_process "$name"
done

if [ "$REMOVE_VOLUMES" = "1" ]; then
    echo "[DOCKER] stopping services + deleting volumes"
    compose down -v --remove-orphans
else
    echo "[DOCKER] stopping services (volumes preserved)"
    compose down --remove-orphans
fi

echo "All tracked processes stopped."
