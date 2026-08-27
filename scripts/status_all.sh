#!/usr/bin/env bash
# Show process-manager and Docker status for the demo stack.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_DIR="$ROOT_DIR/.runtime/pids"
LOG_DIR="$ROOT_DIR/.runtime/logs"

compose() {
    if docker info >/dev/null 2>&1; then
        docker compose "$@"
    else
        sudo docker compose "$@"
    fi
}

process_status() {
    local name="$1"
    local pid_file="$PID_DIR/$name.pid"
    if [ ! -f "$pid_file" ]; then
        printf '%-14s %s\n' "$name" "STOPPED"
        return
    fi

    local pid
    pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        printf '%-14s RUNNING pid=%s\n' "$name" "$pid"
    else
        printf '%-14s %s\n' "$name" "STOPPED (stale pid file)"
    fi
}

cd "$ROOT_DIR"

echo "============================================"
echo " COBOT3 STACK STATUS"
echo "============================================"
for name in isaac nav2 pose_sync adapters p3020_action vision frontend mission; do
    process_status "$name"
done

echo
echo "Docker:"
compose ps || true

echo
echo "Logs: $LOG_DIR"
echo "Examples:"
echo "  tail -f $LOG_DIR/isaac.log"
echo "  tail -f $LOG_DIR/vision.log"
echo "  tail -f $LOG_DIR/mission.log"
