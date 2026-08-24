#!/usr/bin/env bash
# vision_node(LocateBox/ConfirmGrasp)가 필요로 하는 onnxruntime을
# 시스템 python을 건드리지 않고 venv에만 설치한다.
# --system-site-packages 로 만들어서 rclpy/cv_bridge/message_filters 등
# apt로 깔린 ROS2 파이썬 패키지는 그대로 보인다.
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

python3 -m venv --system-site-packages "$VENV_DIR"
"$VENV_DIR/bin/python3" -m pip install --upgrade pip
"$VENV_DIR/bin/python3" -m pip install onnxruntime

echo ""
echo "설치 완료. 사용 전에:"
echo "  source $VENV_DIR/bin/activate"
