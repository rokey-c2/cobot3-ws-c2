#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_isaac_env.sh"

exec "$ISAAC_SIM_DIR/python.sh" test_new_logic.py
