#!/usr/bin/env bash
# Training curves at http://localhost:6006 (reward, episode length, success/collision rate).
set -euo pipefail
source "$(dirname "$0")/lab_env.sh"
cd "$WS"
lab "$LAB_VENV/bin/tensorboard" --logdir logs/rsl_rl/amr_nav_arena --port 6006 "$@"
