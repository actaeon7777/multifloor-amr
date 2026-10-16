#!/usr/bin/env bash
# Collect and classify collisions of one checkpoint (deterministic policy).
#   rl/scripts/analyze_collisions.sh logs/rsl_rl/amr_nav_arena/<run>/model_1000.pt [--rounds 3]
set -euo pipefail
source "$(dirname "$0")/lab_env.sh"
cd "$WS"
lab "$LAB_VENV/bin/python" rl/scripts/analyze_collisions.py --viz none --device cuda:0 "$@"
