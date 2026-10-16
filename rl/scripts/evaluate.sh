#!/usr/bin/env bash
# Deterministic evaluation of checkpoints, one episode per env (1024 by default).
#   rl/scripts/evaluate.sh logs/rsl_rl/amr_nav_arena/<run>/model_300.pt model_1999.pt [--num_envs 1024]
set -euo pipefail
source "$(dirname "$0")/lab_env.sh"
cd "$WS"
lab "$LAB_VENV/bin/python" rl/scripts/evaluate.py --viz none --device cuda:0 "$@"
