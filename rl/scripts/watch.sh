#!/usr/bin/env bash
# Open a viewer that follows the newest checkpoint of the newest run. Close it any time;
# training keeps going.   rl/scripts/watch.sh [--num_envs 4] [--show_scan] [--run <folder>]
set -euo pipefail
source "$(dirname "$0")/lab_env.sh"
cd "$WS"
lab "$LAB_VENV/bin/python" rl/scripts/watch.py --device cuda:0 "$@"
