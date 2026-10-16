#!/usr/bin/env bash
# Start training in the background (survives closing the terminal).
#   rl/scripts/train.sh                         # 1500 iterations, 1024 envs
#   rl/scripts/train.sh agent.max_iterations=500 --num_envs 2048
# Progress: tail -f the printed log. Stop: kill the printed PID.
set -euo pipefail
source "$(dirname "$0")/lab_env.sh"
cd "$WS"
mkdir -p logs/rl
log="logs/rl/train_$(date +%Y%m%d_%H%M%S).log"
nohup env -u PYTHONPATH -u AMENT_PREFIX_PATH -u LD_LIBRARY_PATH \
  "$LAB_VENV/bin/isaaclab" train --rl_library rsl_rl --task AMR-Nav-Arena-v0 --viz none "$@" \
  > "$log" 2>&1 &
echo "training PID $! — log: $log"
echo "  progress : tail -f $log | grep -E 'Learning iteration|success_rate|collision_rate'"
echo "  curves   : rl/scripts/tensorboard.sh"
echo "  watch    : rl/scripts/watch.sh"
echo "  stop     : kill $!"
