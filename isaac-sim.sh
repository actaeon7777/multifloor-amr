#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
if [[ ! -x "$ISAACSIM_PATH/isaac-sim.sh" ]]; then
    echo "Isaac Sim installation is not ready: $ISAACSIM_PATH" >&2
    exit 1
fi
exec "$ISAACSIM_PATH/isaac-sim.sh" \
    --/app/window/width=1280 --/app/window/height=720 \
    --/renderer/multiGpu/enabled=false \
    --/exts/isaacsim.core.simulation_manager/default_engine=physx "$@"

