#!/usr/bin/env bash
# Source this only when a shell needs the AMR workspace paths.
export AMR_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ISAACSIM_PATH="$AMR_WS/runtime/isaac-sim-6.1.0"
export XDG_CACHE_HOME="$AMR_WS/runtime/cache"
export OMNI_KIT_ACCEPT_EULA=YES
export PYTHONUNBUFFERED=1
