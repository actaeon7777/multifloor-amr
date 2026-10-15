#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
if [[ ! -x "$ISAACSIM_PATH/python.sh" ]]; then
    echo "Isaac Sim bundled Python is not ready: $ISAACSIM_PATH" >&2
    exit 1
fi
exec "$ISAACSIM_PATH/python.sh" "$@"

