#!/usr/bin/env bash
set -euo pipefail
AMR_SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$AMR_SCRIPT_ROOT/python.sh" "$AMR_SCRIPT_ROOT/sim/scripts/run_amr.py" "$@"

