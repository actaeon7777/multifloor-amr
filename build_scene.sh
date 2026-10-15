#!/usr/bin/env bash
set -euo pipefail
AMR_SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Every floor in src/amr_system/amr_stack/config/floors.yaml (4F output is unchanged).
python3 "$AMR_SCRIPT_ROOT/sim/tools/build_floor_layers.py"
python3 "$AMR_SCRIPT_ROOT/sim/scripts/prepare_physics_urdf.py"
# Stacked 3F/4F/5F building with the elevator (text USD; references the layers above).
python3 "$AMR_SCRIPT_ROOT/sim/tools/build_building_scene.py"
exec "$AMR_SCRIPT_ROOT/python.sh" "$AMR_SCRIPT_ROOT/sim/scripts/build_scene.py" "$@"
