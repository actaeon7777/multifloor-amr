# Shared by train.sh / watch.sh: run Isaac Lab without the ROS paths the shell profile adds.
export OMNI_KIT_ACCEPT_EULA=YES
LAB_VENV="${LAB_VENV:-$HOME/IsaacLab/.venv}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
lab() { env -u PYTHONPATH -u AMENT_PREFIX_PATH -u LD_LIBRARY_PATH "$@"; }
