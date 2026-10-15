#!/usr/bin/env bash
set -eo pipefail
export AMR_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-sim}
[[ $# -eq 0 ]] || shift
case "$mode" in
  sim) export ROS_DOMAIN_ID=107 ;;
  real) export ROS_DOMAIN_ID=7 ;;
  *) echo 'Usage: experiment.sh sim|real [launch arguments]' >&2; exit 2 ;;
esac
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CONDA_PREFIX
source "$AMR_WS/scripts/env.sh"
if [[ $mode == real ]]; then
  if [[ ${AMR_REQUIRE_TIME_SYNC:-0} == 1 ]]; then
    /usr/bin/python3 "$AMR_WS/scripts/check_time_sync.py"
  fi
  exec ros2 launch amr_stack experiment.launch.py mode:=real \
    map:="$AMR_WS/src/amr_system/amr_stack/maps/G_4_floor/4_floor_0522.yaml" "$@"
fi
exec ros2 launch amr_stack experiment.launch.py mode:=sim "$@"
