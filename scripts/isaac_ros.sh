#!/usr/bin/env bash
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ ${ROS_DOMAIN_ID:-107} == 107 ]] || { echo 'Isaac bridge requires domain 107' >&2; exit 1; }
export ROS_DOMAIN_ID=107
# Isaac supplies its own Python and Jazzy libraries. Do not leak Conda/native overlays.
unset ROS_DISTRO PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec "$workspace_dir/python.sh" "$workspace_dir/sim/scripts/run_ros_sim.py" "$@"
