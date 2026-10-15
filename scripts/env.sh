#!/usr/bin/env bash
# Source this file to load this relocatable Jazzy workspace.
amr_workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
alias ros2kill="/usr/bin/python3 $(printf '%q' "$amr_workspace_dir/scripts/ros2kill.py")"
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$amr_workspace_dir/install/local_setup.bash" ]]; then
    printf 'Jazzy 또는 AMR 빌드 환경이 없습니다. README의 빌드 절차를 확인하세요.\n' >&2
    return 1
fi
source /opt/ros/jazzy/setup.bash
source "$amr_workspace_dir/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset amr_workspace_dir
