#!/usr/bin/env bash
# Save experiment identity, active parameters and selected ROS topics.
set -eo pipefail

usage() {
  cat <<'EOF'
사용법: AMR_EXPERIMENT_MODE=sim|real ./scripts/record_trial.sh [실험이름] (기본 sim)

주행 스택을 먼저 실행하고 별도 터미널에서 사용하세요.
한국 시간 기준 bags/YYYY-MM-DD/HHMMSS[_실험이름]/ 에 저장합니다.
실험이름에는 영문, 숫자, 밑줄, 하이픈을 사용할 수 있습니다.
기록을 종료하려면 Ctrl+C를 누르세요.
costmap(전체·증분), ROS 로그, 목표 피드백도 함께 기록합니다.
AMR_RECORD_DWB_EVALUATION=1이면 /evaluation을 추가합니다.
(DWB publish_evaluation은 별도로 활성화해야 합니다.)
EOF
}

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  usage
  exit 0
fi
if (( $# > 1 )) || [[ -n ${1:-} && ! ${1:-} =~ ^[a-zA-Z0-9_-]+$ ]]; then
  usage >&2
  exit 2
fi
label=${1:-}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_dir=$(cd -- "$script_dir/.." && pwd)

# Use the same Jazzy overlay and default domain as the amr shell command.
# ROS setup scripts reference unset variables, so enable nounset afterward.
mode=${AMR_EXPERIMENT_MODE:-sim}
case "$mode" in
  sim) export ROS_DOMAIN_ID=107 ;;
  real) export ROS_DOMAIN_ID=7 ;;
  *) echo 'AMR_EXPERIMENT_MODE must be sim or real' >&2; exit 2 ;;
esac
source "$workspace_dir/scripts/env.sh"
set -u
preparation_started=$SECONDS
if ! command -v ros2 >/dev/null || ! ros2 bag record --help >/dev/null; then
  printf 'ros2 bag record를 사용할 수 없습니다. rosbag2 설치를 확인하세요.\n' >&2
  exit 1
fi
config_dir="$workspace_dir/src/amr_system/amr_stack/config"
if [[ ! -d $config_dir ]]; then
  printf '설정 폴더가 없습니다: %s\n' "$config_dir" >&2
  exit 1
fi

# Derive the date and time from one instant, including across midnight.
stamp=$(TZ=Asia/Seoul date +%Y-%m-%d/%H%M%S)
trial_base="$workspace_dir/bags/$stamp"
if [[ -n $label ]]; then
  trial_base+="_$label"
fi
mkdir -p -- "$(dirname -- "$trial_base")"
trial_dir=$trial_base
sequence=1
while ! mkdir -- "$trial_dir" 2>/dev/null; do
  if [[ ! -d $trial_dir ]]; then
    printf '실험 폴더를 만들 수 없습니다: %s\n' "$trial_dir" >&2
    exit 1
  fi
  sequence=$((sequence + 1))
  trial_dir="${trial_base}_$sequence"
done

printf '저장 폴더: %s\n설정과 파라미터를 저장합니다…\n' "$trial_dir"
cp -R -- "$config_dir" "$trial_dir/config_snapshot"
mkdir -- "$trial_dir/params"
/usr/bin/python3 "$workspace_dir/scripts/capture_manifest.py" --mode "$mode" \
  --id "$(basename -- "$trial_dir")" --output "$trial_dir/manifest.json"
if [[ -e "$workspace_dir/.git" ]]; then
  git -C "$workspace_dir" diff --binary HEAD > "$trial_dir/source_changes.patch"
else
  : > "$trial_dir/source_changes.patch"
  printf '%s\n' 'Source archive: no Git history. See RELEASE.json and manifest.json for source identity.' \
    > "$trial_dir/source_archive.txt"
  if [[ -f "$workspace_dir/RELEASE.json" ]]; then
    cp -- "$workspace_dir/RELEASE.json" "$trial_dir/RELEASE.json"
  fi
fi
cp -R -- "$workspace_dir/sim/config" "$trial_dir/simulation_model"
{
  printf 'prepared_at=%s\n' "$(TZ=Asia/Seoul date --iso-8601=seconds)"
  printf 'timezone=Asia/Seoul\nworkspace=%s\n' "$workspace_dir"
  printf 'ROS_DISTRO=%s\nROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\n' \
    "${ROS_DISTRO:-jazzy}" "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-default}"
} > "$trial_dir/session.txt"

# One DDS participant reads all nodes concurrently, including nested costmaps.
# A shared wall-clock deadline bounds missing/unresponsive nodes together.
# The real CAN command node is intentionally absent from simulation.
/usr/bin/python3 "$script_dir/capture_parameters.py" --mode "$mode" \
  --output "$trial_dir/params"
preparation_elapsed=$((SECONDS - preparation_started))
printf '준비 완료: %s초\n' "$preparation_elapsed"
printf 'preparation_duration_s=%s\n' "$preparation_elapsed" >> "$trial_dir/session.txt"

printf '\n제어·센서·추정 토픽 기록을 시작합니다. 종료: Ctrl+C\n'
printf 'recording_started_at=%s\n' "$(TZ=Asia/Seoul date --iso-8601=seconds)" >> "$trial_dir/session.txt"
# The foreground recorder receives terminal Ctrl+C and finalizes metadata.
# Keep the shell alive afterward to report the saved folder and bag summary.
trap ':' INT
topics=(/clock /cmd_vel /cmd_vel_navigation /cmd_vel_auto /wheel/odometry /joint_states
        /imu/data_raw /scan /odometry/filtered /amcl_pose /tf /tf_static /plan
        /map /local_costmap/costmap /local_costmap/costmap_updates
        /global_costmap/costmap /global_costmap/costmap_updates
        /local_costmap/published_footprint /local_plan /transformed_global_plan
        /rosout /diagnostics /parameter_events /goal_pose /initialpose
        /control_mux/mode /control_mux/status /navigation/status /localization/status /sim/status
        /navigate_to_pose/_action/status /navigation/navigate_to_pose/_action/status
        /navigate_to_pose/_action/feedback /navigation/navigate_to_pose/_action/feedback)
# Current bridge reports PhysX contact counts in /sim/status, not /sim/contacts.
[[ $mode != sim ]] || topics+=(/sim/ground_truth)
# Candidate trajectory clouds (/evaluation) are deliberately opt-in.
[[ ${AMR_RECORD_DWB_EVALUATION:-0} != 1 ]] || topics+=(/evaluation)
printf '%s\n' "${topics[@]}" > "$trial_dir/recorded_topics.txt"
if ros2 bag record -s sqlite3 --include-hidden-topics -o "$trial_dir/bag" --topics "${topics[@]}"; then
  record_status=0
else
  record_status=$?
fi
trap - INT
{
  printf 'recording_finished_at=%s\n' "$(TZ=Asia/Seoul date --iso-8601=seconds)"
  printf 'recorder_exit_code=%s\n' "$record_status"
} >> "$trial_dir/session.txt"

if [[ -f $trial_dir/bag/metadata.yaml ]]; then
  if timeout --foreground 15s ros2 bag info "$trial_dir/bag" \
      > "$trial_dir/bag_info.txt" 2> "$trial_dir/bag_info_errors.txt"; then
    cat "$trial_dir/bag_info.txt"
  else
    printf 'bag 정보 확인에 실패했습니다. bag_info_errors.txt를 확인하세요.\n' >&2
  fi
  printf '\n기록 종료. 실험 폴더: %s\n' "$trial_dir"
else
  printf '\n완료된 metadata.yaml이 없습니다. 기록 출력을 확인하세요.\n실험 폴더: %s\n' "$trial_dir" >&2
  exit 1
fi
if (( record_status != 0 && record_status != 130 )); then
  printf '기록기가 오류로 종료되었습니다 (exit %s).\n' "$record_status" >&2
  exit "$record_status"
fi
