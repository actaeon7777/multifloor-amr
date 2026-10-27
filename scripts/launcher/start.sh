#!/bin/bash
# "AMR 시작" icon: vehicle stack and web UI as user services, then the kiosk.
set -u
ws=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
fail() {
    zenity --error --title='AMR 시작' --width=460 --text="$1" 2>/dev/null || echo "$1" >&2
    exit 1
}
# Real-stack CAN nodes started by hand (ssh); isolated tests set ROS_LOCALHOST_ONLY=1.
manual_stack() {
    for pid in $(pgrep -f 'lib/amr_[a-z_]+/(cmd_vel_to_can|control_mux|can_telemetry|elevator_can_link)' || true); do
        tr '\0' '\n' <"/proc/$pid/environ" 2>/dev/null | grep -qx 'ROS_LOCALHOST_ONLY=1' || echo "$pid"
    done
}

if ! systemctl --user --quiet is-active amr-stack.service; then
    # Two stacks would both drive can0.
    [[ -z $(manual_stack) ]] || fail '차량 스택이 이미 터미널(ssh)에서 실행 중입니다.
그 터미널에서 Ctrl+C로 끈 뒤 다시 누르세요.'
    systemctl --user start amr-stack.service || fail '차량 스택을 시작하지 못했습니다.
journalctl --user-unit amr-stack -n 50 으로 확인하세요.'
fi
# A web UI started by hand already serves the kiosk; leave it alone.
if ! systemctl --user --quiet is-active amr-web.service &&
   ! curl --fail --silent --output /dev/null --max-time 2 http://127.0.0.1:8080/kiosk; then
    systemctl --user start amr-web.service || fail '웹 UI를 시작하지 못했습니다.
journalctl --user-unit amr-web -n 50 으로 확인하세요.'
fi
# Optional central console link; a missing agent.env exits successfully.
systemctl --user start amr-agent.service || echo '중앙 관제 연결 시작 실패: journalctl --user-unit amr-agent -n 50' >&2
sleep 3
systemctl --user --quiet is-active amr-stack.service || [[ -n $(manual_stack) ]] ||
    fail '차량 스택이 시작 직후 종료되었습니다.
journalctl --user-unit amr-stack -n 50 으로 확인하세요.'
exec "$ws/scripts/kiosk/launch.sh"
