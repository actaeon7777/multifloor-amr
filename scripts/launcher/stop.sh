#!/bin/bash
# "AMR 종료" icon (or over ssh: stop.sh --yes). Stops only what start.sh started.
set -u
ws=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
question='주행 중이면 로봇이 바로 멈춥니다. AMR을 종료할까요?'
if [[ ${1:-} != --yes ]]; then
    if [[ -t 0 ]]; then
        read -r -p "$question [y/N] " answer
        [[ $answer == [yY]* ]] || exit 0
    else
        zenity --question --title='AMR 종료' --width=420 --text="$question" 2>/dev/null || exit 0
    fi
fi
# SIGTERM lets launch.sh close Chromium and restore the screen-lock settings.
# The icon's kiosk runs as amr-kiosk (start.sh); a terminal one is matched by path.
systemctl --user stop amr-kiosk.service 2>/dev/null || true
pkill -f "$ws/scripts/kiosk/launch.sh" || true
# A kiosk Chromium whose launcher already died.
pkill -f -- '^/snap/chromium/[^ ]*/chrome .*--class=amr-kiosk( |$)' || true
systemctl --user stop amr-agent.service amr-web.service amr-stack.service
