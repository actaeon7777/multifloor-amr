#!/bin/bash
# Run from the logged-in GNOME desktop. Restore the user's settings on exit.
set -u
kiosk_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
kiosk_url="http://127.0.0.1:${AMR_WEB_PORT:-8080}/kiosk"
kiosk_browser_pid=''
kiosk_idle=''
kiosk_lock=''
kiosk_settings_saved=0

cleanup() {
    trap - EXIT INT TERM HUP
    if [[ -n "$kiosk_browser_pid" ]]; then
        kill "$kiosk_browser_pid" 2>/dev/null || true
        wait "$kiosk_browser_pid" 2>/dev/null || true
    fi
    if [[ "$kiosk_settings_saved" == 1 ]]; then
        gsettings set org.gnome.desktop.session idle-delay "$kiosk_idle" || true
        gsettings set org.gnome.desktop.screensaver lock-enabled "$kiosk_lock" || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

# Prevent a second invocation from changing/restoring the first session's settings.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/amr-kiosk-${UID}.lock"
flock -n 9 || exit 0
kiosk_deadline=$((SECONDS + 120))
until curl --fail --silent --output /dev/null --connect-timeout 1 --max-time 2 "$kiosk_url"; do
    if (( SECONDS >= kiosk_deadline )); then
        echo '키오스크 웹 응답 대기 시간 초과' >&2
        exit 1
    fi
    sleep 0.5
done

# gsettings writes are persistent unless restored; retain both original values.
if kiosk_idle=$(gsettings get org.gnome.desktop.session idle-delay) &&
   kiosk_lock=$(gsettings get org.gnome.desktop.screensaver lock-enabled); then
    kiosk_settings_saved=1
    gsettings set org.gnome.desktop.session idle-delay 0 || true
    gsettings set org.gnome.desktop.screensaver lock-enabled false || true
fi
/snap/bin/chromium --user-data-dir="$HOME/snap/chromium/common/amr-kiosk" \
    --class=amr-kiosk --kiosk --force-device-scale-factor=1.5 \
    --no-first-run --noerrdialogs --disable-pinch --overscroll-history-navigation=0 \
    "$kiosk_url" 9>&- &
kiosk_browser_pid=$!

# Requests before mapping can be lost: repeat until mutter actually resizes it.
for ((kiosk_attempt=0; kiosk_attempt<120; kiosk_attempt++)); do
    kill -0 "$kiosk_browser_pid" 2>/dev/null || break
    kiosk_window=$(xwininfo -root -tree 2>/dev/null | awk '
        /\([^)]*amr-kiosk[^)]*\)/ {print; exit}')
    case "$kiosk_window" in
        *" 1920x1280+"*) break ;;
        "") ;;
        *) read -r kiosk_window_id _ <<< "$kiosk_window"
           python3 -I "$kiosk_dir/fullscreen.py" "$kiosk_window_id" || true ;;
    esac
    sleep 0.5
done
wait "$kiosk_browser_pid"
kiosk_status=$?
kiosk_browser_pid=''
exit "$kiosk_status"
