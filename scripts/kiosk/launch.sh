#!/bin/bash
# Run from the logged-in GNOME desktop. Restore the user's settings on exit.
set -u
kiosk_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
kiosk_url="http://127.0.0.1:${AMR_WEB_PORT:-8080}/kiosk"
kiosk_browser_pid=''
kiosk_idle=''
kiosk_lock=''
kiosk_pid_file=''
kiosk_settings_saved=0

# Main process of a kiosk Chromium, ours or one whose launcher is gone.
kiosk_browser_pattern='^/snap/chromium/[^ ]*/chrome .*--class=amr-kiosk( |$)'

cleanup() {
    # A second TERM (button pressed twice) must not cut cleanup short and orphan Chromium.
    trap '' INT TERM HUP
    trap - EXIT
    if [[ -n "$kiosk_browser_pid" ]]; then
        kill "$kiosk_browser_pid" 2>/dev/null || true
        for ((kiosk_i=0; kiosk_i<50; kiosk_i++)); do
            kill -0 "$kiosk_browser_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL "$kiosk_browser_pid" 2>/dev/null || true
        wait "$kiosk_browser_pid" 2>/dev/null || true
    fi
    if [[ "$kiosk_settings_saved" == 1 ]]; then
        gsettings set org.gnome.desktop.session idle-delay "$kiosk_idle" || true
        gsettings set org.gnome.desktop.screensaver lock-enabled "$kiosk_lock" || true
    fi
    [[ -z "$kiosk_pid_file" ]] || rm -f "$kiosk_pid_file"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

# Prevent a second invocation from changing/restoring the first session's settings.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/amr-kiosk-${UID}.lock"
flock -n 9 || exit 0
# The web UI's "키오스크 닫기" (admin mode) sends SIGTERM to this PID.
echo $$ >"${XDG_RUNTIME_DIR:-/tmp}/amr-kiosk-${UID}.pid" &&
    kiosk_pid_file="${XDG_RUNTIME_DIR:-/tmp}/amr-kiosk-${UID}.pid"
# With the lock held, a running kiosk Chromium has lost its launcher. Starting
# another would only hand the URL to it ("Opening in existing browser session"),
# leaving it without the fullscreen watch and "키오스크 닫기"; replace it.
if pkill -f -- "$kiosk_browser_pattern"; then
    for ((kiosk_i=0; kiosk_i<50; kiosk_i++)); do
        pgrep -f -- "$kiosk_browser_pattern" >/dev/null || break
        sleep 0.1
    done
    pkill -KILL -f -- "$kiosk_browser_pattern" || true
fi
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
kiosk_start=$SECONDS

# Chromium's own --kiosk fullscreen lasts ~0.1 s before it falls back to a normal
# window (measured 2026-10-28), and a request before mapping is lost. Check the
# window-manager state, not the size, for as long as the browser runs.
kiosk_window=''
kiosk_window_state() {
    [[ -n "$kiosk_window" ]] && xprop -id "$kiosk_window" _NET_WM_STATE 2>/dev/null
}
while kill -0 "$kiosk_browser_pid" 2>/dev/null; do
    if ! kiosk_state=$(kiosk_window_state); then
        # The ("chrome" "amr-kiosk") client leader window is never mapped; skip it.
        kiosk_window=$(xwininfo -root -tree 2>/dev/null | awk '
            /\("chromium [^"]*" "amr-kiosk"\)/ {print $1; exit}')
        kiosk_state=$(kiosk_window_state)
    fi
    if [[ -n "$kiosk_window" && "$kiosk_state" != *_NET_WM_STATE_FULLSCREEN* ]]; then
        python3 -I "$kiosk_dir/fullscreen.py" "$kiosk_window" || true
    fi
    kiosk_delay=2
    (( SECONDS >= kiosk_start + 30 )) || kiosk_delay=0.5
    # Background sleep keeps the TERM trap immediate.
    sleep "$kiosk_delay" & wait $!
done
wait "$kiosk_browser_pid"
kiosk_status=$?
kiosk_browser_pid=''
exit "$kiosk_status"
