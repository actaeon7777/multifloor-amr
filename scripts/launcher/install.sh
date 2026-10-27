#!/bin/bash
# One-time setup: link the user services and put the start/stop icons in the app list and on the desktop.
set -eu
dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
chmod +x "$dir/start.sh" "$dir/stop.sh"
systemctl --user link "$dir/amr-stack.service" "$dir/amr-web.service" "$dir/amr-agent.service"
systemctl --user daemon-reload
apps="$HOME/.local/share/applications"
mkdir -p "$apps"
for name in amr-start amr-stop; do
    sed "s|@DIR@|$dir|g" "$dir/$name.desktop.in" >"$apps/$name.desktop"
    if [[ -d $HOME/Desktop ]]; then
        install -m 755 "$apps/$name.desktop" "$HOME/Desktop/$name.desktop"
        gio set "$HOME/Desktop/$name.desktop" metadata::trusted true 2>/dev/null || true
    fi
done
echo '설치 완료: 앱 목록과 바탕화면에 "AMR 시작"·"AMR 종료"가 생겼습니다.'
