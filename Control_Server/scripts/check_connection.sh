#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v tailscale >/dev/null; then
  printf '%s\n' 'Install Tailscale and log in first; see README.md.' >&2
  exit 1
fi
tailscale status
host_ip="$(tailscale ip -4)"
if [[ -z "$host_ip" || "$host_ip" == *$'\n'* ]]; then
  printf '%s\n' 'A single logged-in host Tailscale IPv4 address is required.' >&2
  exit 1
fi
if [[ -n "${1:-}" ]]; then
  tailscale ping --timeout=5s --c=3 "$1"
fi
"$project_root/.venv/bin/python" - "$host_ip" "${AMR_PORT:-8081}" <<'PY'
import http.client
import sys
connection = http.client.HTTPConnection(sys.argv[1], int(sys.argv[2]), timeout=3)
try:
    connection.request('GET', '/fleet')
    response = connection.getresponse()
    print(f'Host server HTTP status: {response.status} (401 is expected without credentials)')
    if response.status != 401 or not response.getheader('WWW-Authenticate', '').startswith('Basic '):
        raise SystemExit('Unexpected server response; verify the process listening on this port.')
finally:
    connection.close()
PY

