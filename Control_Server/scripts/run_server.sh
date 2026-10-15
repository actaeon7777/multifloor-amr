#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
packages="$project_root/../src/amr_system"
export PYTHONPATH="$project_root:$packages/amr_remote_bridge:$packages/amr_web_ui:$packages/amr_control${PYTHONPATH:+:$PYTHONPATH}"
if [[ -f config/server.env ]]; then
  set -a
  source config/server.env
  set +a
fi
if [[ ! -x .venv/bin/python ]]; then
  printf '%s\n' 'Create .venv and install requirements first; see README.md.' >&2
  exit 1
fi
case "${1:---tailscale}" in
  --local) bind_address=127.0.0.1 ;;
  --tailscale)
    if ! command -v tailscale >/dev/null; then
      printf '%s\n' 'Tailscale is not installed; see README.md.' >&2
      exit 1
    fi
    bind_address="$(tailscale ip -4)"
    if [[ -z "$bind_address" || "$bind_address" == *$'\n'* ]]; then
      printf '%s\n' 'A single logged-in host Tailscale IPv4 address is required.' >&2
      exit 1
    fi
    ;;
  *) printf '%s\n' 'Usage: scripts/run_server.sh [--local|--tailscale]' >&2; exit 2 ;;
esac
if (( $# > 1 )); then
  printf '%s\n' 'Usage: scripts/run_server.sh [--local|--tailscale]' >&2
  exit 2
fi
exec .venv/bin/python -m app --host "$bind_address" --port "${AMR_PORT:-8081}"
