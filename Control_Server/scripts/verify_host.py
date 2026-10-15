#!/usr/bin/env python3
"""Read-only verification of the host's authenticated Tailscale HTTP endpoints."""
import argparse
import base64
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'vendor/amr_remote_bridge'))
from amr_remote_bridge.common import HttpClient, loads, read_token


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', help='Defaults to tailscale ip -4')
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--token-file', type=Path, default=Path.home() / '.config/amr_remote_bridge/token')
    parser.add_argument('--output', type=Path, default=ROOT / 'diagnostics/host_result.json')
    args = parser.parse_args()
    host = args.host or subprocess.check_output(['tailscale', 'ip', '-4'], text=True).strip()
    url = f'http://{host}:{args.port}'
    token = read_token(args.token_file)
    auth = {'Authorization': 'Basic ' + base64.b64encode(('operator:' + token).encode()).decode()}
    http = HttpClient()
    anonymous = http.request(url + '/fleet', timeout=3)
    assert anonymous[0] == 401
    checks = {}
    for path in ('/', '/robots/amr-01/', '/fleet', '/api/robots'):
        status, headers, body = http.request(url + path, headers=auth, timeout=3)
        assert status == 200, (path, status)
        normalized = {key.lower(): value for key, value in headers.items()}
        assert normalized['cache-control'] == 'no-store'
        checks[path] = status
        if path == '/api/robots':
            robot_count = len(loads(body)['robots'])
    state_status, _, _ = http.request(url + '/api/state', headers=auth, timeout=3)
    assert state_status in (200, 503)
    report = {'host_url': url, 'anonymous_status': anonymous[0], 'authenticated_paths': checks,
              'state_status': state_status, 'registered_robots': robot_count,
              'scope': 'host requests to its own Tailscale address; remote AMR not verified'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()

