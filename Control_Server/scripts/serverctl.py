#!/usr/bin/env python3
"""Start/stop only this project's background foreground-entry-point process."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / '.run'
STATE = RUN / 'server.json'
LOG = ROOT / 'diagnostics/server.log'


def matching_process(pid):
    try:
        process = Path('/proc') / str(pid)
        args = process.joinpath('cmdline').read_bytes().split(b'\0')
        cwd = process.joinpath('cwd').resolve()
        executable = Path(os.fsdecode(args[0]))
        if not executable.is_absolute():
            executable = (cwd / executable if '/' in str(executable) else
                          Path(shutil.which(str(executable)) or str(cwd / executable)))
        allowed = {ROOT / '.venv/bin/python', Path(shutil.which('python3') or '/usr/bin/python3')}
        return (cwd == ROOT and executable.resolve() in {path.resolve() for path in allowed} and
                args[1:3] == [b'-m', b'app'])
    except (OSError, ValueError, IndexError):
        return False


def current():
    try:
        state = json.loads(STATE.read_text())
        return state if matching_process(state['pid']) else None
    except (OSError, ValueError, KeyError):
        return None


def stop():
    state = current()
    if state is None:
        STATE.unlink(missing_ok=True)
        print('This project server is stopped.')
        return
    pid = state['pid']
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline and matching_process(pid):
        time.sleep(.1)
    if matching_process(pid):
        raise RuntimeError('Server is still shutting down; inspect diagnostics/server.log')
    STATE.unlink(missing_ok=True)
    print(f'Stopped project server PID {pid}.')


def start(local=False):
    state = current()
    if state is not None:
        print(f'Already running: {state["url"]} (PID {state["pid"]})')
        return
    LOG.parent.mkdir(exist_ok=True)
    with LOG.open('wb') as output:
        process = subprocess.Popen([str(ROOT / 'scripts/run_server.sh'), '--local' if local else '--tailscale'],
                                   cwd=ROOT, stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    deadline = time.monotonic() + 8
    try:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Server startup failed; inspect diagnostics/server.log')
            text = LOG.read_text(errors='replace')
            if 'AMR central host: ' in text and matching_process(process.pid):
                args = (Path('/proc') / str(process.pid) / 'cmdline').read_bytes().split(b'\0')
                host = os.fsdecode(args[args.index(b'--host') + 1])
                port = int(args[args.index(b'--port') + 1])
                state = {'pid': process.pid, 'url': f'http://{host}:{port}/', 'mode': 'local' if local else 'tailscale'}
                STATE.write_text(json.dumps(state, indent=2) + '\n')
                print(f'Started: {state["url"]} (PID {state["pid"]})')
                print(f'Log: {LOG}')
                return
            time.sleep(.1)
        raise RuntimeError('Server startup timed out; inspect diagnostics/server.log')
    except Exception:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=8)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('start', 'stop', 'restart', 'status'))
    parser.add_argument('--local', action='store_true', help='Bind loopback when starting')
    args = parser.parse_args()
    RUN.mkdir(mode=0o700, exist_ok=True)
    with (RUN / 'control.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            if args.command in ('stop', 'restart'):
                stop()
            if args.command in ('start', 'restart'):
                start(args.local)
            elif args.command == 'status':
                state = current()
                print(f'Running: {state["url"]} (PID {state["pid"]})' if state else 'Stopped')
        except (RuntimeError, OSError) as exc:
            parser.exit(1, f'{exc}\n')


if __name__ == '__main__':
    main()

