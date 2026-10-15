#!/usr/bin/env python3
"""Force-stop this Linux user's local ROS 2 processes, across all domains."""
import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import re
import signal
import sys


@dataclass
class Process:
    pid: int
    ppid: int
    start_time: str
    argv: list[str]
    reason: str = ''


def read_process(path):
    if path.stat().st_uid != os.getuid():
        return None
    fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
    if fields[0] == 'Z':
        return None
    argv = [a.decode(errors='replace') for a in (path / 'cmdline').read_bytes().split(b'\0') if a]
    return Process(int(path.name), int(fields[1]), fields[19], argv) if argv else None


def ros_reason(process, maps):
    argv = process.argv
    executable = Path(argv[0]).name
    entrypoint = Path(argv[1]).name if executable.startswith('python') and len(argv) > 1 else executable
    if entrypoint in {'ros2', '_ros2_daemon'}:
        return 'ROS 2 CLI/launch/daemon'
    if entrypoint == 'run_ros_sim.py':
        return 'Isaac Sim ROS bridge'
    if '--ros-args' in argv:
        return 'ROS 2 node'
    if re.search(r'/librcl(?:cpp)?\.so(?:[.\s]|$)', maps):
        return 'ROS 2 client library'
    return ''


def select_targets(processes, protected):
    selected = {pid: p for pid, p in processes.items() if p.reason and pid not in protected}
    # Include launch children such as Isaac's python.sh wrapper and components.
    while True:
        children = {pid: p for pid, p in processes.items()
                    if pid not in selected and pid not in protected and p.ppid in selected}
        if not children:
            return selected
        for p in children.values():
            p.reason = 'ROS process child'
        selected.update(children)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='종료 대상만 표시')
    args = parser.parse_args()
    processes = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            process = read_process(path)
            if process:
                try:
                    maps = (path / 'maps').read_text()
                except OSError:
                    maps = ''
                process.reason = ros_reason(process, maps)
                processes[process.pid] = process
        except OSError:
            continue
    # Never target this command, its calling terminal, or an ancestor launcher.
    protected = {os.getpid()}
    ancestor = os.getpid()
    while ancestor in processes and processes[ancestor].ppid not in protected:
        ancestor = processes[ancestor].ppid
        protected.add(ancestor)
    targets = select_targets(processes, protected)
    if not targets:
        print('현재 계정에서 실행 중인 ROS 2 프로세스가 없습니다.')
        return 0
    for process in sorted(targets.values(), key=lambda p: p.pid):
        print(f'{process.pid:>7}  {process.reason}: {" ".join(process.argv)}', flush=True)
    if args.dry_run:
        print(f'총 {len(targets)}개 종료 대상입니다. 실제 종료하지 않았습니다.')
        return 0
    failures = []
    # Kill parents before their children so ROS launch cannot respawn a node.
    def depth(process):
        seen = set()
        while process.ppid in targets and process.ppid not in seen:
            seen.add(process.ppid)
            process = targets[process.ppid]
        return len(seen)
    stopped = 0
    for process in sorted(targets.values(), key=depth):
        try:
            fd = os.pidfd_open(process.pid)
            try:
                current = read_process(Path('/proc') / str(process.pid))
                if current and current.start_time == process.start_time:
                    signal.pidfd_send_signal(fd, signal.SIGKILL)
                    stopped += 1
            finally:
                os.close(fd)
        except ProcessLookupError:
            pass
        except FileNotFoundError:
            pass
        except OSError as error:
            failures.append(process.pid)
            print(f'PID {process.pid} 종료 실패: {error}', file=sys.stderr)
    print(f'ROS 2 프로세스 {stopped}개에 강제 종료(SIGKILL)를 전달했습니다.')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
