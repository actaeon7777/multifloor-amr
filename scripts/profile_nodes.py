#!/usr/bin/env python3
"""Record per-process CPU/RSS of the running AMR stack at 1 Hz until Ctrl+C.

Usage:
    python3 scripts/profile_nodes.py --label before_opt
    python3 scripts/profile_nodes.py --summarize reports/profile/*.csv
Writes reports/profile/<date>_<label>.csv (one row per process per second) and
prints a per-node summary (mean / p95 / max CPU %) on exit. Only psutil is used,
so the sampler itself costs well under 1 % of a core.

A node name may cover several PIDs (a launcher and its worker, a restarted
node). The summary sums the PIDs sampled at the same instant and then takes
mean/p95/max over time, so a 0 % launcher next to a 20 % worker reads 20 %,
not 10 % (the pre-2026-10-27 version averaged all samples of a name together).
"""
import argparse
import csv
import os
import signal
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import psutil

ROOT = Path(__file__).resolve().parent.parent

# Substrings of the command line that identify the processes we care about.
PATTERNS = [
    'navigation_startup', 'navigation_manager', 'scan_body_filter', 'joy_autoselect',
    'joy_node', 'joy_teleop', 'control_mux', 'time_health', 'can_telemetry',
    'cmd_vel_to_can', 'elevator_can_link', 'vehicle_status', 'ekf_node',
    'web_ui', 'remote_client', 'remote_bridge', 'elevator_monitor', 'floor_mission',
    'mission_node', 'monitor_node', 'controller_server', 'planner_server',
    'bt_navigator', 'behavior_server', 'smoother_server', 'velocity_smoother',
    'lifecycle_manager', 'amcl', 'map_server', 'tim_global_localizer',
    'robot_state_publisher', 'sllidar_node', 'orbbec_camera_node',
    'depth_persistence_filter', 'ros2 launch', 'chromium', 'chrome',
    'update-manager', 'gnome-shell', 'gnome-software', 'snapd',
]
# Measurement tools whose command lines mention node names (py-spy output
# files, this script's own CSV path) must not be counted as the node.
TOOL_MARKERS = ('profile_nodes', 'py-spy', 'pyspy_', 'reports/profile')
COLUMNS = ['time', 'label', 'pid', 'ppid', 'exe', 'cpu_percent', 'rss_mb', 'threads']


def label_of(cmdline):
    text = ' '.join(cmdline)
    if any(marker in text for marker in TOOL_MARKERS):
        return None
    for pattern in PATTERNS:
        if pattern in text:
            if pattern == 'ros2 launch':
                return 'ros2_launch'
            return pattern
    return None


def exe_name(proc, cmdline):
    # The first non-interpreter argument names a Python node better than python3.
    for arg in cmdline[:2]:
        base = os.path.basename(arg)
        if base and not base.startswith('python'):
            return base
    try:
        return proc.name()
    except psutil.Error:
        return ''


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values) - 1, int(len(values) * fraction))]


def summarize(rows, skip_seconds=0.):
    """Per-node (time-aligned PID sum) and per-PID statistics from CSV rows.

    Returns (elapsed, by_label, by_pid): by_label maps label ->
    {'mean', 'p95', 'max', 'pids'}; by_pid maps (label, pid) ->
    {'mean', 'p95', 'max', 'samples', 'exe'}.
    """
    per_instant = defaultdict(lambda: defaultdict(float))  # label -> time -> cpu sum
    per_pid = defaultdict(list)
    exe_of = {}
    last = 0.
    for row in rows:
        stamp = float(row['time'])
        if stamp < skip_seconds:
            continue
        last = max(last, stamp)
        label, pid, cpu = row['label'], row['pid'], float(row['cpu_percent'])
        per_instant[label][stamp] += cpu
        per_pid[label, pid].append(cpu)
        exe_of[label, pid] = row.get('exe', '')
    by_label = {}
    for label, series in per_instant.items():
        values = list(series.values())
        pids = sorted({pid for (name, pid) in per_pid if name == label}, key=int)
        by_label[label] = dict(mean=sum(values) / len(values), p95=percentile(values, .95),
                               max=max(values), pids=pids)
    by_pid = {key: dict(mean=sum(v) / len(v), p95=percentile(v, .95), max=max(v),
                        samples=len(v), exe=exe_of[key]) for key, v in per_pid.items()}
    return last, by_label, by_pid


def print_summary(path, rows, skip_seconds, ncpu=None, show_pids=False):
    elapsed, by_label, by_pid = summarize(rows, skip_seconds)
    cores = f', CPU {ncpu}코어' if ncpu else ''
    print(f'\n{path}\n요약 ({elapsed:.0f} 초 경과, 처음 {skip_seconds:g} 초 제외{cores}, 1코어 = 100 %)')
    print(f'{"node":28s} {"mean":>7s} {"p95":>7s} {"max":>7s}  pids')
    for label, stat in sorted(by_label.items(), key=lambda item: -item[1]['mean']):
        print(f'{label:28s} {stat["mean"]:7.1f} {stat["p95"]:7.1f} {stat["max"]:7.1f}  '
              f'{",".join(stat["pids"]) if len(stat["pids"]) > 1 else ""}')
    multi = [label for label, stat in by_label.items() if len(stat['pids']) > 1]
    if show_pids or multi:
        print(f'\nPID별 (같은 이름이 여러 PID인 노드{"" if show_pids else "만"})')
        print(f'{"node":28s} {"pid":>7s} {"mean":>7s} {"p95":>7s} {"max":>7s} {"n":>5s}  exe')
        for (label, pid), stat in sorted(by_pid.items(), key=lambda item: (item[0][0], int(item[0][1]))):
            if show_pids or label in multi:
                print(f'{label:28s} {pid:>7s} {stat["mean"]:7.1f} {stat["p95"]:7.1f} '
                      f'{stat["max"]:7.1f} {stat["samples"]:5d}  {stat["exe"]}')


def record(args):
    out_dir = ROOT / 'reports' / 'profile'
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = out_dir / f'{stamp}_{args.label}.csv'

    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    tracked = {}  # pid -> (label, psutil.Process, exe, ppid, first sample pending)
    rows = []
    ncpu = psutil.cpu_count()
    psutil.cpu_percent(None)

    with out_path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(COLUMNS)
        print(f'기록 중: {out_path}\n주행이 끝나면 Ctrl+C 로 멈추세요.', flush=True)
        start = time.monotonic()
        while not stop:
            # Refresh the process table so nodes started later are picked up.
            for proc in psutil.process_iter(['pid', 'ppid', 'cmdline']):
                pid = proc.info['pid']
                if pid in tracked or pid == os.getpid() or not proc.info['cmdline']:
                    continue
                label = label_of(proc.info['cmdline'])
                if label:
                    try:
                        proc.cpu_percent(None)
                    except psutil.Error:
                        continue
                    tracked[pid] = [label, proc, exe_name(proc, proc.info['cmdline']),
                                    proc.info['ppid'], True]
            now = time.monotonic() - start
            total = psutil.cpu_percent(None)
            load = psutil.getloadavg()[0]
            writer.writerow([f'{now:.1f}', '_total', 0, 0, '', f'{total:.1f}', '', f'{load:.2f}'])
            rows.append(dict(time=f'{now:.1f}', label='_total(all cores %)', pid='0', exe='',
                             cpu_percent=f'{total:.1f}'))
            for pid, entry in list(tracked.items()):
                label, proc, exe, ppid, first = entry
                try:
                    with proc.oneshot():
                        cpu = proc.cpu_percent(None)
                        rss = proc.memory_info().rss / 1e6
                        threads = proc.num_threads()
                except psutil.Error:
                    del tracked[pid]
                    continue
                if first:
                    # The first reading covers less than one interval; drop it.
                    entry[4] = False
                    continue
                row = [f'{now:.1f}', label, pid, ppid, exe, f'{cpu:.1f}', f'{rss:.0f}', threads]
                writer.writerow(row)
                rows.append(dict(zip(COLUMNS, map(str, row))))
            handle.flush()
            time.sleep(args.interval)

    print_summary(out_path, rows, skip_seconds=0., ncpu=ncpu)
    print(f'\nCSV: {out_path}')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--label', default='run')
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--summarize', nargs='+', metavar='CSV',
                        help='print the summary of existing CSV files instead of recording')
    parser.add_argument('--skip', type=float, default=5.,
                        help='with --summarize: seconds to drop from the start (default 5)')
    parser.add_argument('--pids', action='store_true',
                        help='with --summarize: list every PID, not only multi-PID nodes')
    args = parser.parse_args()
    if args.summarize:
        for path in args.summarize:
            with open(path, newline='') as handle:
                rows = list(csv.DictReader(handle))
            print_summary(path, rows, args.skip, show_pids=args.pids)
        return 0
    return record(args)


if __name__ == '__main__':
    sys.exit(main())
