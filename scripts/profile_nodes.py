#!/usr/bin/env python3
"""Record per-process CPU/RSS of the running AMR stack at 1 Hz until Ctrl+C.

Usage:
    python3 scripts/profile_nodes.py --label before_opt
Writes reports/profile/<date>_<label>.csv (one row per process per second) and
prints a per-node summary (mean / p95 / max CPU %) on exit. Only psutil is used,
so the sampler itself costs well under 1 % of a core.
"""
import argparse
import csv
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


def label_of(cmdline):
    text = ' '.join(cmdline)
    for pattern in PATTERNS:
        if pattern in text:
            if pattern == 'ros2 launch':
                return 'ros2_launch'
            return pattern
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', default='run')
    parser.add_argument('--interval', type=float, default=1.0)
    args = parser.parse_args()

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

    tracked = {}  # pid -> (label, psutil.Process)
    samples = defaultdict(list)  # label -> [cpu %]
    ncpu = psutil.cpu_count()
    psutil.cpu_percent(None)

    with out_path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['time', 'label', 'pid', 'cpu_percent', 'rss_mb', 'threads'])
        print(f'기록 중: {out_path}\n주행이 끝나면 Ctrl+C 로 멈추세요.', flush=True)
        start = time.monotonic()
        while not stop:
            # Refresh the process table so nodes started later are picked up.
            for proc in psutil.process_iter(['pid', 'cmdline']):
                pid = proc.info['pid']
                if pid in tracked or not proc.info['cmdline']:
                    continue
                label = label_of(proc.info['cmdline'])
                if label:
                    try:
                        proc.cpu_percent(None)
                    except psutil.Error:
                        continue
                    tracked[pid] = (label, proc)
            now = time.monotonic() - start
            total = psutil.cpu_percent(None)
            load = psutil.getloadavg()[0]
            writer.writerow([f'{now:.1f}', '_total', 0, f'{total:.1f}', '', f'{load:.2f}'])
            samples['_total(all cores %)'].append(total)
            samples['_loadavg1'].append(load)
            for pid, (label, proc) in list(tracked.items()):
                try:
                    with proc.oneshot():
                        cpu = proc.cpu_percent(None)
                        rss = proc.memory_info().rss / 1e6
                        threads = proc.num_threads()
                except psutil.Error:
                    del tracked[pid]
                    continue
                writer.writerow([f'{now:.1f}', label, pid, f'{cpu:.1f}', f'{rss:.0f}', threads])
                samples[label].append(cpu)
            handle.flush()
            time.sleep(args.interval)

    print(f'\n요약 ({len(samples["_loadavg1"])} 초, CPU {ncpu}코어, 1코어 = 100 %)')
    print(f'{"node":28s} {"mean":>7s} {"p95":>7s} {"max":>7s}')
    rows = []
    for label, values in samples.items():
        values = sorted(values)
        if not values:
            continue
        mean = sum(values) / len(values)
        p95 = values[min(len(values) - 1, int(len(values) * 0.95))]
        rows.append((mean, label, p95, values[-1]))
    for mean, label, p95, peak in sorted(rows, reverse=True):
        print(f'{label:28s} {mean:7.1f} {p95:7.1f} {peak:7.1f}')
    print(f'\nCSV: {out_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
