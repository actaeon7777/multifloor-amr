#!/usr/bin/env python3
"""Watch the AMR drive autonomously in the Isaac Sim GUI. Simulation only (domain 107).

Starts the full sim stack with the Isaac window (RViz only with --rviz), waits for control_mux
AUTO (no joystick is started) as in run_sim_trial.py, then sends a goal
loop. Failed goals are reported and skipped. Ctrl-C stops everything.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess

import rclpy
from action_msgs.msg import GoalStatus

from run_sim_trial import ROOT, Trial

# Corridor goals checked against the floor4 map (x, y, yaw); spawn is (11.5, 3.31).
ROUTES = {
    # Lobby -> east corridor (y -1.2..1.7) -> north corridor (x 14.5..17) -> start.
    'floor4': [(15.5, 3.31, 0.), (25., .2, 0.), (15.7, 12., math.pi/2), (11.5, 3.31, math.pi)],
    'arena': [(2., 0., 0.), (0., 0., math.pi)],
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='floor4', choices=sorted(ROUTES))
    parser.add_argument('--loops', type=int, default=1, help='0 repeats until Ctrl-C')
    parser.add_argument('--timeout', type=float, default=180., help='Per-goal wall-clock limit')
    parser.add_argument('--rviz', action='store_true', help='Also open RViz (the web UI is the normal operator view)')
    parser.add_argument('--room', nargs='+', help='floor4 room numbers to visit in order (sim/config/floor4_rooms.json)')
    args = parser.parse_args()
    route = ROUTES[args.world]
    if args.room:
        if args.world != 'floor4':
            parser.error('--room requires --world floor4')
        rooms = json.loads((ROOT/'sim/config/floor4_rooms.json').read_text())['rooms']
        unknown = [room for room in args.room if room not in rooms]
        if unknown:
            parser.error(f'unknown room {unknown}; known: {sorted(rooms)}')
        route = [tuple(rooms[room]['goal']) for room in args.room]
    if os.environ.get('ROS_DOMAIN_ID') != '107':
        raise SystemExit('Only domain 107 is allowed. Source scripts/env.sh, then export ROS_DOMAIN_ID=107.')

    log_dir = ROOT/'logs'; log_dir.mkdir(exist_ok=True)
    log = (log_dir/'sim_demo_launch.log').open('w')
    process = subprocess.Popen([str(ROOT/'scripts/experiment.sh'), 'sim', 'headless:=false', 'start_joy:=false',
                                f'world:={args.world}', f'use_rviz:={str(args.rviz).lower()}'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    spawn = json.loads((ROOT/f'sim/config/{args.world}.json').read_text()).get('spawn', [0., 0., 0.])
    rclpy.init(args=[])
    node = Trial(spawn)
    try:
        print('Waiting for Isaac Sim, localization and Nav2 (first GUI start can take a few minutes)...', flush=True)
        node.initialize()
        print('AUTO mode on. Driving.', flush=True)
        loop = 0
        while args.loops == 0 or loop < args.loops:
            loop += 1
            for x, y, yaw in route:
                print(f'[loop {loop}] goal ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)', flush=True)
                try:
                    # navigation_manager rejects goals while localization is not READY
                    # (e.g. briefly after an AMCL/TF mismatch); wait for it to recover.
                    if not node.localization_status.get('ready'):
                        print('  waiting for localization READY...', flush=True)
                    node.until(lambda: node.localization_status.get('ready') is True, 60)
                    status = node.goal(x, y, yaw, args.timeout)
                    p, q = node.truth.pose.pose.position, node.truth.pose.pose.orientation
                    truth_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    error = (f'truth ({p.x:.2f}, {p.y:.2f}, {math.degrees(truth_yaw):.0f} deg), '
                             f'error {math.hypot(p.x-x, p.y-y):.2f} m / '
                             f'{abs(math.degrees(math.remainder(truth_yaw-yaw, math.tau))):.0f} deg')
                    print(f'  reached; {error}' if status == GoalStatus.STATUS_SUCCEEDED
                          else f'  failed (status {status}); {error}; skipping', flush=True)
                except (TimeoutError, RuntimeError) as error:
                    print(f'  {error}; skipping', flush=True)
        print('Route finished. Robot holds position; Ctrl-C to quit.', flush=True)
        while process.poll() is None:
            node.spin(.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(30)
            except subprocess.TimeoutExpired: os.killpg(process.pid, signal.SIGKILL)


if __name__ == '__main__':
    main()
