#!/usr/bin/env python3
"""Watch multi-floor missions in the Isaac Sim GUI (stacked building, domain 107).

Starts ``experiment.sh sim world:=building`` with the Isaac window, switches to AUTO
with the same synthetic Joy heartbeat as run_sim_trial.py, then sends the given
floor/room goals in order through /floor_mission/navigate and prints each phase.
The simulation keeps running afterwards until Ctrl-C.

  scripts/floor_demo.py 3F:305 5F:505
  scripts/floor_demo.py            # no stops: only initialize and hold AUTO, then use the web UI

Demo video: pass --events and --exit-when-done, and export AMR_SIM_HOOK=sim/scripts/demo_recorder.py
plus AMR_DEMO_OUTPUT (see scripts/compose_demo_video.py); destination discs are added automatically.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402

from amr_stack.floor_config import load_floor_config  # noqa: E402
from run_floor_trial import FloorTrial, NavigateToFloorPose  # noqa: E402
from run_sim_trial import ROOT  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('stops', nargs='*', help='FLOOR:ROOM, e.g. 3F:305 5F:505 (none: web UI only)')
    parser.add_argument('--start-floor', default='4F')
    parser.add_argument('--timeout', type=float, default=600., help='per mission, wall-clock seconds')
    parser.add_argument('--events', type=Path, help='JSONL of mission phases and arrivals (sim time)')
    parser.add_argument('--exit-when-done', action='store_true', help='stop the simulation after the last stop')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '107':
        raise SystemExit('Only domain 107 is allowed. Source scripts/env.sh, then export ROS_DOMAIN_ID=107.')
    building = load_floor_config()
    stops = []
    for stop in args.stops:
        floor, _, room = stop.partition(':')
        stops.append((floor, room, building.floor(floor).rooms[room].goal))
    if os.environ.get('AMR_SIM_HOOK'):
        # Destination discs for sim/scripts/demo_recorder.py (inherited by the Isaac process).
        os.environ['AMR_DEMO_MARKERS'] = json.dumps(
            [{'floor': floor, 'room': room, 'x': pose[0], 'y': pose[1]} for floor, room, pose in stops])
    events = args.events.open('w') if args.events else None
    log = (ROOT / 'logs/floor_demo_launch.log').open('w')
    process = subprocess.Popen([str(ROOT / 'scripts/experiment.sh'), 'sim', 'headless:=false', 'start_joy:=false',
                                'world:=building', f'start_floor:={args.start_floor}'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    world = json.loads((ROOT / 'sim/config/building.json').read_text())
    rclpy.init(args=[])
    node = FloorTrial(world['spawn'])

    def event(kind, **fields):
        if events:
            stamp = node.truth.header.stamp if node.truth is not None else None
            sim_time = stamp.sec + stamp.nanosec * 1e-9 if stamp else None
            events.write(json.dumps({'event': kind, 'sim_time': sim_time, **fields}, ensure_ascii=False) + '\n')
            events.flush()
    try:
        print('Isaac Sim, 위치 추정, Nav2를 기다립니다 (GUI 첫 실행은 몇 분 걸릴 수 있음)...', flush=True)
        node.initialize()
        node.until(lambda: node.floor_state is not None and node.floor_state.confidence in ('BEACON', 'CONFIRMED'), 60)
        print(f'AUTO 전환 완료. 현재 층 {node.floor_state.floor}.', flush=True)
        event('ready', floor=node.floor_state.floor)
        for floor, room, pose in stops:
            print(f'\n=== {node.floor_state.floor} → {floor} {room}호 ===', flush=True)
            goal = NavigateToFloorPose.Goal(floor=floor, destination_id=room)
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
            node.until(lambda: node.localization_status.get('ready') is True, 120)
            sent = node.floor_action.send_goal_async(goal)
            node.until(sent.done, 10)
            handle = sent.result()
            if not handle.accepted:
                print('  목표가 거절되었습니다.', flush=True)
                event('rejected', floor=floor, room=room)
                continue
            event('goal', start_floor=node.floor_state.floor, floor=floor, room=room)
            result = handle.get_result_async()
            phase = None
            deadline = time.monotonic() + args.timeout
            while not result.done() and time.monotonic() < deadline:
                node.spin(.2)
                current = node.mission.get('phase')
                if current != phase:
                    phase = current
                    print(f'  [{node.elevator.get("sim_time", 0):6.1f} s] {phase}', flush=True)
                    event('phase', phase=phase, floor=floor, room=room)
            if not result.done():
                print('  시간 초과: 목표를 취소합니다.', flush=True)
                handle.cancel_goal_async()
                node.spin(2.)
                event('timeout', floor=floor, room=room)
                continue
            response = result.result()
            p = node.truth.pose.pose.position
            error = math.hypot(p.x - pose[0], p.y - pose[1])
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                print(f'  도착: 실제 위치 ({p.x:.2f}, {p.y:.2f}, z {p.z:.2f}), 오차 {error:.2f} m', flush=True)
                event('arrived', floor=floor, room=room, error_m=round(error, 3))
            else:
                print(f'  실패: {response.result.error_code} {response.result.message}', flush=True)
                event('failed', floor=floor, room=room, error_code=response.result.error_code,
                      message=response.result.message)
        if args.exit_when_done:
            node.spin(4.)  # a few seconds of the robot at the last door
            event('end')
            print('\n모든 목적지를 마쳤습니다. 시뮬레이션을 종료합니다.', flush=True)
            return
        print('\n' + ('모든 목적지를 마쳤습니다. ' if stops else '웹 UI에서 층·호실을 선택하세요. ') +
              'AUTO를 유지하며 시뮬레이션을 계속 실행합니다 (Ctrl-C로 종료).', flush=True)
        while True:
            node.spin(1.)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
        log.close()
        if events:
            events.close()


if __name__ == '__main__':
    main()
