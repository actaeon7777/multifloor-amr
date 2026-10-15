#!/usr/bin/env python3
"""One isolated multi-floor (elevator) trial in the stacked building, domain 107.

Starts ``experiment.sh sim world:=building``, initializes like run_sim_trial.py,
sends one /floor_mission/navigate goal (optionally with elevator faults) and
writes result.json, metrics.json and samples.json to a fresh output directory.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from amr_elevator_interfaces.action import NavigateToFloorPose  # noqa: E402
from amr_elevator_interfaces.msg import ElevatorDoorState, FloorEvent, FloorState  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

from amr_stack.floor_config import load_floor_config  # noqa: E402
from run_sim_trial import Trial  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def pose2d(pose):
    q = pose.orientation
    return [round(pose.position.x, 4), round(pose.position.y, 4),
            round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 4)]


class FloorTrial(Trial):
    def __init__(self, spawn):
        super().__init__(spawn)
        self.floor_action = ActionClient(self, NavigateToFloorPose, '/floor_mission/navigate')
        self.inject_pub = self.create_publisher(String, '/sim/elevator/inject', 10)
        self.elevator, self.floor_state, self.mission, self.door = {}, None, {}, None
        self.events, self.recovery_by_goal = [], {}
        self.estimate, self.wheel = None, None
        self.subs += [
            self.create_subscription(String, '/sim/elevator/status',
                                     lambda m: setattr(self, 'elevator', json.loads(m.data)), LATCHED),
            self.create_subscription(FloorState, '/floor/state', lambda m: setattr(self, 'floor_state', m), LATCHED),
            self.create_subscription(FloorEvent, '/floor/events', self.on_event, 50),
            self.create_subscription(String, '/floor_mission/status',
                                     lambda m: setattr(self, 'mission', json.loads(m.data)), LATCHED),
            self.create_subscription(ElevatorDoorState, '/elevator/door', lambda m: setattr(self, 'door', m), 10),
            self.create_subscription(PoseStamped, '/floor/pose_estimate', lambda m: setattr(self, 'estimate', m), 10),
            self.create_subscription(Odometry, '/wheel/odometry', lambda m: setattr(self, 'wheel', m), 10),
            self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback',
                                     self.on_nav_feedback, 20),
        ]
        self.floor_samples, self.last_floor_sample = [], 0.
        self.triggers, self.injected = [], []

    def on_event(self, message):
        self.events.append({'t': time.monotonic(), 'event': message.event, 'from': message.from_floor,
                            'to': message.to_floor, 'detail': message.detail,
                            'sim_time': self.elevator.get('sim_time')})

    def on_nav_feedback(self, message):
        key = bytes(message.goal_id.uuid)
        self.recovery_by_goal[key] = max(self.recovery_by_goal.get(key, 0), message.feedback.number_of_recoveries)

    def spin(self, seconds=.05):
        super().spin(seconds)
        now = time.monotonic()
        if self.truth is not None and now - self.last_floor_sample >= .1:
            self.last_floor_sample = now
            s = self.floor_state
            p = self.truth.pose.pose.position
            self.floor_samples.append({
                't': self.truth.header.stamp.sec + self.truth.header.stamp.nanosec / 1e9,
                'z': p.z, 'truth_floor': self.elevator.get('robot_floor'),
                'truth_in_car': self.elevator.get('robot_in_car'),
                'car_floor': self.elevator.get('car_floor'), 'car_door': self.elevator.get('door'),
                'zone': None if s is None else s.zone, 'floor': None if s is None else s.floor,
                'confidence': None if s is None else s.confidence,
                'door': None if self.door is None else self.door.state,
                'phase': self.mission.get('phase'),
                'rtf': self.sim_status.get('real_time_factor'),
                'truth_xyyaw': pose2d(self.truth.pose.pose),
                'odom': None if self.odom is None else pose2d(self.odom.pose.pose),
                'wheel': None if self.wheel is None else pose2d(self.wheel.pose.pose),
                'monitor': None if self.estimate is None else pose2d(self.estimate.pose),
            })
        for trigger in list(self.triggers):
            if self.mission.get('phase') == trigger['phase']:
                self.inject_pub.publish(String(data=json.dumps(trigger['inject'])))
                self.triggers.remove(trigger)
                self.injected.append({'phase': trigger['phase'], 'inject': trigger['inject'],
                                      'sim_time': self.elevator.get('sim_time')})

    def inject(self, payload, timeout=15.):
        """Publish until the simulated elevator reports the faults (volatile topic, late discovery)."""
        faults = payload.get('faults', {})
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.inject_pub.publish(String(data=json.dumps(payload)))
            self.spin(.5)
            active = self.elevator.get('faults', {})
            if all(active.get(k) == v for k, v in faults.items()):
                self.injected.append({'phase': 'START', 'inject': payload, 'sim_time': self.elevator.get('sim_time')})
                return
        raise RuntimeError(f'simulated elevator did not apply faults {faults}')

    def floor_goal(self, floor, room, pose, timeout):
        goal = NavigateToFloorPose.Goal(floor=floor, destination_id=room)
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.until(lambda: self.floor_action.server_is_ready(), 30)
        sent = self.floor_action.send_goal_async(goal)
        self.until(sent.done, 10)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('floor_mission_manager rejected the goal')
        result = handle.get_result_async()
        try:
            self.until(result.done, timeout)
        except TimeoutError:
            handle.cancel_goal_async()
            self.spin(2.)
            raise
        self.spin(1.)
        return result.result()


def evaluate(node, report, building, goal_floor, goal_pose, floor_z):
    metrics = {'world': 'building', 'calibration_status': 'uncalibrated'}
    samples = node.floor_samples
    truth = node.truth.pose.pose if node.truth else None
    if truth is not None:
        q = truth.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        metrics['arrival_distance_m'] = math.dist((truth.position.x, truth.position.y), goal_pose[:2])
        metrics['arrival_yaw_error_rad'] = abs((yaw - goal_pose[2] + math.pi) % (2 * math.pi) - math.pi)
        metrics['arrival_z_m'] = truth.position.z
        metrics['arrived_on_target_floor'] = abs(truth.position.z - floor_z[goal_floor]) < .2
    if samples:
        metrics['duration_s'] = samples[-1]['t'] - samples[0]['t']
        rtf = [s['rtf'] for s in samples if isinstance(s['rtf'], (int, float))]
        metrics['real_time_factor'] = {'mean': sum(rtf) / len(rtf), 'min': min(rtf)} if rtf else None
        # Floor/zone agreement with ground truth (transit samples have no floor by design).
        checked = mismatched = 0
        mismatch_times = []
        for s in samples:
            if s['zone'] is None or s['truth_in_car'] is None:
                continue
            checked += 1
            zone_ok = (s['zone'] == FloorState.ZONE_IN_ELEVATOR) == bool(s['truth_in_car'])
            floor_ok = not s['floor'] or s['floor'] == s['truth_floor']
            if not (zone_ok and floor_ok):
                mismatched += 1
                mismatch_times.append(s['t'])
        longest, run, previous = 0., 0., None
        for t in mismatch_times:
            run = run + (t - previous) if previous is not None and t - previous < .25 else 0.
            longest, previous = max(longest, run), t
        metrics['floor_zone_agreement'] = {'samples': checked, 'mismatched': mismatched,
                                           'ratio': 1 - mismatched / checked if checked else None,
                                           'longest_mismatch_s': longest,
                                           'final_match': bool(samples[-1]['floor'] == samples[-1]['truth_floor']
                                                               and samples[-1]['zone'] == FloorState.ZONE_OUTSIDE)}
    metrics['obstacle_contact_events'] = node.sim_status.get('obstacle_contact_events')
    metrics['nav2_recoveries'] = sum(node.recovery_by_goal.values())
    metrics['mission_counters'] = node.mission.get('counters', {})
    metrics['phase_timings_s'] = node.mission.get('timings', {})
    metrics['floor_events'] = [e['event'] for e in node.events]
    metrics['success'] = bool(report.get('mission_success'))
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--floor', required=True, help='destination floor, e.g. 3F')
    parser.add_argument('--room', default='', help='room id on that floor (uses floors.yaml goal)')
    parser.add_argument('--goal', nargs=3, type=float, help='explicit goal pose instead of --room')
    parser.add_argument('--start-floor', default='4F', help='floor the AMR starts on (spawn x/y from floors.yaml)')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--fault', default='{}', help='JSON for /sim/elevator/inject at start')
    parser.add_argument('--fault-at', action='append', default=[],
                        help='PHASE=JSON: inject when the mission enters PHASE (repeatable)')
    parser.add_argument('--timeout', type=float, default=600.)
    parser.add_argument('--bag', action='store_true', help='record a diagnostic rosbag')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '107':
        raise SystemExit('Only domain 107 is allowed. Source env, then export ROS_DOMAIN_ID=107.')
    if args.output.exists():
        raise SystemExit('Output already exists; each trial requires a fresh directory')
    building = load_floor_config()
    floor = building.floor(args.floor)
    if args.goal:
        goal_pose, room = tuple(args.goal), args.room
    else:
        room = args.room or sorted(floor.rooms)[0]
        goal_pose = floor.rooms[room].goal
    world = json.loads((ROOT / 'sim/config/building.json').read_text())
    floor_z = world['floor_z']
    args.output.mkdir(parents=True)
    log = (args.output / 'launch.log').open('w')
    process = subprocess.Popen([str(ROOT / 'scripts/experiment.sh'), 'sim', 'headless:=true', 'start_joy:=false',
                                'world:=building', f'seed:={args.seed}', f'start_floor:={args.start_floor}'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    recorder = None
    if args.bag:
        bag_log = (args.output / 'bag.log').open('w')
        recorder = subprocess.Popen(
            ['ros2', 'bag', 'record', '--use-sim-time', '-s', 'sqlite3', '-o', str(args.output / 'bag'),
             '/clock', '/scan', '/odometry/filtered', '/wheel/odometry', '/amcl_pose', '/tf', '/tf_static', '/plan',
             '/local_costmap/costmap', '/cmd_vel', '/cmd_vel_auto', '/localization/status', '/navigation/status',
             '/sim/ground_truth', '/sim/status', '/sim/elevator/status', '/floor/state', '/floor/events',
             '/floor/pose_estimate', '/floor_mission/status', '/elevator/door', '/elevator/beacon',
             '/elevator/command', '/map', '/initialpose', '/rosout'],
            stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
    report = {'passed': False, 'world': 'building', 'seed': args.seed, 'start_floor': args.start_floor,
              'goal_floor': args.floor, 'goal_room': room, 'goal': list(goal_pose),
              'fault': json.loads(args.fault), 'fault_at': args.fault_at, 'calibration_status': 'uncalibrated'}
    rclpy.init(args=[])
    node = FloorTrial(world['spawn'])
    for item in args.fault_at:
        phase, payload = item.split('=', 1)
        node.triggers.append({'phase': phase, 'inject': json.loads(payload)})
    try:
        node.initialize()
        node.until(lambda: node.floor_state is not None and node.floor_state.confidence in ('BEACON', 'CONFIRMED'), 30)
        if report['fault']:
            node.inject({'faults': report['fault']})
        node.samples.clear()
        node.floor_samples.clear()
        started = time.monotonic()
        result = node.floor_goal(args.floor, room, goal_pose, args.timeout).result
        report['wall_s'] = time.monotonic() - started
        report['mission_success'] = result.success
        report['error_code'] = result.error_code
        report['message'] = result.message
        report['stopped'] = bool(node.odom and abs(node.odom.twist.twist.linear.x) < .03
                                 and abs(node.odom.twist.twist.angular.z) < .05)
        report['simulation'] = node.sim_status
        report['elevator'] = node.elevator
    except Exception as exc:  # noqa: BLE001
        report['error'] = str(exc)
    finally:
        metrics = evaluate(node, report, building, args.floor, goal_pose, floor_z)
        report['passed'] = bool(report.get('mission_success') and report.get('stopped')
                                and metrics.get('obstacle_contact_events') == 0
                                and metrics.get('arrived_on_target_floor')
                                and metrics.get('arrival_distance_m', 9.) < .5)
        report['events'] = node.events
        report['injected'] = node.injected
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        if recorder is not None and recorder.poll() is None:
            os.killpg(recorder.pid, signal.SIGINT)
            try:
                recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(recorder.pid, signal.SIGTERM)
        log.close()
        (args.output / 'samples.json').write_text(json.dumps({'pose': node.samples, 'floor': node.floor_samples}) + '\n')
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        (args.output / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'events'}, indent=2))
    print(json.dumps(metrics, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
