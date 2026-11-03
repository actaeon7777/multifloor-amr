"""Typed elevator requests, simulated human presses, and arrival recognition.

The standalone adapter and Isaac JSON relay share the same auto-confirm policy.
/sim/elevator/status remains ground truth for animation and evaluation.
"""

import json
import math

from amr_elevator_interfaces.msg import ElevatorRequest, FloorArrivalConfirm, FloorState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from amr_sim_bridge.elevator_model import ElevatorSim


def point_in_polygon(x, y, polygon):
    inside = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


class ArrivalRecognizer:
    """One answer per candidate ID; repeated states and withdrawn candidates do not rearm it."""
    def __init__(self, node, auto_confirm=True, confirm_delay_s=1.0):
        if not math.isfinite(confirm_delay_s) or confirm_delay_s < 0:
            raise ValueError('confirm_delay_s must be nonnegative')
        self.node, self.enabled, self.delay = node, auto_confirm, confirm_delay_s
        self.candidate = None
        self.answered_id = None
        self.publisher = node.create_publisher(FloorArrivalConfirm, '/floor/arrival_confirm', 10)
        node.create_subscription(FloorState, '/floor/state', self.on_state,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_state(self, state):
        if self.answered_id is not None and state.candidate_id < self.answered_id:
            self.answered_id = None  # the monitor restarted and counts from 1 again
        if not state.awaiting_confirm or state.zone != FloorState.ZONE_IN_ELEVATOR:
            self.candidate = None
        elif self.candidate is None or self.candidate['candidate_id'] != state.candidate_id:
            self.candidate = dict(floor=state.floor, candidate_id=state.candidate_id,
                                  due=self.now() + self.delay)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def tick(self, status):
        c = self.candidate
        if not self.enabled or c is None or c['candidate_id'] == self.answered_id or self.now() < c['due'] or status is None:
            return
        message = FloorArrivalConfirm(source=FloorArrivalConfirm.SOURCE_SIM, floor=c['floor'],
            candidate_id=c['candidate_id'],
            accepted=(status.get('moving') is False and status.get('car_floor') == c['floor']
                      and status.get('door') == 'OPEN'), score=1.0)
        if not message.accepted and status.get('moving') is False and status.get('door') == 'OPEN':
            message.actual_floor = status.get('car_floor') or ''
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(message)
        self.answered_id = c['candidate_id']


class ElevatorRosAdapter:
    def __init__(self, node, building, elevator_id='E1', start_floor=None, seed=0,
                 reference_floor=None, auto_confirm=True, confirm_delay_s=1.0, **model_options):
        self.node = node
        self.building = building
        self.elevator_id = elevator_id
        ordered = sorted([*building.floors.values(), *building.pass_floors.values()], key=lambda f: f.level)
        self.names = [f.name for f in ordered]
        spec = building.elevators[elevator_id]
        self.reference_index = self.names.index(reference_floor or building.default_floor)
        self.model = ElevatorSim(
            self.names,
            floor_height_m=building.sim.floor_height_m, speed_m_s=building.sim.car_speed_m_s,
            accel_m_s2=building.sim.car_accel_m_s2, open_time_s=spec.door_open_time_s,
            seed=seed, start_floor=start_floor or building.default_floor, **model_options)
        self.model.set_robot(self.reference_index, False)
        self.robot_override = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = node.create_publisher(String, 'sim/elevator/status', latched)
        node.create_subscription(ElevatorRequest, '/elevator/request', self.on_request, latched)
        self.recognizer = ArrivalRecognizer(node, auto_confirm, confirm_delay_s)
        node.create_subscription(String, 'sim/elevator/inject', self.on_inject, 10)
        node.create_subscription(Odometry, 'sim/ground_truth', self.on_ground_truth, 10)
        self.last_status = None
        self.next_status = 0.

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def floor_z(self, index):
        return (index - self.reference_index) * self.building.sim.floor_height_m

    def car_z(self):
        return self.floor_z(self.model.position)

    def on_request(self, message):
        self.model.request(self.now(), message.kind, message.start_floor, message.target_floor,
                           message.direction)

    def on_inject(self, message):
        try:
            request = json.loads(message.data)
            if 'faults' in request:
                self.model.set_faults(**request['faults'])
            if 'obstruct' in request:
                self.model.obstruct(request['obstruct'])
            if 'robot' in request:
                robot = request['robot']
                self.robot_override = True
                self.model.set_robot(self.names.index(robot['floor']), bool(robot.get('in_car')))
        except (ValueError, KeyError, TypeError) as error:
            self.node.get_logger().error(f'Invalid elevator injection: {error}')

    def on_ground_truth(self, message):
        if self.robot_override:
            return
        position = message.pose.pose.position
        level = self.reference_index + position.z / self.building.sim.floor_height_m
        index = min((i for i, name in enumerate(self.names) if name in self.building.floors),
                    key=lambda i: abs(i - level))
        geometry = self.building.elevator_geometry(self.names[index], self.elevator_id)
        in_car = (point_in_polygon(position.x, position.y, list(geometry.car_polygon))
                  and abs(position.z - self.car_z()) < .5)
        self.model.set_robot(level, in_car)

    def tick(self, now=None):
        now = self.now() if now is None else now
        self.model.step(now)
        status = self.model.status()
        status.update(elevator_id=self.elevator_id, car_z_m=self.car_z(), sim_time=now,
                      robot_level=self.model.robot_level, robot_in_car=self.model.robot_in_car)
        self.recognizer.tick(status)
        key = {k: v for k, v in status.items() if k not in ('sim_time', 'hold_remaining_s')}
        if key != self.last_status or now >= self.next_status:
            self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
            self.last_status = key
            self.next_status = now + .5
        return status


class ElevatorSimNode(Node):
    def __init__(self, building=None, **kwargs):
        super().__init__('elevator_sim', **kwargs)
        from amr_stack.floor_config import load_floor_config
        path = self.declare_parameter('floors_config', '').value
        start = self.declare_parameter('start_floor', '').value or None
        seed = int(self.declare_parameter('seed', 0).value)
        press = float(self.declare_parameter('press_delay_s', 1.0).value)
        confirm = float(self.declare_parameter('confirm_delay_s', 1.0).value)
        auto = bool(self.declare_parameter('auto_confirm', True).value)
        building = building or load_floor_config(path or None)
        self.adapter = ElevatorRosAdapter(self, building, start_floor=start, seed=seed,
                                          press_delay_s=press, confirm_delay_s=confirm, auto_confirm=auto)
        self.create_timer(.02, self.tick)
        self.get_logger().info(
            f'Simulated elevator {self.adapter.elevator_id}: floors {self.adapter.names}, '
            f'car at {self.adapter.model.status()["car_floor"]}')

    def tick(self):
        if self.get_clock().now().nanoseconds > 0:  # wait for /clock with use_sim_time
            self.adapter.tick()


class ElevatorRelayNode(Node):
    """Translate typed requests to Isaac JSON and recognize arrival from fresh ground truth."""
    def __init__(self, **kwargs):
        super().__init__('elevator_sim_relay', **kwargs)
        auto = bool(self.declare_parameter('auto_confirm', True).value)
        delay = float(self.declare_parameter('confirm_delay_s', 1.0).value)
        self.recognizer = ArrivalRecognizer(self, auto, delay)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.raw_pub = self.create_publisher(String, '/sim/elevator/request_raw', latched)
        self.create_subscription(ElevatorRequest, '/elevator/request', self.on_request, latched)
        self.create_subscription(String, '/sim/elevator/status', self.on_status, latched)
        self.status, self.status_at = None, -math.inf
        self.create_timer(.05, self.tick)

    def on_request(self, message):
        self.raw_pub.publish(String(data=json.dumps({key: getattr(message, key) for key in
            ('kind', 'start_floor', 'target_floor', 'direction', 'sequence')})))

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
            self.status, self.status_at = status, self.recognizer.now()
        except (ValueError, TypeError):
            pass

    def tick(self):
        if 0 <= self.recognizer.now() - self.status_at < 1.0:
            self.recognizer.tick(self.status)


def relay_main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ElevatorRelayNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ElevatorSimNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
