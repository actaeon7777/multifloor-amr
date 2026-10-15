"""ROS adapter for the simulated elevator (replaces RF -> STM32 -> CAN in simulation).

Publishes the same /elevator/beacon the real elevator_can_link does and consumes
/elevator/command.  ``ElevatorRosAdapter`` can be attached to any node, so the
Isaac runner can step it with physics; ``elevator_sim`` runs it standalone for
ROS-only tests.  /sim/elevator/status (JSON) is ground truth for evaluation and
must not be used by AMR logic.
"""

import json
import math

from amr_elevator_interfaces.msg import ElevatorBeacon, ElevatorCommand
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String

from amr_sim_bridge.elevator_model import ElevatorSim


def point_in_polygon(x, y, polygon):
    inside = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


class ElevatorRosAdapter:
    def __init__(self, node, building, elevator_id='E1', start_floor=None, seed=0,
                 reference_floor=None, **model_options):
        self.node = node
        self.building = building
        self.elevator_id = elevator_id
        ordered = sorted(building.floors.values(), key=lambda f: f.level)
        self.names = [f.name for f in ordered]
        spec = building.elevators[elevator_id]
        self.reference_index = self.names.index(reference_floor or building.default_floor)
        self.model = ElevatorSim(
            [(f.name, f.beacon_floor_code) for f in ordered],
            floor_height_m=building.sim.floor_height_m, speed_m_s=building.sim.car_speed_m_s,
            accel_m_s2=building.sim.car_accel_m_s2, open_time_s=spec.door_open_time_s,
            seed=seed, start_floor=start_floor or building.default_floor, **model_options)
        self.model.set_robot(self.reference_index, False)
        self.robot_override = False
        self.beacon_pub = node.create_publisher(ElevatorBeacon, 'elevator/beacon', 20)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = node.create_publisher(String, 'sim/elevator/status', latched)
        node.create_subscription(ElevatorCommand, 'elevator/command', self.on_command, 10)
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

    def on_command(self, message):
        self.model.command(self.now(), message.command, message.floor_code, message.direction,
                           message.sequence)

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
        index = min(range(len(self.names)), key=lambda i: abs(i - level))
        geometry = self.building.elevator_geometry(self.names[index], self.elevator_id)
        in_car = (point_in_polygon(position.x, position.y, list(geometry.car_polygon))
                  and abs(position.z - self.car_z()) < .5)
        self.model.set_robot(level, in_car)

    def tick(self, now=None):
        now = self.now() if now is None else now
        for frame in self.model.step(now):
            beacon = ElevatorBeacon(
                event=frame.event, location=frame.location, floor_code=frame.floor_code,
                sequence=frame.sequence, ack_sequence=frame.ack_sequence,
                rssi_dbm=max(-128, min(127, frame.rssi_dbm)), source='sim')
            beacon.header.stamp = Time(nanoseconds=round(max(0., frame.stamp) * 1e9)).to_msg()
            self.beacon_pub.publish(beacon)
        status = self.model.status()
        status.update(elevator_id=self.elevator_id, car_z_m=self.car_z(), sim_time=now,
                      robot_level=self.model.robot_level, robot_in_car=self.model.robot_in_car)
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
        leak = float(self.declare_parameter('leak_probability', .2).value)
        building = building or load_floor_config(path or None)
        self.adapter = ElevatorRosAdapter(self, building, start_floor=start, seed=seed,
                                          leak_probability=leak)
        self.create_timer(.02, self.tick)
        self.get_logger().info(
            f'Simulated elevator {self.adapter.elevator_id}: floors {self.adapter.names}, '
            f'car at {self.adapter.model.status()["car_floor"]}')

    def tick(self):
        if self.get_clock().now().nanoseconds > 0:  # wait for /clock with use_sim_time
            self.adapter.tick()


class ElevatorRelayNode(Node):
    """Typed elevator topics <-> the Isaac process's JSON topics (Isaac has no custom msgs)."""

    def __init__(self, **kwargs):
        super().__init__('elevator_sim_relay', **kwargs)
        self.beacon_pub = self.create_publisher(ElevatorBeacon, 'elevator/beacon', 50)
        self.raw_pub = self.create_publisher(String, 'sim/elevator/command_raw', 20)
        self.create_subscription(ElevatorCommand, 'elevator/command', self.on_command, 20)
        self.create_subscription(String, 'sim/elevator/beacon_raw', self.on_raw, 50)

    def on_command(self, message):
        self.raw_pub.publish(String(data=json.dumps({
            'command': message.command, 'floor_code': message.floor_code,
            'direction': message.direction, 'sequence': message.sequence})))

    def on_raw(self, message):
        try:
            frames = json.loads(message.data)
            for frame in frames:
                beacon = ElevatorBeacon(
                    event=int(frame['event']), location=int(frame['location']),
                    floor_code=int(frame['floor_code']), sequence=int(frame['sequence']) & 0xff,
                    ack_sequence=int(frame['ack_sequence']) & 0xff,
                    rssi_dbm=max(-128, min(127, int(frame['rssi_dbm']))), source='sim')
                beacon.header.stamp = Time(nanoseconds=round(max(0., float(frame['stamp'])) * 1e9)).to_msg()
                self.beacon_pub.publish(beacon)
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().error(f'Invalid simulated beacon frame: {error}')


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
