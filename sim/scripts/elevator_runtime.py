"""Isaac side of the simulated elevator: moves the car and doors with physics.

Runs inside the Isaac process (bundled rclpy, std_msgs only).  The ROS-typed
/elevator/command and /elevator/beacon are translated by the system-Python
``elevator_sim_relay`` node over JSON topics:
  /sim/elevator/command_raw  (in)   {"command", "floor_code", "direction", "sequence", "repeat"}
  /sim/elevator/beacon_raw   (out)  [{"event", "location", "floor_code", "sequence", ...}]
  /sim/elevator/command_status_raw (out) [{"cmd_seq", "status", "device_result", "target", ...}]
  /sim/elevator/status       (out)  ground truth for evaluation (latched)
  /sim/elevator/inject       (in)   {"faults": {...}} | {"obstruct": bool}
"""

import json
import math
import sys


class ElevatorRuntime:
    def __init__(self, stage, node, root, seed=0):
        from pxr import Gf
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        for package in ('amr_stack', 'amr_elevator'):
            sys.path.insert(0, str(root / 'src/amr_system' / package))
        from amr_elevator.geometry import point_in_polygon
        self.point_in_polygon = point_in_polygon
        from amr_stack.floor_config import load_floor_config
        from amr_sim_bridge.elevator_model import ElevatorSim
        self.Gf = Gf
        self.String = String
        building_prim = stage.GetPrimAtPath('/World/Building')
        self.elevator_id = building_prim.GetAttribute('elevator:id').Get()
        self.building = building = load_floor_config(root / 'src/amr_system/amr_stack/config/floors.yaml')
        self.floors = sorted(building.floors.values(), key=lambda f: f.level)
        self.names = [f.name for f in self.floors]
        reference = building.floor(building_prim.GetAttribute('referenceFloor').Get())
        self.reference_index = self.names.index(reference.name)
        self.height = building.sim.floor_height_m
        spec = building.elevators[self.elevator_id]
        self.model = ElevatorSim([(f.name, f.beacon_floor_code) for f in self.floors],
                                 floor_height_m=self.height, speed_m_s=building.sim.car_speed_m_s,
                                 accel_m_s2=building.sim.car_accel_m_s2, open_time_s=spec.door_open_time_s,
                                 seed=seed, start_floor=reference.name)
        self.geometry = building.elevator_geometry(reference.name, self.elevator_id)
        self.stop = reference.elevators[self.elevator_id]
        self.near = spec.threshold_depth_m
        self.door_half = spec.door_width_m / 2
        base = f'/World/Elevator/{self.elevator_id}'
        self.car = stage.GetPrimAtPath(base + '/Car').GetAttribute('xformOp:translate')
        self.car_doors = [self._door(stage.GetPrimAtPath(f'{base}/{n}')) for n in ('CarDoorLeft', 'CarDoorRight')]
        self.hall_doors = {}
        for floor in self.floors:
            root_path = f'/World/Building/Floor_{"".join(c if c.isalnum() else "_" for c in floor.name)}'
            prim = stage.GetPrimAtPath(root_path)
            from pxr import Usd
            self.hall_doors[floor.name] = [
                self._door(p) for p in Usd.PrimRange(prim) if p.GetAttribute('door:type').Get() == 'elevator']
        self.written = {}
        self.now = 0.
        self.obstructed = False
        self.next_status = 0.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.beacon_pub = node.create_publisher(String, '/sim/elevator/beacon_raw', 50)
        self.command_status_pub = node.create_publisher(String, '/sim/elevator/command_status_raw', 20)
        self.status_pub = node.create_publisher(String, '/sim/elevator/status', latched)
        node.create_subscription(String, '/sim/elevator/command_raw', self.on_command, 20)
        node.create_subscription(String, '/sim/elevator/inject', self.on_inject, 10)
        self.robot = (0., 0., 0.)
        self.apply(force=True)

    @staticmethod
    def _door(prim):
        return (prim.GetAttribute('xformOp:translate'), prim.GetAttribute('door:closedPosition').Get(),
                prim.GetAttribute('door:openPosition').Get())

    def floor_z(self, index):
        return (index - self.reference_index) * self.height

    def on_command(self, message):
        try:
            c = json.loads(message.data)
            self.model.command(self.now, int(c['command']), int(c['floor_code']), int(c.get('direction', 0)),
                               int(c['sequence']), int(c.get('repeat', 0)))
        except (ValueError, KeyError, TypeError):
            pass

    def on_inject(self, message):
        try:
            request = json.loads(message.data)
            if 'faults' in request:
                self.model.set_faults(**request['faults'])
            if 'obstruct' in request:
                self.model.obstruct(bool(request['obstruct']))
        except (ValueError, KeyError, TypeError):
            pass

    def step(self, elapsed, position):
        """Called after each physics step with the AMR base position."""
        self.now = elapsed
        x, y, z = (float(v) for v in position[:3])
        self.robot = (x, y, z)
        car_z = self.floor_z(self.model.position)
        level = self.reference_index + z / self.height
        in_car = self.point_in_polygon(x, y, self.geometry.car_polygon) and abs(z - car_z) < .5
        self.model.set_robot(level, in_car)
        yaw = self.stop.car_inward_yaw
        dx, dy = x - self.stop.door_center[0], y - self.stop.door_center[1]
        depth = dx * math.cos(yaw) + dy * math.sin(yaw)
        lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
        # Simulated safety edge: the AMR body in the doorway keeps the doors from closing.
        self.obstructed = (-.4 < depth < self.near + .4 and abs(lateral) < self.door_half + .3
                           and abs(z - car_z) < .5)
        self.model.obstruct(self.obstructed)
        frames = self.model.step(elapsed)
        if frames:
            self.beacon_pub.publish(self.String(data=json.dumps([frame.__dict__ for frame in frames])))
        statuses = self.model.take_statuses()
        if statuses:
            self.command_status_pub.publish(self.String(data=json.dumps([s.__dict__ for s in statuses])))
        self.apply()
        if elapsed >= self.next_status:
            self.publish_status()
            self.next_status = elapsed + .2

    def apply(self, force=False):
        Gf = self.Gf
        car_z = self.floor_z(self.model.position)
        ratio = self.model.door_ratio
        self._set(self.car, Gf.Vec3d(0., 0., car_z), force)
        for attribute, closed, opened in self.car_doors:
            self._set(attribute, Gf.Vec3d(closed[0] + (opened[0] - closed[0]) * ratio,
                                          closed[1] + (opened[1] - closed[1]) * ratio, car_z), force)
        at = None if self.model.motion else round(self.model.position)
        for index, name in enumerate(self.names):
            hall = ratio if index == at else 0.
            for attribute, closed, opened in self.hall_doors[name]:
                self._set(attribute, Gf.Vec3d(*(c + (o - c) * hall for c, o in zip(closed, opened))), force)

    def _set(self, attribute, value, force):
        key = attribute.GetPath()
        previous = self.written.get(key)
        if force or previous is None or any(abs(a - b) > 1e-6 for a, b in zip(previous, value)):
            attribute.Set(value)
            self.written[key] = tuple(value)

    def debug_contact(self, position, orientation, query, elapsed):
        """Diagnostics (AMR_ELEVATOR_DEBUG=1): what lies under the AMR when it tilts."""
        w, x, y, z = (float(v) for v in orientation)
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        if abs(roll) < 1.5 or getattr(self, '_debugged', 0) >= 3 or elapsed - getattr(self, '_debug_t', -9) < 1.:
            return
        self._debugged = getattr(self, '_debugged', 0) + 1
        self._debug_t = elapsed
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        report = []
        for fx in (-.3, -.2, 0., .2, .3):
            for fy in (-.27, -.22, 0., .22, .27):
                ox = position[0] + fx * math.cos(yaw) - fy * math.sin(yaw)
                oy = position[1] + fx * math.sin(yaw) + fy * math.cos(yaw)
                hits = []

                def collect(hit):
                    if not hit.rigid_body.startswith('/World/AMR'):
                        hits.append((round(float(hit.distance), 3), str(hit.collision)))
                    return True
                query.raycast_all((ox, oy, position[2] + .4), (0., 0., -1.), 1., collect)
                report.append(((fx, fy), (round(ox, 3), round(oy, 3)), sorted(hits)[:3]))
        print('AMR_ELEVATOR_DEBUG ' + json.dumps({'t': elapsed, 'roll': roll, 'pose': [float(v) for v in position],
                                                  'yaw': yaw, 'below': report}, default=float), flush=True)

    def publish_status(self):
        status = self.model.status()
        status.update(elevator_id=self.elevator_id, car_z_m=self.floor_z(self.model.position),
                      sim_time=self.now, robot_level=self.model.robot_level,
                      robot_in_car=self.model.robot_in_car, robot_z_m=self.robot[2],
                      robot_floor=self.names[min(range(len(self.names)),
                                                 key=lambda i: abs(i - self.model.robot_level))],
                      obstructed=self.obstructed)
        self.status_pub.publish(self.String(data=json.dumps(status, sort_keys=True)))
