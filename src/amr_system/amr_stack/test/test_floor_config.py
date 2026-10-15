"""floors.yaml must stay the single, consistent source for multi-floor data."""

import copy
import hashlib
import json
import math
from pathlib import Path
import re

import pytest
import yaml

from amr_stack.floor_config import (
    FloorConfigError, find_workspace_root, load_floor_config, load_occupancy_map,
    parse_floor_config,
)

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config/floors.yaml'
WORKSPACE = find_workspace_root(PACKAGE)
ROBOT_CLEARANCE_M = 0.3


@pytest.fixture(scope='module')
def config():
    return load_floor_config(CONFIG, package_root=PACKAGE)


@pytest.fixture(scope='module')
def raw():
    return yaml.safe_load(CONFIG.read_text(encoding='utf-8'))


def test_three_floors_with_unique_beacons(config):
    assert config.default_floor == '4F'
    assert sorted(config.floors) == ['3F', '4F', '5F']
    assert {f.beacon_floor_code for f in config.floors.values()} == {3, 4, 5}
    assert config.floor_by_beacon_code(5).name == '5F'
    assert config.floor_by_beacon_code(0) is None


def test_only_measured_floor_is_enabled_on_the_real_vehicle(config):
    assert {n for n, f in config.floors.items() if f.real_enabled} == {'4F'}
    assert all(f.provisional_copy_of == '4F' for n, f in config.floors.items() if n != '4F')


def test_referenced_files_exist(config):
    for name, floor in config.floors.items():
        for relative in (floor.map, floor.sim_map, floor.annotations):
            assert (PACKAGE / relative).is_file(), (name, relative)
        assert config.sim_layer_path(name, WORKSPACE).is_file(), name


def _image_hash(yaml_path):
    meta = yaml.safe_load(yaml_path.read_text())
    image = (yaml_path.parent / meta['image']).read_bytes()
    meta.pop('image')
    return hashlib.sha256(image).hexdigest(), meta


def test_provisional_copies_match_their_source(config):
    """While a floor is a provisional copy its maps must equal the source floor."""
    for floor in config.floors.values():
        if floor.provisional_copy_of is None:
            continue
        source = config.floor(floor.provisional_copy_of)
        for attribute in ('map', 'sim_map'):
            assert _image_hash(PACKAGE / getattr(floor, attribute)) == \
                _image_hash(PACKAGE / getattr(source, attribute)), (floor.name, attribute)
        assert (PACKAGE / floor.annotations).read_bytes() == (PACKAGE / source.annotations).read_bytes()
        layer = config.sim_layer_path(floor.name, WORKSPACE).read_text().splitlines()
        source_layer = config.sim_layer_path(source.name, WORKSPACE).read_text().splitlines()
        differing = [a for a, b in zip(layer, source_layer) if a != b]
        assert len(layer) == len(source_layer)
        assert all('sourceMap' in line or 'sourceAnnotations' in line or 'doc =' in line
                   for line in differing), differing


@pytest.mark.parametrize('simulation', [False, True])
def test_stop_points_and_rooms_are_free(config, simulation):
    for name, floor in config.floors.items():
        grid = load_occupancy_map(config.map_path(name, simulation))
        points = {'spawn': floor.spawn}
        for stop in floor.elevators.values():
            points[f'{stop.elevator_id}.hall_wait'] = stop.hall_wait_pose
            points[f'{stop.elevator_id}.hall_exit'] = stop.hall_exit_pose
        points.update({f'room {r.room_id}': r.goal for r in floor.rooms.values()})
        for label, pose in points.items():
            assert grid.disk_is_free(pose[0], pose[1], ROBOT_CLEARANCE_M), (name, label, simulation)


@pytest.mark.parametrize('simulation', [False, True])
def test_car_interior_has_no_mapped_wall(config, simulation):
    for name in config.floors:
        grid = load_occupancy_map(config.map_path(name, simulation))
        polygon = config.elevator_geometry(name, 'E1').car_polygon
        xs, ys = [p[0] for p in polygon], [p[1] for p in polygon]
        step = grid.resolution
        x = min(xs) + 2 * step
        while x < max(xs) - 2 * step:
            y = min(ys) + 2 * step
            while y < max(ys) - 2 * step:
                assert grid.value_at(x, y) != 100, (name, x, y, simulation)
                y += step
            x += step


def test_elevator_geometry_matches_drawing(config):
    geometry = config.elevator_geometry('4F', 'E1')
    # Door centred in a 1.87 m car (0.385 + 1.10 + 0.385), car 0.66 m beyond the hall wall.
    assert geometry.car_pose[0] == pytest.approx(9.975 - 0.66 - 2.23 / 2)
    assert geometry.car_pose[1] == pytest.approx(3.31)
    assert geometry.car_pose[2] == pytest.approx(0.0, abs=1e-5)  # facing the door (east)
    (x0, y0), (x1, y1) = geometry.hall_door
    assert x0 == pytest.approx(9.975) and x1 == pytest.approx(9.975)
    assert abs(y0 - y1) == pytest.approx(1.10)
    assert {round(p[0], 3) for p in geometry.car_door} == {round(9.975 - 0.66, 3)}
    xs = sorted({round(p[0], 3) for p in geometry.car_polygon})
    ys = sorted({round(p[1], 3) for p in geometry.car_polygon})
    assert xs == [round(9.975 - 0.66 - 2.23, 3), round(9.975 - 0.66, 3)]
    assert ys == [round(3.31 - 0.935, 3), round(3.31 + 0.935, 3)]


def test_elevator_door_matches_generated_layer(config):
    """The configured hall door must be the door the scene generator built."""
    for name, floor in config.floors.items():
        text = config.sim_layer_path(name, WORKSPACE).read_text()
        assert 'custom int elevatorDoorCount = 1' in text
        block = text[text.index('def Xform "ElevatorDoor_01"'):]
        closed = re.findall(r'door:closedPosition = \(([^)]*)\)', block)[:2]
        centres = [tuple(float(v) for v in c.split(',')) for c in closed]
        centre = (sum(c[0] for c in centres) / 2, sum(c[1] for c in centres) / 2)
        door = floor.elevators['E1'].door_center
        assert math.dist(centre, door) < 0.05, (name, centre, door)


def test_wait_and_exit_poses_face_the_right_way(config):
    for floor in config.floors.values():
        for stop in floor.elevators.values():
            inward = stop.car_inward_yaw
            ux, uy = math.cos(inward), math.sin(inward)
            for pose, expected_yaw, label in ((stop.hall_wait_pose, inward, 'wait'),
                                              (stop.hall_exit_pose, inward + math.pi, 'exit')):
                depth = (pose[0] - stop.door_center[0]) * ux + (pose[1] - stop.door_center[1]) * uy
                assert depth < -0.5, (floor.name, label, depth)  # on the hall side
                assert math.cos(pose[2] - expected_yaw) > 0.999, (floor.name, label)


def test_room_lookup(config):
    floor, room = config.find_room('302')
    assert (floor.name, room.goal) == ('3F', (33.73, -0.2, -1.5708))
    with pytest.raises(FloorConfigError):
        config.find_room('999')
    with pytest.raises(FloorConfigError):
        config.find_room('302', floor='5F')


def test_floor4_rooms_json_stays_consistent(config):
    """sim_demo.py still reads floor4_rooms.json; it must agree with floors.yaml."""
    rooms = json.loads((WORKSPACE / 'sim/config/floor4_rooms.json').read_text())['rooms']
    configured = config.floor('4F').rooms
    assert sorted(rooms) == sorted(configured)
    for room_id, room in rooms.items():
        assert tuple(room['goal']) == configured[room_id].goal
        assert tuple(room['door']) == configured[room_id].door


def test_paths_resolve_against_package_root(config):
    assert config.map_path('3F', simulation=True) == PACKAGE / 'maps/sim_floor3/floor3.yaml'
    assert config.map_path('4F', simulation=False) == PACKAGE / 'maps/G_4_floor/4_floor_0522.yaml'


def _broken(raw, mutate):
    data = copy.deepcopy(raw)
    mutate(data)
    return data


@pytest.mark.parametrize('mutate, message', [
    (lambda d: d.update(schema_version=2), 'schema_version'),
    (lambda d: d.update(default_floor='9F'), 'default_floor'),
    (lambda d: d['floors']['5F'].update(beacon_floor_code=3), 'duplicate beacon_floor_code'),
    (lambda d: d['floors']['5F'].update(level=4), 'duplicate level'),
    (lambda d: d['floors']['3F'].update(beacon_floor_code=0), 'non-zero int8'),
    (lambda d: d['floors']['3F'].update(beacon_floor_code=200), 'non-zero int8'),
    (lambda d: d['floors']['3F'].update(provisional_copy_of='2F'), 'provisional_copy_of'),
    (lambda d: d['floors']['3F'].update(real_enabled='yes'), 'real_enabled'),
    (lambda d: d['floors']['3F'].update(map=''), 'map'),
    (lambda d: d['floors']['3F']['elevators']['E1'].update(hall_wait_pose=[1, 2]), 'hall_wait_pose'),
    (lambda d: d['floors']['3F']['elevators']['E1'].update(door_center=[1, float('nan')]), 'finite'),
    (lambda d: d['floors']['3F']['elevators'].update(E9=d['floors']['3F']['elevators']['E1']), 'unknown elevator'),
    (lambda d: d['floors']['3F']['rooms']['301'].update(goal=[1, 2, 'x']), 'goal'),
    (lambda d: d['elevators']['E1'].update(door_width_m=1.7), 'does not fit'),
    (lambda d: d['elevators']['E1'].update(door_open_time_s=[10, 6]), 'door_open_time_s'),
    (lambda d: d['elevators']['E1'].update(car_interior_m=[1.87, -1]), 'positive'),
    (lambda d: d['sim'].update(floor_height_m=0), 'floor_height_m'),
    (lambda d: d.update(floors={}), 'no floors'),
])
def test_invalid_configs_are_rejected(raw, mutate, message):
    with pytest.raises(FloorConfigError, match=message):
        parse_floor_config(_broken(raw, mutate))
