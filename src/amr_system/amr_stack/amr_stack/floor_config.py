"""Building, floor, map and elevator configuration for multi-floor missions.

``config/floors.yaml`` is the single source for floor name → map → simulation
layer → elevator stop points.  Replacing a provisional floor map must only
require editing that file, so every consumer (ROS nodes, simulation tools and
the web UI) reads it through this ROS-independent module.

Path conventions:
* ``map``, ``annotations`` and ``sim_map`` are relative to the amr_stack package
  root, which has the same layout in ``src/`` and in the installed share.
* ``sim_layer`` is relative to the AMR workspace root (simulation only).
"""

from dataclasses import dataclass, field
import math
import os
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple

import numpy as np
import yaml


SCHEMA_VERSION = 1
CONFIG_NAME = 'floors.yaml'
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]

Point = Tuple[float, float]
Pose = Tuple[float, float, float]


class FloorConfigError(ValueError):
    """The floor configuration is malformed or inconsistent."""


@dataclass(frozen=True)
class ElevatorSpec:
    elevator_id: str
    car_width_m: float
    car_depth_m: float
    door_width_m: float
    door_offset_from_left_m: float
    threshold_depth_m: float
    door_open_time_s: Tuple[float, float]
    hall_recess_m: float = 0.  # hall wall face -> door_center plane (the door recess)


@dataclass(frozen=True)
class FloorElevator:
    elevator_id: str
    door_center: Point
    car_inward_yaw: float
    hall_wait_pose: Pose
    hall_exit_pose: Pose


@dataclass(frozen=True)
class Room:
    room_id: str
    goal: Pose
    door: Optional[Point]


@dataclass(frozen=True)
class Floor:
    name: str
    level: int
    beacon_floor_code: int
    real_enabled: bool
    provisional_copy_of: Optional[str]
    map: str
    annotations: Optional[str]
    sim_map: str
    sim_layer: str
    spawn: Pose
    elevators: Mapping[str, FloorElevator]
    rooms: Mapping[str, Room]


@dataclass(frozen=True)
class PassFloor:
    """A floor the car stops at but the AMR has no map for (``pass_floors``).

    Its hall beacon only tells the floor state that the car is at another floor,
    so the ride continues; it can never be a mission goal.
    """
    name: str
    level: int
    beacon_floor_code: int


@dataclass(frozen=True)
class SimSettings:
    floor_height_m: float
    car_speed_m_s: float
    car_accel_m_s2: float


@dataclass(frozen=True)
class ElevatorGeometry:
    """Elevator shapes on one floor's map frame, derived from the config.

    ``hall_door`` is the hall door plane (``door_center``, the wall on the map)
    the robot crosses when boarding.  ``car_door`` is the car's inner front wall.
    ``car_polygon`` is the car interior, counter-clockwise.  ``car_pose`` is the
    car centre facing out.  ``hall_entry`` is the hall wall face, ``hall_recess_m``
    before ``hall_door``: the door tunnel with its side walls starts there.
    """
    hall_door: Tuple[Point, Point]
    car_door: Tuple[Point, Point]
    car_polygon: Tuple[Point, Point, Point, Point]
    car_pose: Pose
    hall_entry: Optional[Tuple[Point, Point]] = None


@dataclass(frozen=True)
class Home:
    floor: str
    room: str


@dataclass(frozen=True)
class BuildingConfig:
    schema_version: int
    building: str
    default_floor: str
    elevators: Mapping[str, ElevatorSpec]
    floors: Mapping[str, Floor]
    sim: SimSettings
    source: Optional[Path] = None
    package_root: Path = _PACKAGE_ROOT
    pass_floors: Mapping[str, PassFloor] = field(default_factory=dict)

    home: Optional[Home] = None

    def floor(self, name: str) -> Floor:
        try:
            return self.floors[name]
        except KeyError:
            raise FloorConfigError(f'unknown floor {name!r}') from None

    def floor_by_beacon_code(self, code: int) -> Optional[Floor]:
        return next((f for f in self.floors.values() if f.beacon_floor_code == code), None)

    def pass_floor_by_beacon_code(self, code: int) -> Optional[PassFloor]:
        return next((f for f in self.pass_floors.values() if f.beacon_floor_code == code), None)

    def find_room(self, room_id: str, floor: Optional[str] = None) -> Tuple[Floor, Room]:
        """Return the floor and room; ``floor`` is required only if ambiguous."""
        candidates = [f for f in self.floors.values() if room_id in f.rooms
                      and (floor is None or f.name == floor)]
        if not candidates:
            raise FloorConfigError(f'unknown room {room_id!r}')
        if len(candidates) > 1:
            raise FloorConfigError(f'room {room_id!r} exists on several floors; give the floor')
        return candidates[0], candidates[0].rooms[room_id]

    def elevator_geometry(self, floor: str, elevator_id: str) -> ElevatorGeometry:
        stop = self.floor(floor).elevators.get(elevator_id)
        if stop is None:
            raise FloorConfigError(f'floor {floor!r} has no elevator {elevator_id!r}')
        return elevator_geometry(self.elevators[elevator_id], stop)

    def map_path(self, floor: str, simulation: bool) -> Path:
        item = self.floor(floor)
        return self.package_root / (item.sim_map if simulation else item.map)

    def sim_layer_path(self, floor: str, workspace_root: Optional[Path] = None) -> Path:
        return (workspace_root or find_workspace_root()) / self.floor(floor).sim_layer


def elevator_geometry(spec: ElevatorSpec, stop: FloorElevator) -> ElevatorGeometry:
    ux, uy = math.cos(stop.car_inward_yaw), math.sin(stop.car_inward_yaw)
    lx, ly = -uy, ux  # left of a robot facing into the car
    cx, cy = stop.door_center
    half_door = spec.door_width_m / 2
    left = half_door + spec.door_offset_from_left_m
    right = left - spec.car_width_m
    near, far = spec.threshold_depth_m, spec.threshold_depth_m + spec.car_depth_m

    def at(depth, lateral):
        return (cx + ux * depth + lx * lateral, cy + uy * depth + ly * lateral)

    centre = at((near + far) / 2, (left + right) / 2)
    outward = math.atan2(-uy, -ux)
    return ElevatorGeometry(
        hall_door=(at(0., half_door), at(0., -half_door)),
        car_door=(at(near, half_door), at(near, -half_door)),
        car_polygon=(at(near, right), at(far, right), at(far, left), at(near, left)),
        car_pose=(centre[0], centre[1], outward),
        hall_entry=(at(-spec.hall_recess_m, half_door), at(-spec.hall_recess_m, -half_door)),
    )


def default_config_path() -> Path:
    """Installed share config when available, otherwise the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('amr_stack')) / 'config' / CONFIG_NAME
        if share.is_file():
            return share
    except Exception:  # noqa: BLE001 - ament index is absent outside ROS
        pass
    return _PACKAGE_ROOT / 'config' / CONFIG_NAME


def find_workspace_root(start: Optional[Path] = None) -> Path:
    """``$AMR_WS`` or the nearest parent holding both ``src/`` and ``sim/``."""
    if start is None and os.environ.get('AMR_WS'):
        return Path(os.environ['AMR_WS'])
    for candidate in (start or _PACKAGE_ROOT).resolve().parents:
        if (candidate / 'src').is_dir() and (candidate / 'sim').is_dir():
            return candidate
    raise FloorConfigError('AMR workspace root not found; set AMR_WS')


def load_floor_config(path: Optional[Path] = None, package_root: Optional[Path] = None) -> BuildingConfig:
    path = Path(path) if path else default_config_path()
    try:
        raw = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise FloorConfigError(f'{path}: {error}') from error
    root = Path(package_root) if package_root else path.resolve().parent.parent
    return parse_floor_config(raw, source=path, package_root=root)


def parse_floor_config(raw, source: Optional[Path] = None,
                       package_root: Path = _PACKAGE_ROOT) -> BuildingConfig:
    where = str(source or 'floors')
    data = _mapping(raw, where)
    if data.get('schema_version') != SCHEMA_VERSION:
        raise FloorConfigError(f'{where}: schema_version must be {SCHEMA_VERSION}')
    elevators = {str(k): _elevator_spec(str(k), _mapping(v, f'{where}.elevators.{k}'))
                 for k, v in _mapping(data.get('elevators'), f'{where}.elevators').items()}
    floors = {str(k): _floor(str(k), _mapping(v, f'{where}.floors.{k}'), elevators)
              for k, v in _mapping(data.get('floors'), f'{where}.floors').items()}
    if not floors:
        raise FloorConfigError(f'{where}: no floors')
    default_floor = str(data.get('default_floor'))
    if default_floor not in floors:
        raise FloorConfigError(f'{where}: default_floor {default_floor!r} is not a floor')
    pass_floors = {str(k): PassFloor(str(k), *_level_and_code(
                       f'pass floor {k}', _mapping(v, f'{where}.pass_floors.{k}')))
                   for k, v in _mapping(data.get('pass_floors', {}), f'{where}.pass_floors').items()}
    for name in pass_floors:
        if name in floors:
            raise FloorConfigError(f'{where}: pass floor {name!r} is also a mapped floor')
    for key in ('level', 'beacon_floor_code'):
        values = [getattr(f, key) for f in (*floors.values(), *pass_floors.values())]
        if len(set(values)) != len(values):
            raise FloorConfigError(f'{where}: duplicate {key}')
    for item in floors.values():
        if item.provisional_copy_of is not None and item.provisional_copy_of not in floors:
            raise FloorConfigError(f'floor {item.name}: provisional_copy_of is not a floor')
    sim = _mapping(data.get('sim', {}), f'{where}.sim')
    settings = SimSettings(
        floor_height_m=_positive(sim.get('floor_height_m', 3.6), 'sim.floor_height_m'),
        car_speed_m_s=_positive(sim.get('car_speed_m_s', 1.0), 'sim.car_speed_m_s'),
        car_accel_m_s2=_positive(sim.get('car_accel_m_s2', 0.5), 'sim.car_accel_m_s2'),
    )
    home = None
    if 'home' in data:
        entry = _mapping(data['home'], f'{where}.home')
        home = Home(str(entry.get('floor', '')), str(entry.get('room', '')))
        if home.floor not in floors or home.room not in floors[home.floor].rooms:
            raise FloorConfigError(f'{where}: home must reference an existing floor and room')
    return BuildingConfig(SCHEMA_VERSION, str(data.get('building', '')), default_floor,
                          elevators, floors, settings, source, Path(package_root), pass_floors, home)


def _elevator_spec(name, data) -> ElevatorSpec:
    width, depth = _numbers(data.get('car_interior_m'), 2, f'elevator {name}.car_interior_m')
    door = _positive(data.get('door_width_m'), f'elevator {name}.door_width_m')
    offset = _number(data.get('door_offset_from_left_m'), f'elevator {name}.door_offset_from_left_m')
    threshold = _number(data.get('threshold_depth_m'), f'elevator {name}.threshold_depth_m')
    recess = _number(data.get('hall_recess_m', 0.), f'elevator {name}.hall_recess_m')
    open_min, open_max = _numbers(data.get('door_open_time_s'), 2, f'elevator {name}.door_open_time_s')
    if width <= 0 or depth <= 0:
        raise FloorConfigError(f'elevator {name}: car dimensions must be positive')
    if offset < 0 or offset + door > width + 1e-9:
        raise FloorConfigError(f'elevator {name}: door does not fit the car width')
    if threshold < 0:
        raise FloorConfigError(f'elevator {name}: threshold_depth_m must not be negative')
    if recess < 0:
        raise FloorConfigError(f'elevator {name}: hall_recess_m must not be negative')
    if not 0 < open_min <= open_max:
        raise FloorConfigError(f'elevator {name}: door_open_time_s must be 0 < min <= max')
    return ElevatorSpec(name, width, depth, door, offset, threshold, (open_min, open_max), recess)


def _level_and_code(label, data):
    code = data.get('beacon_floor_code')
    if not isinstance(code, int) or isinstance(code, bool) or not -128 <= code <= 127 or code == 0:
        raise FloorConfigError(f'{label}: beacon_floor_code must be a non-zero int8 (0 is the car)')
    if not -8 <= code <= 20:
        # Hall beacon RF node 0x40 + code must stay in 0x38-0x54 (ELEVATOR_RF_PROTOCOL.md 4).
        raise FloorConfigError(f'{label}: beacon_floor_code must be in -8..20')
    level = data.get('level')
    if not isinstance(level, int) or isinstance(level, bool):
        raise FloorConfigError(f'{label}: level must be an integer')
    return level, code


def _floor(name, data, elevators) -> Floor:
    label = f'floor {name}'
    level, code = _level_and_code(label, data)
    real_enabled = data.get('real_enabled', False)
    if not isinstance(real_enabled, bool):
        raise FloorConfigError(f'{label}: real_enabled must be true or false')
    stops: Dict[str, FloorElevator] = {}
    for key, value in _mapping(data.get('elevators', {}), f'{label}.elevators').items():
        key = str(key)
        if key not in elevators:
            raise FloorConfigError(f'{label}: unknown elevator {key!r}')
        value = _mapping(value, f'{label}.elevators.{key}')
        stops[key] = FloorElevator(
            key,
            _numbers(value.get('door_center'), 2, f'{label}.{key}.door_center'),
            _number(value.get('car_inward_yaw'), f'{label}.{key}.car_inward_yaw'),
            _numbers(value.get('hall_wait_pose'), 3, f'{label}.{key}.hall_wait_pose'),
            _numbers(value.get('hall_exit_pose'), 3, f'{label}.{key}.hall_exit_pose'),
        )
    rooms: Dict[str, Room] = {}
    for key, value in _mapping(data.get('rooms', {}), f'{label}.rooms').items():
        value = _mapping(value, f'{label}.rooms.{key}')
        door = value.get('door')
        rooms[str(key)] = Room(
            str(key), _numbers(value.get('goal'), 3, f'{label}.rooms.{key}.goal'),
            None if door is None else _numbers(door, 2, f'{label}.rooms.{key}.door'))
    copy_of = data.get('provisional_copy_of')
    return Floor(
        name=name, level=level, beacon_floor_code=code, real_enabled=real_enabled,
        provisional_copy_of=None if copy_of is None else str(copy_of),
        map=_text(data.get('map'), f'{label}.map'),
        annotations=None if data.get('annotations') is None else _text(data['annotations'], f'{label}.annotations'),
        sim_map=_text(data.get('sim_map'), f'{label}.sim_map'),
        sim_layer=_text(data.get('sim_layer'), f'{label}.sim_layer'),
        spawn=_numbers(data.get('spawn'), 3, f'{label}.spawn'),
        elevators=stops, rooms=rooms,
    )


def _mapping(value, where):
    if not isinstance(value, dict):
        raise FloorConfigError(f'{where}: expected a mapping')
    return value


def _text(value, where):
    if not isinstance(value, str) or not value.strip():
        raise FloorConfigError(f'{where}: expected a non-empty path')
    return value


def _number(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise FloorConfigError(f'{where}: expected a finite number')
    return float(value)


def _positive(value, where):
    value = _number(value, where)
    if value <= 0:
        raise FloorConfigError(f'{where}: must be positive')
    return value


def _numbers(value, count, where):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise FloorConfigError(f'{where}: expected {count} numbers')
    return tuple(_number(v, where) for v in value)


# Map checks ---------------------------------------------------------------

@dataclass(frozen=True)
class OccupancyMap:
    """A ROS trinary map; ``cells`` row 0 is the bottom (map y = origin y)."""
    cells: np.ndarray  # int8: 0 free, 100 occupied, -1 unknown
    resolution: float
    origin: Pose

    def value_at(self, x: float, y: float) -> int:
        col = math.floor((x - self.origin[0]) / self.resolution)
        row = math.floor((y - self.origin[1]) / self.resolution)
        if not (0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]):
            return -1
        return int(self.cells[row, col])

    def disk_is_free(self, x: float, y: float, radius: float) -> bool:
        steps = int(math.ceil(radius / self.resolution))
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                dx, dy = i * self.resolution, j * self.resolution
                if dx * dx + dy * dy <= radius * radius and self.value_at(x + dx, y + dy) != 0:
                    return False
        return True


def load_occupancy_map(yaml_path: Path) -> OccupancyMap:
    """Read a ROS map_server YAML + binary PGM (P5) without ROS."""
    meta = yaml.safe_load(Path(yaml_path).read_text(encoding='utf-8'))
    if str(meta.get('mode', 'trinary')).lower() != 'trinary':
        raise FloorConfigError(f'{yaml_path}: only trinary maps are supported')
    image = Path(yaml_path).parent / meta['image']
    pixels = _read_pgm(image).astype(float)
    if int(meta.get('negate', 0)):
        pixels = 255 - pixels
    occupancy = (255 - pixels) / 255.
    cells = np.full(pixels.shape, -1, dtype=np.int8)
    cells[occupancy > float(meta['occupied_thresh'])] = 100
    cells[occupancy < float(meta['free_thresh'])] = 0
    origin = tuple(float(v) for v in meta['origin'])
    return OccupancyMap(np.flipud(cells), float(meta['resolution']), origin)


def _read_pgm(path: Path) -> np.ndarray:
    data = Path(path).read_bytes()
    tokens, index = [], 0
    while len(tokens) < 4:
        while data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b'#':
            index = data.index(b'\n', index) + 1
            continue
        start = index
        while not data[index:index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        raise FloorConfigError(f'{path}: only 8-bit binary PGM is supported')
    width, height = int(tokens[1]), int(tokens[2])
    raster = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=index + 1)
    return raster.reshape(height, width)
