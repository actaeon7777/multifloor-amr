"""ROS-independent route configuration and same-floor home resolution."""
from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Mapping, Optional, Tuple

import yaml


class PatrolConfigError(ValueError):
    """Invalid patrol configuration."""


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: Optional[float]
    dwell_s: float
    note: str


@dataclass(frozen=True)
class Route:
    name: str
    floor: str
    end: str
    waypoints: Tuple[Waypoint, ...]


@dataclass(frozen=True)
class PatrolConfig:
    routes: Mapping[str, Route]
    default_dwell_s: float = 3.0
    home_pose: Optional[Tuple[float, float, float]] = None
    home_floor: Optional[str] = None


def default_config_path():
    from amr_stack.floor_config import default_config_path as floors_path
    return floors_path().with_name('patrol.yaml')


def _mapping(value, label):
    if not isinstance(value, dict):
        raise PatrolConfigError(f'{label}: expected a mapping')
    return value


def _number(value, label, nonnegative=False):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or (nonnegative and value < 0)):
        raise PatrolConfigError(f'{label}: expected a finite '
                                + ('non-negative number' if nonnegative else 'number'))
    return float(value)


def load_patrol_config(path=None, building=None) -> PatrolConfig:
    path = Path(path) if path is not None else default_config_path()
    try:
        raw = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise PatrolConfigError(f'{path}: {error}') from error
    data = _mapping(raw, str(path))
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise PatrolConfigError('schema_version must be 1')
    default_dwell = _number(data.get('default_dwell_s', 3.0), 'default_dwell_s', True)
    routes = {}
    for name, entry in _mapping(data.get('routes'), 'routes').items():
        if not isinstance(name, str) or re.fullmatch(r'[A-Za-z0-9_-]{1,32}', name) is None:
            raise PatrolConfigError('route name must match [A-Za-z0-9_-]{1,32}')
        entry = _mapping(entry, name)
        floor = entry.get('floor')
        if not isinstance(floor, str) or not floor.strip():
            raise PatrolConfigError(f'{name}: floor must be a non-empty string')
        end = entry.get('end', 'home')
        if end not in ('home', 'stay'):
            raise PatrolConfigError(f'{name}: end must be home or stay')
        if building is not None:
            if floor not in building.floors:
                raise PatrolConfigError(f'{name}: unknown floor {floor}')
            if end == 'home' and (building.home is None or building.home.floor != floor):
                raise PatrolConfigError(f'{name}: end home requires home on the route floor')
        points = entry.get('waypoints')
        if not isinstance(points, list) or not points:
            raise PatrolConfigError(f'{name}: at least one waypoint is required')
        waypoints = []
        for index, point in enumerate(points, 1):
            label = f'{name}.waypoints[{index}]'
            point = _mapping(point, label)
            allowed = {'x', 'y', 'yaw', 'dwell_s', 'note'}
            unknown = set(point) - allowed
            if unknown:
                raise PatrolConfigError(f'{label}: unknown keys {sorted(map(str, unknown))}; '
                                        'allowed keys: x y yaw dwell_s note')
            note = point.get('note', '')
            if not isinstance(note, str):
                raise PatrolConfigError(f'{label}.note must be a string')
            waypoints.append(Waypoint(
                _number(point.get('x'), label + '.x'),
                _number(point.get('y'), label + '.y'),
                _number(point['yaw'], label + '.yaw') if 'yaw' in point else None,
                _number(point.get('dwell_s', default_dwell), label + '.dwell_s', True), note))
        routes[name] = Route(name, floor, end, tuple(waypoints))
    if not routes:
        raise PatrolConfigError('at least one route is required')
    home_pose, home_floor = None, None
    if building is not None and building.home is not None:
        home_floor = building.home.floor
        try:
            home_pose = tuple(building.floor(home_floor).rooms[building.home.room].goal)
        except (KeyError, ValueError) as error:
            raise PatrolConfigError('home must reference an existing room') from error
    return PatrolConfig(routes, default_dwell, home_pose, home_floor)
