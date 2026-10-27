from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from amr_patrol.routes import PatrolConfigError, load_patrol_config
from amr_stack.floor_config import Home, load_floor_config


STACK = Path(__file__).resolve().parents[2] / 'amr_stack'


@pytest.fixture
def raw():
    return dict(schema_version=1, default_dwell_s=3.0, routes={
        '4F_test': dict(floor='4F', end='home', waypoints=[dict(x=1, y=2)])})


def load(tmp_path, raw, building=None):
    path = tmp_path / 'patrol.yaml'
    path.write_text(yaml.safe_dump(raw), encoding='utf-8')
    return load_patrol_config(path, building)


def test_repository_route_and_building():
    building = load_floor_config(STACK / 'config/floors.yaml')
    config = load_patrol_config(STACK / 'config/patrol.yaml', building)
    route = config.routes['4F_full']
    assert route.floor == '4F' and route.end == 'home'
    assert len(route.waypoints) == 12
    assert building.home.room == '401'
    assert config.home_floor == '4F'
    assert config.home_pose == building.floor('4F').rooms['401'].goal


def test_missing_yaw_dwell_and_note_defaults(tmp_path, raw):
    config = load(tmp_path, raw)
    point = config.routes['4F_test'].waypoints[0]
    assert (point.x, point.y, point.yaw, point.dwell_s, point.note) == (1., 2., None, 3., '')
    del raw['default_dwell_s']
    assert load(tmp_path, raw).default_dwell_s == 3.


@pytest.mark.parametrize('field,value', [
    ('x', float('nan')), ('x', float('inf')), ('y', -float('inf')),
    ('x', True), ('x', '1'), ('y', None), ('yaw', float('nan')),
    ('yaw', None), ('yaw', False), ('dwell_s', -1), ('dwell_s', float('inf')),
    ('dwell_s', False), ('note', 1),
])
def test_invalid_waypoint(tmp_path, raw, field, value):
    raw['routes']['4F_test']['waypoints'][0][field] = value
    with pytest.raises(PatrolConfigError):
        load(tmp_path, raw)


@pytest.mark.parametrize('name', ['', 'bad name', '한글', 'a' * 33, 42])
def test_invalid_route_name(tmp_path, raw, name):
    raw['routes'][name] = raw['routes'].pop('4F_test')
    with pytest.raises(PatrolConfigError):
        load(tmp_path, raw)


@pytest.mark.parametrize('field,value', [
    ('floor', ''), ('floor', None), ('floor', 4), ('end', 'elsewhere'),
    ('end', None), ('waypoints', []), ('waypoints', {}), ('waypoints', [None]),
    ('waypoints', [dict(y=1)]),
])
def test_invalid_route(tmp_path, raw, field, value):
    raw['routes']['4F_test'][field] = value
    with pytest.raises(PatrolConfigError):
        load(tmp_path, raw)


@pytest.mark.parametrize('field,value', [
    ('schema_version', 2), ('schema_version', True), ('schema_version', None),
    ('routes', []), ('routes', {}), ('routes', {'okay': None}),
    ('default_dwell_s', -1), ('default_dwell_s', float('nan')), ('default_dwell_s', '3'),
])
def test_invalid_config(tmp_path, raw, field, value):
    raw[field] = value
    with pytest.raises(PatrolConfigError):
        load(tmp_path, raw)


def test_bad_file_and_document(tmp_path):
    path = tmp_path / 'missing.yaml'
    with pytest.raises(PatrolConfigError):
        load_patrol_config(path)
    for text in ('[broken', '', '[]'):
        path.write_text(text)
        with pytest.raises(PatrolConfigError):
            load_patrol_config(path)


def test_building_checks(tmp_path, raw):
    building = load_floor_config(STACK / 'config/floors.yaml')
    assert load(tmp_path, raw, building).home_pose is not None
    with pytest.raises(PatrolConfigError, match='home'):
        load(tmp_path, raw, replace(building, home=None))
    with pytest.raises(PatrolConfigError, match='home'):
        load(tmp_path, raw, replace(building, home=Home('3F', '301')))
    raw['routes']['4F_test']['floor'] = 'unknown'
    with pytest.raises(PatrolConfigError, match='unknown floor'):
        load(tmp_path, raw, building)
    raw['routes']['4F_test'].update(floor='4F', end='stay')
    assert load(tmp_path, raw, replace(building, home=None)).home_pose is None


def test_explicit_zero_and_heading(tmp_path, raw):
    raw['routes']['4F_test']['waypoints'][0].update(dwell_s=0, yaw=1.5)
    point = load(tmp_path, raw).routes['4F_test'].waypoints[0]
    assert point.dwell_s == 0 and point.yaw == 1.5


@pytest.mark.parametrize('key', ['dwell', 'heading', 'z', 42])
def test_unknown_waypoint_keys_list_allowed_keys(tmp_path, raw, key):
    raw['routes']['4F_test']['waypoints'][0][key] = 3
    with pytest.raises(PatrolConfigError, match='allowed keys: x y yaw dwell_s note') as error:
        load(tmp_path, raw)
    assert str(key) in str(error.value)
