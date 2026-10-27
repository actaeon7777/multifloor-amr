"""Pure summary projection, including absent/malformed local blocks."""
from copy import deepcopy
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
from amr_remote_bridge.summary import summarize


def test_missing_fields():
    result = summarize({})
    assert result == dict(floor=None, zone=None, robot=None, task='', mode=None, navigation=None,
                          estop=dict(state=None, healthy=None, detail=None, error=None),
                          hold={'held': False}, goal=None, floor_mission=None, battery=None,
                          localization={'ready': False, 'age': None}, notice=None, pending_request=None)


@pytest.mark.parametrize('state', [None, [], {'floor': None, 'telemetry': [], 'robot': 42},
                                  {'localization': 'bad', 'hold': False, 'task': None}])
def test_missing_or_wrong_blocks_never_raise(state):
    result = summarize(state)
    assert result['floor'] is None and result['task'] == ''
    assert result['hold'] == {'held': False}


def test_projection_is_independent_and_drops_large_fields():
    state = dict(floor={'floor': None, 'transit_from': '4F', 'zone': 1},
                 robot=dict(x=1, y=2, yaw=3, age=.1, covariance=list(range(100))),
                 task='401호 배송 중', mode='AUTO', navigation='ACTIVE',
                 hold=dict(held=True, since=123.4, reason='operator'),
                 estop=dict(state='READY', healthy=True, detail='ok', error=None, age=.1),
                 goal=dict(request_id='delivery_01', kind='goal', status='ACCEPTED', floor='4F',
                           room='401', origin='return_home', error_message=None, phase='NAV', x=10),
                 floor_mission=dict(phase='RIDING', detail='이동', history=[1, 2]),
                 telemetry=dict(battery=dict(soc=80, voltage=24, valid=True, fresh=True, charge_state=1,
                                             current=3)),
                 localization=dict(ready=True, age=.2, covariance=[1, 2]),
                 notice=dict(status='OK'), pending_request=dict(request_id='request_01', kind='stop'),
                 layers={'map': {'version': 1}}, scan={'points': [[1, 2]]}, camera={'age': 0})
    original = deepcopy(state)
    result = summarize(state)
    assert state == original
    assert result['floor'] == '4F' and result['zone'] == 1
    assert result['robot'] == dict(x=1, y=2, yaw=3, age=.1)
    assert result['battery'] == dict(soc=80, voltage=24, valid=True, fresh=True, charge_state=1)
    assert result['floor_mission'] == dict(phase='RIDING', detail='이동')
    assert result['localization'] == dict(ready=True, age=.2)
    assert result['goal']['origin'] == 'return_home' and 'x' not in result['goal']
    assert not {'layers', 'scan', 'camera', 'telemetry'} & result.keys()
    result['hold']['held'] = False
    result['notice']['status'] = 'changed'
    assert state == original


def test_current_floor_precedes_transit_from():
    assert summarize({'floor': {'floor': '3F', 'transit_from': '4F'}})['floor'] == '3F'
