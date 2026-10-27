"""Pure, deliberately small projection of the local robot state."""
from copy import deepcopy


def _dict(value):
    return value if isinstance(value, dict) else {}


def _subset(value, keys):
    if not isinstance(value, dict):
        return None
    return {key: deepcopy(value.get(key)) for key in keys.split()}


def summarize(state: dict) -> dict:
    state = _dict(state)
    floor = _dict(state.get('floor'))
    # During a ride the live position still belongs to the departure floor.
    current = floor.get('floor') or floor.get('transit_from')
    localization = _dict(state.get('localization'))
    return dict(
        floor=current if isinstance(current, str) else None,
        zone=deepcopy(floor.get('zone')),
        robot=_subset(state.get('robot'), 'x y yaw age'),
        task=state.get('task') if isinstance(state.get('task'), str) else '',
        mode=deepcopy(state.get('mode')),
        navigation=deepcopy(state.get('navigation')),
        estop=_subset(_dict(state.get('estop')), 'state healthy detail error'),
        # Fixed key sets: the server refuses an exchange whose summary carries unknown keys.
        hold=_subset(state.get('hold'), 'held since reason pending') or {'held': False},
        goal=_subset(state.get('goal'), 'request_id kind status floor room origin error_message phase'),
        floor_mission=_subset(state.get('floor_mission'), 'phase detail'),
        battery=_subset(_dict(state.get('telemetry')).get('battery'), 'soc voltage valid fresh charge_state'),
        localization=dict(ready=localization.get('ready', False), age=localization.get('age')),
        notice=_subset(state.get('notice'), 'request_id status detail'),
        pending_request=_subset(state.get('pending_request'), 'request_id kind'),
    )
