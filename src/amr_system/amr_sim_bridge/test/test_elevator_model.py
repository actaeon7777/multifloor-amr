"""Simulated elevator devices: only beacon/ack frames out, car and doors move plausibly."""

import pytest

from amr_sim_bridge.elevator_model import (
    CMD_CAR_FLOOR, CMD_DOOR_OPEN, CMD_HALL_CALL, DIR_DOWN, DIR_NONE, DIR_UP, EVENT_BUTTON_ACK,
    EVENT_PRESENCE, LOCATION_CAR, LOCATION_HALL, ElevatorSim,
)

FLOORS = [('3F', 3), ('4F', 4), ('5F', 5)]


def sim(**kwargs):
    kwargs.setdefault('start_floor', '3F')
    model = ElevatorSim(FLOORS, **kwargs)
    model.set_robot(1., False)  # robot standing in the 4F hall
    return model


def run(model, until, start=None, dt=.05):
    frames, t = [], model.now if start is None else start
    while t < until - 1e-9:
        t = round(t + dt, 6)
        frames.extend(model.step(t))
    return frames


def acks(frames):
    return [f for f in frames if f.event == EVENT_BUTTON_ACK]


def presence(frames, code=None):
    return [f for f in frames if f.event == EVENT_PRESENCE and (code is None or f.floor_code == code)]


def test_hall_call_brings_the_car_and_door_cycles():
    model = sim(seed=3)
    model.step(0.)
    model.command(0., CMD_HALL_CALL, 4, DIR_DOWN, 17)
    frames = run(model, .3)
    (ack,) = acks(frames)
    assert (ack.location, ack.floor_code, ack.ack_sequence) == (LOCATION_HALL, 4, 17)
    assert model.status()['moving']
    run(model, 6.)  # 3.6 m at 1 m/s, 0.5 m/s^2: 5.6 s
    status = model.status()
    assert status['car_floor'] == '4F' and not status['moving'] and status['door'] in ('OPENING', 'OPEN')
    assert status['hall_calls'] == []
    run(model, 6. + 1.5 + 5.9)  # opened, minimum hold not yet over
    assert model.status()['door'] == 'OPEN'
    run(model, 6. + 1.5 + 10. + 1.6)
    assert model.status()['door'] == 'CLOSED'


@pytest.mark.parametrize('seed', range(20))
def test_door_hold_time_stays_in_the_configured_range(seed):
    model = sim(seed=seed, start_floor='4F')
    model.step(0.)
    model.command(0., CMD_HALL_CALL, 4, DIR_UP, 1)
    run(model, .2)
    hold = model.status()['hold_remaining_s'] - 1.5
    assert 6. - .06 <= hold <= 10.


def test_hall_beacons_announce_every_second_with_leakage():
    model = sim(seed=1, leak_probability=.5)
    frames = run(model, 20., start=0.)
    own = presence(frames, 4)
    assert len(own) == 20 and all(-53 <= f.rssi_dbm <= -47 for f in own)
    assert [f.sequence for f in own] == list(range(20))
    leaked = presence(frames, 3) + presence(frames, 5)
    assert 0 < len(leaked) < 40 and all(f.rssi_dbm <= -79 for f in leaked)
    assert all(f.location == LOCATION_HALL for f in frames)


def test_inside_the_car_only_the_open_floor_is_heard_clearly():
    model = sim(seed=2, leak_probability=0., start_floor='4F')
    model.set_robot(1., True)
    model.step(0.)
    model.command(0., CMD_CAR_FLOOR, 5, DIR_NONE, 3)
    frames = run(model, .3)
    (ack,) = acks(frames)
    assert (ack.location, ack.floor_code) == (LOCATION_CAR, 0)
    travel = run(model, 6.)
    assert presence(travel) == []  # doors closed / moving: metal car
    model.set_robot(2., True)
    arrived = run(model, 9.)
    assert model.status()['car_floor'] == '5F'
    heard = presence(arrived)
    assert heard and {f.floor_code for f in heard} == {5} and all(f.rssi_dbm <= -55 for f in heard)


def test_door_open_button_and_obstruction_keep_the_door_open():
    model = sim(start_floor='4F', faults={'hold_time_s': 6.})
    model.step(0.)
    model.command(0., CMD_HALL_CALL, 4, DIR_UP, 1)
    run(model, 7.)
    model.command(7., CMD_DOOR_OPEN, 0, DIR_NONE, 2)
    run(model, 12.)
    assert model.status()['door'] == 'OPEN'  # 7.2 + 6 s hold
    run(model, 13.8)
    assert model.status()['door'] == 'CLOSING'
    model.obstruct(True)
    run(model, 14.5)
    assert model.status()['door'] in ('OPENING', 'OPEN')
    model.obstruct(False)
    run(model, 30.)
    assert model.status()['door'] == 'CLOSED'


def test_unknown_floor_and_malformed_commands_do_nothing():
    model = sim()
    model.step(0.)
    model.command(0., CMD_HALL_CALL, 9, DIR_UP, 1)
    model.command(0., CMD_HALL_CALL, 4, DIR_NONE, 2)
    model.command(0., CMD_CAR_FLOOR, 9, DIR_NONE, 3)
    model.command(0., 77, 4, DIR_NONE, 4)
    assert acks(run(model, 2.)) == [] and not model.status()['moving']


def test_faults_drop_delay_duplicate_and_lag():
    model = sim(start_floor='4F', faults={'drop_ack_probability': 1.})
    model.step(0.)
    model.command(0., CMD_HALL_CALL, 4, DIR_UP, 1)
    assert acks(run(model, 1.)) == []
    assert model.status()['door'] in ('OPENING', 'OPEN')  # pressed, only the report was lost

    model = sim(start_floor='4F', faults={'ack_extra_delay_s': 3., 'duplicate_probability': 1.,
                                          'stamp_lag_s': 2.})
    model.step(0.)
    model.command(0., CMD_HALL_CALL, 4, DIR_UP, 9)
    assert acks(run(model, 3.)) == []
    late = acks(run(model, 3.5))
    assert len(late) == 2 and late[0] is late[1]
    assert late[0].stamp == pytest.approx(3.2 - 2., abs=.06)


def test_reordered_frames_are_delivered_later():
    model = sim(seed=0, faults={'reorder_probability': 1.})
    frames = run(model, 5., start=0.)
    sequences = [f.sequence for f in presence(frames, 4)]
    assert sorted(sequences) == list(range(len(sequences))) and sequences != sorted(sequences)


def test_unexpected_stop_door_failure_and_wrong_floor():
    model = sim(start_floor='3F', faults={'stop_at_floor_code': 4})
    model.set_robot(0., True)
    model.step(0.)
    model.command(0., CMD_CAR_FLOOR, 5, DIR_NONE, 1)
    run(model, 6.5)
    assert model.status()['car_floor'] == '4F'      # someone else's stop
    assert model.status()['car_calls'] == ['5F']    # our call is kept
    run(model, 40.)
    assert model.status()['car_floor'] == '5F'

    model = sim(start_floor='3F', faults={'door_never_opens': True})
    model.step(0.)
    model.command(0., CMD_HALL_CALL, 4, DIR_DOWN, 1)
    run(model, 20.)
    assert model.status()['car_floor'] == '4F' and model.status()['door'] == 'CLOSED'

    model = sim(faults={'button_fails': True})
    model.step(0.)
    model.command(0., CMD_HALL_CALL, 4, DIR_DOWN, 1)
    assert acks(run(model, 10.)) == [] and model.status()['car_floor'] == '3F'

    model = sim(faults={'wrong_floor_code': 5}, leak_probability=0.)
    assert {f.floor_code for f in presence(run(model, 3., start=0.))} == {4, 5}


@pytest.mark.parametrize('kwargs', [
    dict(floors=[]), dict(floors=[('3F', 3), ('3F', 4)]), dict(floors=[('3F', 0)]),
    dict(floors=FLOORS, floor_height_m=0.), dict(floors=FLOORS, open_time_s=(10., 6.)),
])
def test_invalid_setup_is_rejected(kwargs):
    with pytest.raises(ValueError):
        ElevatorSim(**kwargs)
    model = ElevatorSim(FLOORS)
    with pytest.raises(ValueError):
        model.set_faults(no_such_fault=1)


def test_drop_next_acks_is_deterministic():
    model = sim(start_floor='4F', faults={'drop_next_acks': 2})
    model.step(0.)
    frames = []
    for sequence in (1, 2, 3):
        model.command(model.now, CMD_HALL_CALL, 4, DIR_UP, sequence)
        frames += run(model, model.now + .5)
    assert [f.ack_sequence for f in acks(frames)] == [3]
    assert 'drop_next_acks' not in model.status()['faults']
