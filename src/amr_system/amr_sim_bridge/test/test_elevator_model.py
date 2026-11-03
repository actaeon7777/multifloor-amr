"""Delayed human requests and car/door dynamics without ROS."""
import pytest
from amr_sim_bridge.elevator_model import (
    ElevatorSim, KIND_CALL, KIND_CAR_FLOOR, KIND_CANCEL, DIR_DOWN, DIR_NONE, DIR_UP,
)

FLOORS = ['3F', '4F', '5F']


def sim(**kwargs):
    return ElevatorSim(FLOORS, **kwargs)


def run(model, until, dt=.05):
    while model.now < until - 1e-9:
        model.step(round(model.now + dt, 6))


def test_press_delay_then_hall_call_arrives_and_opens():
    model = sim(press_delay_s=2.)
    model.request(0., KIND_CALL, '4F', '3F', DIR_DOWN)
    run(model, 1.95)
    assert not model.status()['moving'] and model.status()['hall_calls'] == []
    run(model, 2.)
    assert model.status()['moving'] and model.status()['hall_calls'] == ['4F']
    assert model.hall_directions[1] == DIR_DOWN
    run(model, 9.2)
    assert model.status()['car_floor'] == '4F' and model.status()['door'] == 'OPEN'
    assert model.status()['hall_calls'] == []
    run(model, 25.)
    assert model.status()['door'] == 'CLOSED'


def test_car_call_travels_to_named_target():
    model = sim(start_floor='4F')
    model.request(0., KIND_CAR_FLOOR, '', '5F', DIR_NONE)
    run(model, .95)
    assert model.status()['car_floor'] == '4F'
    run(model, 1.)
    assert model.status()['moving']
    run(model, 8.2)
    assert model.status()['car_floor'] == '5F' and model.status()['door'] == 'OPEN'


def test_cancel_does_not_erase_registered_or_pending_calls():
    model = sim()
    model.request(0., KIND_CAR_FLOOR, '', '5F', DIR_NONE)
    model.request(.5, KIND_CANCEL, '', '', DIR_NONE)
    run(model, 1.)
    model.request(1., KIND_CANCEL, '', '', DIR_NONE)
    assert model.status()['car_calls'] == ['5F']
    run(model, 12.)
    assert model.status()['car_floor'] == '5F'


@pytest.mark.parametrize('seed', range(20))
def test_configured_door_hold_and_obstruction(seed):
    model = sim(seed=seed, start_floor='4F')
    model.request(0., KIND_CALL, '4F', '5F', DIR_UP)
    run(model, 1.)
    assert 6. <= model.status()['hold_remaining_s'] - model.door_move_s <= 10.
    run(model, 3.)
    model.obstruct(True)
    run(model, 30.)
    assert model.status()['door'] == 'OPEN'
    model.obstruct(False)
    run(model, 45.)
    assert model.status()['door'] == 'CLOSED'


def test_other_passenger_stop_preserves_target_call():
    model = sim(faults={'stop_at_floor': '4F'})
    model.request(0., KIND_CAR_FLOOR, '', '5F', DIR_NONE)
    run(model, 8.2)
    assert model.status()['car_floor'] == '4F' and model.status()['door'] == 'OPEN'
    assert model.status()['car_calls'] == ['5F']
    run(model, 40.)
    assert model.status()['car_floor'] == '5F'


def test_door_and_press_failure():
    model = sim(faults={'door_never_opens': True})
    model.request(0., KIND_CALL, '4F', '3F', DIR_DOWN)
    run(model, 20.)
    assert model.status()['car_floor'] == '4F' and model.status()['door'] == 'CLOSED'
    model = sim(faults={'button_fails': True})
    model.request(0., KIND_CALL, '4F', '3F', DIR_DOWN)
    run(model, 20.)
    assert model.status()['car_floor'] == '3F'


def test_unknown_floor_and_invalid_requests_are_ignored():
    model = sim()
    for args in [(KIND_CALL, '9F', '3F', DIR_UP), (KIND_CALL, '4F', '3F', DIR_NONE),
                 (KIND_CAR_FLOOR, '', '9F', DIR_NONE), (77, '4F', '5F', DIR_UP)]:
        model.request(0., *args)
    run(model, 20.)
    assert model.status()['car_floor'] == '3F' and model.status()['door'] == 'CLOSED'


@pytest.mark.parametrize('kwargs', [dict(floors=[]), dict(floors=['3F','3F']), dict(floors=['']),
    dict(floors=FLOORS, floor_height_m=0), dict(floors=FLOORS, open_time_s=(10,6)),
    dict(floors=FLOORS, press_delay_s=-1), dict(floors=FLOORS, press_delay_s=float('nan'))])
def test_invalid_setup(kwargs):
    with pytest.raises(ValueError):
        ElevatorSim(**kwargs)
    with pytest.raises(ValueError):
        sim().set_faults(no_such_fault=1)


@pytest.fixture
def recognizer():
    """Generated messages and a fake clock/publisher; no ROS node or executor is started."""
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time
    from amr_sim_bridge.elevator_node import ArrivalRecognizer
    sent, callbacks = [], []
    clock = SimpleNamespace(nanoseconds=0, to_msg=lambda: Time(sec=clock.nanoseconds // 10**9))
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: clock),
                           create_publisher=lambda *_args: SimpleNamespace(publish=sent.append),
                           create_subscription=lambda _type, _topic, callback, _qos: callbacks.append(callback))
    return ArrivalRecognizer(node), clock, sent


def test_auto_confirm_delay_truth_and_once_per_candidate(recognizer):
    from types import SimpleNamespace
    model, clock, sent = recognizer
    state = SimpleNamespace(awaiting_confirm=True, zone=2, floor='4F', candidate_id=7)
    model.on_state(state)
    truth = dict(car_floor='4F', moving=False, door='OPEN')
    clock.nanoseconds = 999_000_000
    model.tick(truth)
    assert not sent
    clock.nanoseconds = 1_000_000_000
    model.tick(truth)
    assert sent[0].accepted and sent[0].source == 3 and sent[0].header.stamp.sec == 1
    assert sent[0].candidate_id == 7 and sent[0].actual_floor == ''
    for _ in range(3):
        clock.nanoseconds += 1_000_000_000
        model.on_state(state)
        model.tick(truth)
    assert len(sent) == 1
    state.awaiting_confirm = False
    model.on_state(state)
    state.awaiting_confirm = True
    model.on_state(state)
    clock.nanoseconds += 1_000_000_000
    model.tick(truth)
    assert len(sent) == 1  # awaiting false/true cannot rearm the same ID
    state.candidate_id = 8  # same floor, no intervening false required
    model.on_state(state)
    model.tick(truth)
    assert len(sent) == 1  # new candidate starts a fresh delay
    clock.nanoseconds += 1_000_000_000
    model.tick(dict(car_floor='3F', moving=False, door='OPEN'))
    assert len(sent) == 2 and not sent[-1].accepted and sent[-1].candidate_id == 8
    assert sent[-1].actual_floor == '3F'


@pytest.mark.parametrize('truth', [dict(car_floor='4F', moving=True, door='OPEN'),
                                    dict(car_floor='4F', moving=False, door='CLOSED'),
                                    dict(car_floor='4F', moving=False, door='OPENING')])
def test_auto_confirm_requires_stopped_car_and_open_door(recognizer, truth):
    from types import SimpleNamespace
    model, clock, sent = recognizer
    model.on_state(SimpleNamespace(awaiting_confirm=True, zone=2, floor='4F', candidate_id=7))
    clock.nanoseconds = 1_000_000_000
    model.tick(truth)
    assert len(sent) == 1 and not sent[0].accepted and sent[0].actual_floor == ''


def test_auto_confirm_disabled_or_withdrawn_candidate(recognizer):
    from types import SimpleNamespace
    model, clock, sent = recognizer
    state = SimpleNamespace(awaiting_confirm=True, zone=2, floor='4F', candidate_id=7)
    model.enabled = False
    model.on_state(state)
    clock.nanoseconds = 1_000_000_000
    model.tick(dict(car_floor='4F', moving=False, door='OPEN'))
    assert not sent
    model.enabled = True
    state.awaiting_confirm = False
    model.on_state(state)
    model.tick(dict(car_floor='4F', moving=False, door='OPEN'))
    assert not sent


def test_new_candidate_replaces_pending_confirmation_for_same_floor(recognizer):
    from types import SimpleNamespace
    model, clock, sent = recognizer
    state = SimpleNamespace(awaiting_confirm=True, zone=2, floor='4F', candidate_id=7)
    model.on_state(state)
    clock.nanoseconds = 500_000_000
    state.candidate_id = 8
    model.on_state(state)
    truth = dict(car_floor='4F', moving=False, door='OPEN')
    clock.nanoseconds = 1_000_000_000
    model.tick(truth)
    assert not sent
    clock.nanoseconds = 1_500_000_000
    model.tick(truth)
    assert [m.candidate_id for m in sent] == [8]


def test_adapter_request_subscription_retains_only_latest():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from amr_sim_bridge.elevator_node import ElevatorRosAdapter
    from amr_stack.floor_config import load_floor_config
    subscriptions = {}
    node = SimpleNamespace(
        create_publisher=lambda *_args: SimpleNamespace(publish=lambda _message: None),
        create_subscription=lambda _type, topic, _callback, qos: subscriptions.update({topic: qos}))
    ElevatorRosAdapter(node, load_floor_config())
    qos = subscriptions['/elevator/request']
    assert qos.depth == 1 and qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE


def test_auto_confirm_pass_floor_and_restart(recognizer):
    from types import SimpleNamespace
    model, clock, sent = recognizer
    model.on_state(SimpleNamespace(awaiting_confirm=True, zone=2, floor='4F', candidate_id=7))
    clock.nanoseconds = 1_000_000_000
    model.tick(dict(car_floor='1F', moving=False, door='OPEN'))
    assert sent[-1].actual_floor == '1F' and not sent[-1].accepted
    model.on_state(SimpleNamespace(awaiting_confirm=False, zone=2, floor='', candidate_id=0))
    model.on_state(SimpleNamespace(awaiting_confirm=True, zone=2, floor='4F', candidate_id=1))
    clock.nanoseconds = 2_000_000_000
    model.tick(dict(car_floor='4F', moving=False, door='OPEN'))
    assert len(sent) == 2 and sent[-1].accepted and sent[-1].actual_floor == ''
