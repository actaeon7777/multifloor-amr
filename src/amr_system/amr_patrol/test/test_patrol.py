import math

import pytest

from amr_patrol.patrol import Patrol, validate_settings
from amr_patrol.routes import PatrolConfig, Route, Waypoint


def make(end='home', dwell=2., **kwargs):
    route = Route('test', '4F', end, (
        Waypoint(0., 0., None, 0., '첫 지점'),
        Waypoint(0., 2., None, dwell, '관측'),
        Waypoint(3., 2., 0., 0., '끝'),
    ))
    # Existing sequence tests use zero delays; timing behavior is tested below
    # with production defaults and explicit ROS-clock samples.
    kwargs = {'retry_delay_s': 0., 'battery_low_s': 0., **kwargs}
    return Patrol(PatrolConfig({'test': route}, home_pose=(9., 9., 1.), home_floor='4F'), 0., **kwargs)


def start(p, **kwargs):
    values = dict(route='test', request_id='request_123', floor='4F', battery_v=28., nav_busy=False)
    values.update(kwargs)
    return p.start(**values)


def nav(p, now=0.):
    action = p.tick(now)
    assert action[0] == 'navigate'
    return action[1]


def test_three_points_dwell_and_home():
    p = make()
    assert start(p)
    assert nav(p) == (0., 0., 0.)
    p.on_nav_result(True)
    assert nav(p) == (0., 2., math.pi / 2)
    p.on_nav_result(True)
    assert p.phase == 'DWELL' and p.index == 2
    begin = p.tick(0.)
    assert begin[0] == 'publish_observation' and begin[1]['event'] == 'begin'
    assert begin[1]['yaw'] == math.pi / 2
    assert p.tick(1.) == ('wait', 1.)
    end = p.tick(2.)
    assert end[1]['event'] == 'end' and end[1]['index'] == 2
    assert nav(p, 2.) == (3., 2., 0.)
    p.on_nav_result(True)
    assert p.phase == 'RETURN' and p.index == 4
    assert nav(p, 2.) == (9., 9., 1.)
    p.on_nav_result(True)
    assert p.tick(3.)[0] == 'finish'
    status = p.status(10.)
    assert status['phase'] == 'DONE' and status['active'] is False
    assert status['elapsed_s'] == 2.
    assert status['routes'] == ['test'] and status['total'] == 3
    assert [event['kind'] for event in p.events] == ['patrol.started', 'patrol.finished']
    assert p.tick(20.) is None and p.phase == 'DONE'


def test_retry_then_skip_and_reset_retries():
    p = make(dwell=0.)
    start(p)
    first = nav(p)
    p.on_nav_result(False)
    assert nav(p) == first and p.retries == 1
    p.on_nav_result(False)
    nav(p)
    assert p.skipped == [1] and p.index == 2
    assert p.retries == 0 and p.failures == 1
    p.on_nav_result(True)
    assert p.failures == 0
    assert list(p.events)[1]['kind'] == 'patrol.waypoint_skipped'


def test_consecutive_failures_abort():
    p = make(max_retries=0)
    start(p)
    for _ in range(3):
        nav(p)
        p.on_nav_result(False)
    assert p.phase == 'ABORTED' and p.skipped == [1, 2, 3]
    assert p.tick(0.)[0] == 'finish'


@pytest.mark.parametrize('reason', ['순찰 정지 요청', '관리자 정지', 'E-stop'])
def test_stop_during_nav_never_returns_or_resumes(reason):
    p = make()
    start(p)
    nav(p)
    serial = p.goal_serial
    p.stop(reason)
    assert p.phase == 'ABORTED' and p.detail == reason and p.goal_serial != serial
    p.on_nav_result(True)  # late result cannot advance
    assert p.tick(0.)[0] == 'finish'
    assert p.tick(100.) is None


def test_stop_during_dwell_closes_observation():
    p = make()
    start(p)
    nav(p)
    p.on_nav_result(True)
    nav(p)
    p.on_nav_result(True)
    assert p.tick(0.)[1]['event'] == 'begin'
    p.stop()
    assert p.tick(1.)[1]['event'] == 'end'
    assert p.tick(1.)[0] == 'finish'


@pytest.mark.parametrize('in_dwell', [False, True])
def test_low_battery_returns_early(in_dwell):
    p = make(end='stay')
    start(p)
    nav(p)
    if in_dwell:
        p.on_nav_result(True)
        nav(p)
        p.on_nav_result(True)
        assert p.tick(0.)[1]['event'] == 'begin'
    p.battery(26.)
    assert p.phase == 'RETURN' and p.index == 4
    if in_dwell:
        assert p.tick(0.)[1]['event'] == 'end'
    assert nav(p) == (9., 9., 1.)
    p.battery(25.)  # does not restart the return goal
    assert p.tick(0.) is None
    p.on_nav_result(True)
    assert p.phase == 'DONE' and p.detail == '배터리 부족으로 조기 복귀'


def test_disabled_battery_limit_and_unavailable_voltage():
    p = make(min_battery_voltage=0)
    assert start(p, battery_v=1.)
    nav(p)
    p.battery(1.)
    assert p.phase == 'NAV'
    for voltage in (None, float('nan'), float('inf')):
        assert start(make(), battery_v=voltage)


def test_stay_finishes_without_home_goal():
    p = make(end='stay', dwell=0.)
    start(p)
    for _ in range(3):
        nav(p)
        p.on_nav_result(True)
    assert p.phase == 'DONE' and p.index == 3
    assert p.tick(0.)[0] == 'finish'
    assert start(p) and p.phase == 'NAV'


@pytest.mark.parametrize('values', [
    dict(route='missing'), dict(route=[]), dict(request_id='bad'), dict(request_id=None),
    dict(floor='3F'), dict(confidence='UNKNOWN'), dict(confidence='TRANSIT'),
    dict(in_elevator=True), dict(held=True), dict(estop_healthy=False),
    dict(nav_busy=True), dict(battery_v=26.),
])
def test_start_rejection(values):
    p = make()
    assert not start(p, **values)
    assert p.last_command['accepted'] is False and p.last_command['detail']
    assert p.phase == 'IDLE' and not p.actions and not p.events


def test_already_active_rejection_preserves_patrol():
    p = make()
    start(p)
    serial = p.goal_serial
    assert not start(p)
    assert p.phase == 'NAV' and p.goal_serial == serial


def test_missing_or_other_floor_home():
    p = make()
    p.config = PatrolConfig(p.config.routes)
    assert not start(p)
    p = make(end='stay')
    p.config = PatrolConfig(p.config.routes, home_pose=(1., 2., 3.), home_floor='3F')
    assert start(p)
    nav(p)
    p.battery(25.)
    assert p.phase == 'ABORTED'


def test_timeout_retries_then_skips():
    p = make(nav_timeout_s=5.)
    start(p)
    first = nav(p)
    assert p.tick(4.) is None
    assert nav(p, 5.) == first
    assert nav(p, 10.) != first and p.skipped == [1]


def test_timeout_with_live_clock_does_not_look_like_clock_reversal():
    p = make(nav_timeout_s=1.)
    now = [0.]

    def clock():
        now[0] += .0001
        return now[0]

    p.clock = clock
    assert start(p)
    first = nav(p, clock())
    now[0] = 1.1
    assert nav(p, clock()) == first
    assert p.phase == 'NAV' and p.retries == 1


def test_return_failure_is_not_success_or_skipped():
    p = make(max_retries=1)
    start(p)
    nav(p)
    p.battery(25.)
    nav(p)
    p.on_nav_result(False)
    nav(p)
    p.on_nav_result(False)
    assert p.phase == 'ABORTED' and p.detail == 'home 복귀 실패' and p.skipped == []


def test_clock_reversal_aborts():
    p = make()
    start(p)
    nav(p, 3.)
    assert p.tick(2.)[0] == 'finish'
    assert p.phase == 'ABORTED'


def test_callback_before_tick_cannot_hide_clock_reversal():
    p = make()
    now = [3.]
    p.clock = lambda: now[0]
    assert start(p)
    nav(p, 3.)
    now[0] = 2.
    p.battery(28.)
    assert p.phase == 'ABORTED'
    p.on_nav_result(True)
    assert p.tick(2.)[0] == 'finish'


@pytest.mark.parametrize('key,value', [
    ('tick_period', 0), ('tick_period', float('nan')), ('nav_timeout_s', -1),
    ('goal_xy_tolerance', 0), ('goal_xy_tolerance', float('inf')),
    ('min_battery_voltage', -1), ('min_battery_voltage', True),
    ('max_retries', -1), ('max_retries', 1.5), ('max_retries', True),
    ('max_consecutive_failures', 0), ('max_consecutive_failures', 2.5),
    ('retry_delay_s', -1), ('retry_delay_s', float('nan')), ('retry_delay_s', True),
    ('battery_low_s', -1), ('battery_low_s', float('inf')), ('battery_low_s', False),
    ('cancel_result_timeout_s', 0), ('cancel_result_timeout_s', float('nan')),
    ('require_estop', 'false'),
])
def test_invalid_settings(key, value):
    settings = dict(tick_period=.2, nav_timeout_s=180., goal_xy_tolerance=.3,
                    min_battery_voltage=26.1, max_retries=1, max_consecutive_failures=3)
    settings[key] = value
    with pytest.raises(ValueError):
        validate_settings(settings)


def test_default_retry_waits_before_resending_and_advancing():
    p = Patrol(make().config, 0.)
    assert p.retry_delay_s == 3. and p.battery_low_s == 10.
    assert start(p)
    first = nav(p)
    p.on_nav_result(False)
    assert p.tick(0.) == ('wait', 3.)
    assert p.tick(2.9)[0] == 'wait'
    assert nav(p, 3.) == first
    p.on_nav_result(False)
    assert p.skipped == [1] and p.index == 1
    assert p.tick(5.9)[0] == 'wait'
    assert nav(p, 6.) != first and p.index == 2


@pytest.mark.parametrize('state,held', [('MANUAL', False), ('IDLE', True)])
def test_manual_or_held_rejection_does_not_count(state, held):
    p = make(retry_delay_s=3.)
    start(p)
    first = nav(p)
    p.on_nav_result(False, nav_state=state, held=held)
    assert p.detail == '수동 조작 중: 순찰 대기'
    assert p.tick(100.)[0] == 'wait'
    assert p.retries == p.failures == 0 and not p.skipped
    p.navigation_state('IDLE')
    assert nav(p, 100.) == first
    p.on_nav_result(False, nav_state=state, held=held)
    assert p.retries == p.failures == 0


def test_manual_blocks_queued_goal_and_timeout_until_release():
    p = make(retry_delay_s=3., nav_timeout_s=1.)
    start(p)
    p.navigation_state('MANUAL')
    assert p.tick(100.) == ('wait', .2)
    assert p.retries == 0
    p.navigation_state('IDLE')
    nav(p, 100.)
    p.on_nav_result(False)
    p.stop()
    assert p.tick(200.)[0] == 'finish' and p.phase == 'ABORTED'


@pytest.mark.parametrize('state', ['RUNNING', 'WAITING_NAV2', 'MANEUVER'])
def test_external_navigation_after_failed_goal_aborts_without_retry(state):
    p = make()
    start(p)
    nav(p)
    p.on_nav_result(False, nav_state=state, other_navigation=True)
    assert p.phase == 'ABORTED' and p.detail == '다른 주행 명령으로 순찰 중단'
    assert p.retries == p.failures == 0 and not p.skipped
    assert p.tick(0.)[0] == 'finish'
    assert p.tick(100.) is None


def test_external_navigation_status_arriving_during_retry_delay_aborts():
    p = make(retry_delay_s=3.)
    start(p)
    nav(p)
    p.on_nav_result(False)
    p.navigation_state('RUNNING', other_navigation=True)
    assert p.phase == 'ABORTED'
    assert p.tick(10.)[0] == 'finish'


def test_battery_sag_recovers_then_requires_continuous_low_readings():
    p = make(battery_low_s=10.)
    start(p)
    nav(p)
    p.battery(25.)
    p.tick(9.9)
    p.battery(25.)
    assert p.phase == 'NAV'
    p.battery(27.)
    p.tick(10.)
    p.battery(25.)
    p.tick(19.9)
    p.battery(25.)
    assert p.phase == 'NAV'
    p.tick(20.)
    p.battery(25.)
    assert p.phase == 'RETURN'
    assert nav(p, 20.) == p.config.home_pose


@pytest.mark.parametrize('unavailable', [None, float('nan'), float('inf')])
def test_unavailable_battery_breaks_low_interval(unavailable):
    p = make(battery_low_s=10.)
    start(p)
    nav(p)
    p.battery(25.)
    p.tick(9.)
    p.battery(unavailable)
    p.tick(20.)
    p.battery(25.)
    assert p.phase == 'NAV' and p.battery_low_since == 20.
    # Paused ROS time cannot accumulate the low-voltage interval.
    for _ in range(20):
        p.battery(25.)
    assert p.phase == 'NAV'
    p.tick(30.)
    p.battery(25.)
    assert p.phase == 'RETURN'


def test_start_voltage_check_is_instantaneous_despite_debounce():
    p = make(battery_low_s=10.)
    assert not start(p, battery_v=25.)


def test_stop_while_manual_preserves_reason_after_release():
    p = make()
    start(p)
    nav(p)
    p.navigation_state('MANUAL')
    p.stop('순찰 정지 요청')
    p.navigation_state('IDLE')
    assert p.phase == 'ABORTED' and p.detail == '순찰 정지 요청'


def test_manual_time_does_not_consume_navigation_timeout():
    p = make(nav_timeout_s=1.)
    start(p)
    nav(p)
    p.navigation_state('MANUAL')
    assert p.tick(100.)[0] == 'wait'
    p.navigation_state('IDLE')
    assert p.tick(100.) is None and p.retries == 0
    p.on_nav_result(True)
    assert p.index == 2
