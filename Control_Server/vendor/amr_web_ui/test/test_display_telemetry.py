import pytest
from amr_web_ui.server import SharedState
from amr_web_ui.telemetry import DisplayTelemetry


def samples(session='publisher',epoch=0):
    return dict(vehicle=dict(session=session,epoch=epoch,age=0.,valid=True,fresh=True,hz=None),
                wheel=dict(age=0.,valid=True,fresh=True,hz=100,linear=.3))


def test_common_status_lease_expires_even_with_fresh_source_flags():
    telemetry = DisplayTelemetry()
    telemetry.receive_vehicle(samples(),dict(value=2.,epoch=0,valid=True),False)
    delayed = telemetry.snapshot(now=telemetry.vehicle_received+.6)
    assert not delayed['wheel']['fresh']
    assert delayed['wheel']['valid']
    assert delayed['wheel']['hz'] is None
    assert delayed['vehicle']['age']==pytest.approx(.6)


def test_web_goal_distance_excludes_previous_driving_and_resets_for_new_goal():
    telemetry = DisplayTelemetry()
    telemetry.receive_vehicle(samples(),dict(value=2.,epoch=0,valid=True),False)
    telemetry.begin_journey('first')
    telemetry.receive_vehicle(samples(),dict(value=2.5,epoch=0,valid=True),True)
    assert telemetry.snapshot()['journey']['travelled']==.5
    assert telemetry.snapshot()['journey']['complete']
    telemetry.receive_vehicle(samples(),dict(value=3.,epoch=0,valid=True),False)
    assert telemetry.snapshot()['journey']['travelled']==.5
    telemetry.begin_journey('second')
    assert telemetry.snapshot()['journey']['travelled']==0
    assert telemetry.snapshot()['journey']['request_id']=='second'


@pytest.mark.parametrize('distance',[
    dict(value=0.,epoch=1,valid=True),dict(value=1.,epoch=0,valid=True),dict(value=2.5,epoch=0,valid=False)])
def test_source_distance_gap_or_reset_invalidates_the_goal_distance(distance):
    telemetry = DisplayTelemetry()
    telemetry.receive_vehicle(samples(),dict(value=2.,epoch=0,valid=True),False)
    telemetry.begin_journey('goal')
    telemetry.receive_vehicle(samples(),distance,True)
    assert not telemetry.snapshot()['journey']['complete']


def test_missing_start_is_unknown_and_controller_restart_clears_display():
    shared = SharedState()
    shared.telemetry.begin_journey('goal')
    shared.telemetry.receive_vehicle(samples(),dict(value=2.,epoch=0,valid=True),True)
    assert not shared.snapshot()['telemetry']['journey']['complete']
    shared.reset_connection('controller restart')
    assert shared.snapshot()['telemetry']=={'journey':None}
    assert shared.commands.empty()


def test_status_publisher_restart_discards_the_old_distance_baseline():
    telemetry = DisplayTelemetry()
    telemetry.receive_vehicle(samples(),dict(value=2.,epoch=0,valid=True),False)
    telemetry.begin_journey('goal')
    telemetry.receive_vehicle(samples('restarted'),dict(value=0.,epoch=0,valid=True),True)
    assert telemetry.snapshot()['journey'] is None
