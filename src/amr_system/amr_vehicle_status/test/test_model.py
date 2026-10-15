import pytest
from amr_vehicle_status.model import StatusModel


def test_source_rate_expiry_and_ros_time_age():
    model = StatusModel()
    for i in range(11):
        model.observe('wheel',dict(linear=.32),10+i/10,stamp=100+i/10)
    sample = model.snapshot('wheel',11.1,101.1)
    assert sample['rate_hz'] == pytest.approx(10)
    assert sample['fresh']
    assert sample['age_sec'] == pytest.approx(.1)
    assert not model.snapshot('wheel',12,102)['fresh']
    assert model.snapshot('wheel',12,102)['valid']
    # Network delay already present at reception must count against freshness.
    model.observe('scan',{},12,stamp=100)
    assert not model.snapshot('scan',12,102)['fresh']


def test_reordered_and_invalid_samples_do_not_refresh_a_value():
    model = StatusModel()
    model.observe('ekf',dict(linear=.3),10,stamp=100)
    assert not model.observe('ekf',dict(linear=9),10.1,stamp=99)
    assert model.snapshot('ekf',10.2,100.2)['values']['linear'] == .3
    model.reject('ekf',10.2)
    sample = model.snapshot('ekf',10.2,100.2)
    assert sample['received'] and not sample['valid'] and not sample['fresh']
    assert model.observe('ekf',dict(linear=.2),10.3,stamp=100.3)
    assert model.snapshot('ekf',10.3,100.3)['fresh']


@pytest.mark.parametrize('x,stamp',[(5.,100.1),(.01,103.),(.01,99.)])
def test_odometry_gap_or_jump_changes_continuity_epoch(x,stamp):
    model = StatusModel()
    model.wheel(0,0,100)
    model.wheel(.03,.04,100.1)
    assert model.distance == pytest.approx(.05)
    previous = model.distance_epoch
    model.wheel(x,0,stamp)
    assert model.distance_epoch == previous+1
    assert model.distance == 0


def test_clock_reset_clears_sensor_validity_and_distance():
    model = StatusModel()
    model.observe('joints',dict(left_rpm=60.,right_rpm=30.),10,stamp=100)
    model.wheel(0,0,100)
    model.reset()
    assert not model.snapshot('joints',10,100)['received']
    assert model.last_wheel is None and model.distance_epoch == 1


def test_wall_stamped_supervisor_status_can_expire_during_sim_pause():
    model = StatusModel()
    model.observe('localization',dict(ready=True),10,base_age=1.9)
    assert model.snapshot('localization',10,100)['fresh']
    assert not model.snapshot('localization',10.2,100)['fresh']
