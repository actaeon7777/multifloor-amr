import math

import pytest

from amr_sim_bridge.model import PhysXWheelAngle
from amr_can_bridge.telemetry import WheelOdometry


@pytest.mark.parametrize('direction', [1, -1])
def test_repeated_physx_boundary_crossings_preserve_continuous_travel(direction):
    angle = PhysXWheelAngle()
    for i in range(400):
        actual = direction * i * .4
        wrapped = (actual+2*math.pi) % (4*math.pi) - 2*math.pi
        assert angle.update(wrapped, i*.1) == pytest.approx(actual)
    assert angle.wraps > 10


def test_recorded_boundary_crossing_is_small_forward_motion():
    angle = PhysXWheelAngle()
    first = angle.update(5.93442154, 131.70499706)
    second = angle.update(-6.27451515, 131.80499705)
    assert second-first == pytest.approx(.3574339243591722)
    assert angle.wraps == 1


def test_actual_reset_is_still_rejected_by_wheel_odometry():
    decoder = WheelOdometry(left_sign=1., right_sign=1.)
    angles = [PhysXWheelAngle(), PhysXWheelAngle()]
    for t, positions in [(0., [6.1, 6.1]), (.1, [0., 0.])]:
        for i, side in enumerate(['left', 'right']):
            sample = decoder.update(side, angles[i].update(positions[i], t)/(2*math.pi), 0., t)
    assert sample is None
    assert decoder.rebases == 1
    assert not any(angle.wraps for angle in angles)


def test_gap_and_impossible_wrap_are_not_repaired():
    angle = PhysXWheelAngle()
    angle.update(6.1, 0.)
    assert angle.update(-6.1, .3) == pytest.approx(-6.1)
    assert angle.wraps == 0
    angle = PhysXWheelAngle()
    angle.update(5., 0.)
    # Corrected motion 2.57rad in 10ms exceeds the encoder motion bound.
    assert angle.update(-5., .01) == pytest.approx(-5.)
    assert angle.wraps == 0


def test_stale_sample_does_not_damage_unwrap_baseline():
    angle = PhysXWheelAngle()
    angle.update(6.1, 1.)
    angle.update(0., 1.)
    angle.update(0., .9)
    assert angle.update(-6.1, 1.1) == pytest.approx(4*math.pi-6.1)
    assert angle.wraps == 1


@pytest.mark.parametrize('position,stamp', [(math.nan, 1.), (1., math.inf)])
def test_nonfinite_samples_do_not_advance_state(position, stamp):
    angle = PhysXWheelAngle()
    angle.update(6.1, 0.)
    with pytest.raises(ValueError):
        angle.update(position, stamp)
    assert angle.update(-6.1, .1) == pytest.approx(4*math.pi-6.1)
