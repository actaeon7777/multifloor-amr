"""Measured trajectories, protocol edge cases, dropouts and IMU pairing."""
from math import pi, sin, cos
import struct

import pytest


def test_clock_rebase_keeps_pose_and_accepts_new_epoch():
    wheels = WheelOdometry()
    pair(wheels, 0., 0., 100.)
    pair(wheels, .1, .1, 100.1)
    before = (wheels.x, wheels.y, wheels.yaw, wheels.travel)
    wheels.reset_time()
    pair(wheels, .2, .2, 1.)
    assert (wheels.x, wheels.y, wheels.yaw, wheels.travel) == before
    pair(wheels, .21, .21, 1.1)
    assert wheels.x > before[0]
    imu = ImuPairer()
    assert imu_pair(imu, 100, 100.) is not None
    imu.reset_time()
    assert imu_pair(imu, 0, 1.) is not None

from amr_can_bridge.telemetry import decode_frame, WheelOdometry, ImuPairer


def pair(wheels, left, right, stamp, lv=0., rv=0.):
    assert wheels.update('left', -left, -lv, stamp) is None
    return wheels.update('right', right, rv, stamp)


def test_protocol_and_physical_mapping():
    assert decode_frame(0x009, struct.pack('<ff', 3., 1.)) == ('right', (3., 1.))
    assert decode_frame(0x029, struct.pack('<ff', -3., -1.)) == ('left', (-3., -1.))
    assert decode_frame(0x301, struct.pack('<hhhH', -32768, 16384, 32767, 65535)) == (
        'accel', (-32768, 16384, 32767, 65535))
    for flag in ('extended', 'remote', 'error', 'fd'):
        assert decode_frame(0x009, bytes(8), **{flag: True}) is None
    assert decode_frame(0x201, bytes(8)) is None
    with pytest.raises(ValueError):
        decode_frame(0x029, bytes(7))
    for bad in (float('nan'), float('inf'), -float('inf')):
        with pytest.raises(ValueError):
            decode_frame(0x009, struct.pack('<ff', bad, 0.))


@pytest.mark.parametrize('sign', [-1., 1.])
def test_straight_measured_distance_and_joint_units(sign):
    w = WheelOdometry()
    first = pair(w, 20., 35., 1.)  # Boot encoder offsets must not become odometry.
    assert (first.x, first.y, first.yaw) == (0., 0., 0.)
    for n in range(1, 11):
        sample = pair(w, 20.+sign*n*.1, 35.+sign*n*.1, 1.+n*.1, sign, sign)
    assert sample.x == pytest.approx(sign*pi*.172)
    assert sample.y == pytest.approx(0.)
    assert sample.yaw == pytest.approx(0.)
    assert sample.v == pytest.approx(sign*pi*.172)
    assert sample.w == pytest.approx(0.)
    assert sample.positions == pytest.approx(((20.+sign)*2*pi, (35.+sign)*2*pi))
    assert sample.velocities == pytest.approx((sign*2*pi, sign*2*pi))


@pytest.mark.parametrize('sign', [-1., 1.])
def test_pure_turn_and_exact_arc(sign):
    w = WheelOdometry()
    pair(w, 0., 0., 1.)
    s = pair(w, -sign*.1, sign*.1, 1.1, -sign, sign)
    angle = sign*.2*pi*.172/.499
    assert (s.x, s.y) == pytest.approx((0., 0.))
    assert s.yaw == pytest.approx(angle)
    assert s.w == pytest.approx(angle/.1)
    w = WheelOdometry()
    pair(w, 0., 0., 1.)
    s = pair(w, 0., sign*.2, 1.1, 0., sign*2.)
    assert s.x == pytest.approx(.499/2*sin(angle))
    assert s.y == pytest.approx(.499/2*(1-cos(angle)))


def test_asynchronous_pairing_no_old_wheel_reuse():
    w = WheelOdometry(pair_slop=.02)
    pair(w, 0., 0., 1.)
    assert w.update('left', -.1, -1., 1.1) is None
    assert w.update('left', -.2, -1., 1.2) is None
    assert w.update('right', .1, 1., 1.1) is None  # Outside pair slop.
    s = w.update('right', .2, 1., 1.201)
    assert s.x == pytest.approx(.2*pi*.172)
    assert w.update('right', .2, 1., 1.201) is None  # Duplicate timestamp.
    assert w.update('right', .2, 1., 1.205) is None  # No fresh left.


def test_gaps_and_encoder_reset_rebase_without_pose_jump():
    w = WheelOdometry()
    pair(w, 10., 10., 1.)
    first = pair(w, 10.1, 10.1, 1.1)
    assert pair(w, 12., 12., 2.) is None  # Unobserved interval is not integrated.
    assert w.rebases == 1
    s = pair(w, 12.1, 12.1, 2.1)
    assert s.x == pytest.approx(first.x*2)
    assert pair(w, 0., 0., 2.2) is None  # Encoder restart.
    assert w.rebases == 2
    s = pair(w, .1, .1, 2.3)
    assert s.x == pytest.approx(first.x*3)


def test_corrupt_velocity_and_geometry_rejected():
    w = WheelOdometry()
    assert w.update('right', 1., 100., 1.) is None
    assert w.dropped == 1
    for name in ('diameter', 'track', 'turns_per_wheel_turn', 'pair_slop', 'timeout', 'max_rev_s'):
        for bad in (0, -1, float('nan'), float('inf')):
            with pytest.raises(ValueError):
                WheelOdometry(**{name: bad})
    with pytest.raises(ValueError):
        WheelOdometry(left_sign=0)
    with pytest.raises(ValueError):
        WheelOdometry(pair_slop=.5, timeout=.1)
    w = WheelOdometry(turns_per_wheel_turn=2.)
    pair(w, 0., 0., 1.)
    s = pair(w, .2, .2, 1.1, 2., 2.)
    assert s.x == pytest.approx(.1*pi*.172)
    assert s.v == pytest.approx(pi*.172)


def imu_pair(imu, seq, stamp):
    assert imu.update('gyro', (1000, -1000, 0, seq), stamp) is None
    return imu.update('accel', (0, 0, 16384, seq), stamp+.001)


def test_imu_units_and_bias_no_gravity_removal():
    imu = ImuPairer(gyro_bias=(.01, 0., 0.))
    s = imu_pair(imu, 1, 1.)
    assert s.acceleration == pytest.approx((0., 0., 16384*.000061*9.80665))
    assert s.angular_velocity == pytest.approx((4.375*pi/180-.01, -4.375*pi/180, 0.))


def test_imu_pairs_by_sequence_not_arrival_order_and_expires():
    imu = ImuPairer()
    assert imu.update('accel', (0, 0, 0, 1), 1.) is None
    assert imu.update('gyro', (0, 0, 0, 2), 1.001) is None
    assert imu.update('accel', (0, 0, 0, 2), 1.002).sequence == 2
    assert imu.update('gyro', (0, 0, 0, 1), 1.003) is None  # Old sequence.
    assert imu.update('accel', (0, 0, 0, 3), 1.1) is None
    assert imu.update('gyro', (0, 0, 0, 3), 1.2) is None  # Match arrived too late.
    assert imu.dropped > 0
    assert len(imu.pending) <= 1


def test_imu_wrap_duplicate_and_reboot():
    imu = ImuPairer()
    assert imu_pair(imu, 65535, 1.).sequence == 65535
    assert imu_pair(imu, 0, 1.1).sequence == 0
    assert imu_pair(imu, 0, 1.2) is None
    assert imu_pair(imu, 100, 1.3).sequence == 100
    assert imu_pair(imu, 1, 1.4) is None  # Out of order/reset not yet trusted.
    assert imu_pair(imu, 2, 2.).sequence == 2  # Reboot recovery after timeout.
    with pytest.raises(ValueError):
        ImuPairer(gyro_range_dps=42)
    with pytest.raises(ValueError):
        ImuPairer(gyro_bias=(float('nan'), 0., 0.))


def test_power_and_heartbeat_mapping_and_malformed_frames():
    for cid,side in ((0x017,'right'),(0x037,'left')):
        assert decode_frame(cid,struct.pack('<ff',26.25,-.5)) == (side+'_power',(26.25,-.5))
        for flag in ('extended','remote','error','fd'):
            assert decode_frame(cid,bytes(8),**{flag:True}) is None
        for voltage,current in ((float('nan'),0.),(26.,float('inf')),(0.,0.),(-1.,1.)):
            with pytest.raises(ValueError):
                decode_frame(cid,struct.pack('<ff',voltage,current))
        with pytest.raises(ValueError):
            decode_frame(cid,bytes(4))
    assert decode_frame(0x001,struct.pack('<IBBBB',0x1234,8,1,2,3)) == ('right_heartbeat',(0x1234,8))
    assert decode_frame(0x021,struct.pack('<IBBBB',0,1,0,0,0)) == ('left_heartbeat',(0,1))
    # 0x15 is firmware-dependent: never interpret it as temperature blindly.
    assert decode_frame(0x015,struct.pack('<ff',123.,456.)) is None
