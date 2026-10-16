"""Measured wheel odometry and sequenced IMU decoding, independent of ROS/CAN IO."""

from dataclasses import dataclass
from math import atan2, cos, isfinite, pi, sin
import struct


ENCODER_IDS = {0x009: 'right', 0x029: 'left'}
IMU_IDS = {0x301: 'accel', 0x302: 'gyro'}
POWER_IDS = {0x017: 'right_power', 0x037: 'left_power'}
HEARTBEAT_IDS = {0x001: 'right_heartbeat', 0x021: 'left_heartbeat'}
TELEMETRY_IDS = (*ENCODER_IDS, *IMU_IDS, *POWER_IDS, *HEARTBEAT_IDS)
ACCEL_SCALES = {2: 0.061, 4: 0.122, 8: 0.244, 16: 0.488}  # mg/LSB
GYRO_SCALES = {125: 4.375, 250: 8.75, 500: 17.5, 1000: 35., 2000: 70., 4000: 140.}


def positive(name, value):
    if not isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def decode_frame(can_id, data, *, extended=False, remote=False, error=False, fd=False):
    """Return None for other traffic; reject malformed telemetry, never interpret RTR."""
    if extended or remote or error or fd or can_id not in TELEMETRY_IDS:
        return None
    if len(data) != 8:
        raise ValueError('telemetry DLC must be 8')
    if can_id in POWER_IDS:
        values = struct.unpack('<ff', data)
        if not all(isfinite(x) for x in values) or values[0] <= 0:
            raise ValueError('invalid bus voltage/current')
        return POWER_IDS[can_id], values
    if can_id in HEARTBEAT_IDS:
        # Common CANSimple prefix; firmware-specific trailing bytes are ignored.
        return HEARTBEAT_IDS[can_id], struct.unpack_from('<IB', data)
    if can_id in ENCODER_IDS:
        values = struct.unpack('<ff', data)
        if not all(isfinite(x) for x in values):
            raise ValueError('nonfinite encoder estimate')
        return ENCODER_IDS[can_id], values
    return IMU_IDS[can_id], struct.unpack('<hhhH', data)


@dataclass
class WheelSample:
    stamp: float
    x: float
    y: float
    yaw: float
    v: float
    w: float
    positions: tuple  # left, right; rad
    velocities: tuple  # left, right; rad/s


class WheelOdometry:
    """Integrate paired absolute encoder deltas; rebase after gaps or encoder resets.

    Encoder estimates refer to output/wheel turns with the default ratio 1.
    Node 0 is physically RIGHT; node 1 is LEFT, despite old STM32 macro names.
    """

    def __init__(self, diameter=.172, track=.529, left_sign=-1., right_sign=1.,
                 turns_per_wheel_turn=1., pair_slop=.02, timeout=.25, max_rev_s=5.):
        self.circumference = pi * positive('wheel_diameter', diameter)
        self.track = positive('track_width', track)
        self.ratio = positive('encoder_turns_per_wheel_turn', turns_per_wheel_turn)
        self.slop = positive('wheel_pair_slop', pair_slop)
        self.timeout = positive('wheel_timeout', timeout)
        if self.slop >= self.timeout:
            raise ValueError('wheel_pair_slop must be smaller than wheel_timeout')
        self.max_rev_s = positive('max_valid_wheel_rev_s', max_rev_s)
        if left_sign not in (-1., 1.) or right_sign not in (-1., 1.):
            raise ValueError('wheel direction signs must be -1 or +1')
        self.signs = {'left': left_sign, 'right': right_sign}
        self.pending, self.last_seen = {}, {}
        self.previous = None
        self.x = self.y = self.yaw = self.travel = self.rotation = 0.
        self.rebases = self.dropped = 0

    def reset_time(self):
        """Discard timing/pairing history without resetting integrated pose."""
        self.pending.clear()
        self.last_seen.clear()
        self.previous = None
        self.rebases += 1

    def update(self, side, position, velocity, stamp):
        if (side not in self.signs or not all(isfinite(v) for v in (position, velocity, stamp))):
            raise ValueError('invalid wheel sample')
        if stamp <= self.last_seen.get(side, float('-inf')):
            self.dropped += 1
            return None
        self.last_seen[side] = stamp
        sign = self.signs[side] / self.ratio
        position, velocity = position * sign, velocity * sign
        if abs(velocity) > self.max_rev_s:
            self.pending.clear()
            self.dropped += 1
            return None
        self.pending[side] = (position, velocity, stamp)
        if len(self.pending) != 2:
            return None
        left, right = self.pending['left'], self.pending['right']
        if abs(left[2] - right[2]) > self.slop:
            del self.pending['left' if left[2] < right[2] else 'right']
            self.dropped += 1
            return None
        self.pending.clear()
        stamp = (left[2] + right[2]) / 2
        current = (left[0], right[0], stamp)
        previous, self.previous = self.previous, current
        if previous is not None:
            dt = stamp - previous[2]
            dl, dr = left[0] - previous[0], right[0] - previous[1]
            # Margin permits finite encoder noise; do not integrate a reset/jump.
            if dt <= 0 or dt > self.timeout or max(abs(dl), abs(dr)) > self.max_rev_s * dt + .02:
                self.rebases += 1
                return None
            distance = (dl + dr) * self.circumference / 2
            angle = (dr - dl) * self.circumference / self.track
            half = angle / 2
            chord = distance * (sin(half) / half if abs(half) > 1e-9 else 1.)
            self.x += chord * cos(self.yaw + half)
            self.y += chord * sin(self.yaw + half)
            self.yaw = atan2(sin(self.yaw + angle), cos(self.yaw + angle))
            self.travel += abs(distance)
            self.rotation += abs(angle)
        v = (left[1] + right[1]) * self.circumference / 2
        w = (right[1] - left[1]) * self.circumference / self.track
        return WheelSample(stamp, self.x, self.y, self.yaw, v, w,
                           (left[0] * 2*pi, right[0] * 2*pi),
                           (left[1] * 2*pi, right[1] * 2*pi))


@dataclass
class ImuSample:
    stamp: float
    sequence: int
    acceleration: tuple
    angular_velocity: tuple


class ImuPairer:
    """Pair exactly matching STM32 sequence numbers, including uint16 wraparound."""

    def __init__(self, accel_range_g=2, gyro_range_dps=125, pair_slop=.05, timeout=.5,
                 gyro_bias=(0., 0., 0.)):
        if accel_range_g not in ACCEL_SCALES or gyro_range_dps not in GYRO_SCALES:
            raise ValueError('unsupported LSM6DSV16X full-scale range')
        self.accel_scale = ACCEL_SCALES[accel_range_g] * .001 * 9.80665
        self.gyro_scale = GYRO_SCALES[gyro_range_dps] * .001 * pi / 180
        self.slop = positive('imu_pair_slop', pair_slop)
        self.timeout = positive('imu_timeout', timeout)
        if self.slop >= self.timeout:
            raise ValueError('imu_pair_slop must be smaller than imu_timeout')
        if len(gyro_bias) != 3 or not all(isfinite(v) for v in gyro_bias):
            raise ValueError('gyro_bias must have three finite rad/s values')
        self.bias = tuple(gyro_bias)
        self.pending, self.last = {}, None
        self.dropped = 0

    def reset_time(self):
        self.pending.clear()
        self.last = None

    def update(self, kind, values, stamp):
        if kind not in ('accel', 'gyro') or not isfinite(stamp):
            raise ValueError('invalid IMU sample')
        x, y, z, sequence = values
        if self.last is not None:
            old_seq, old_stamp = self.last
            if stamp <= old_stamp:
                self.dropped += 1
                return None
            delta = (sequence - old_seq) & 0xffff
            if stamp - old_stamp < self.timeout and (delta == 0 or delta >= 0x8000):
                self.dropped += 1
                return None
        # Bounded cache: unmatched frames cannot persist into the next cycle.
        for key, (unused, received) in list(self.pending.items()):
            if stamp - received > self.slop:
                del self.pending[key]
                self.dropped += 1
        self.pending[(sequence, kind)] = ((x, y, z), stamp)
        other_kind = 'gyro' if kind == 'accel' else 'accel'
        if (sequence, other_kind) not in self.pending:
            return None
        accel, at = self.pending.pop((sequence, 'accel'))
        gyro, gt = self.pending.pop((sequence, 'gyro'))
        if abs(at - gt) > self.slop:
            self.dropped += 1
            return None
        sample_stamp = max(at, gt)
        self.last = (sequence, sample_stamp)
        return ImuSample(sample_stamp, sequence,
                         tuple(v * self.accel_scale for v in accel),
                         tuple(v * self.gyro_scale - b for v, b in zip(gyro, self.bias)))
