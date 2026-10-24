"""STM32 E-stop status (0x202, version 1) and the Jetson restart interlock.

The STM32 releases its own stop when the button returns. The bridge then re-arms
by itself: only (0, 0) is sent until the STM32 reports its gate open, and motion
resumes only from commands issued after that. See docs/ESTOP_CAN_PROTOCOL.md.
"""

import struct


ESTOP_STATUS_ID = 0x202
ESTOP_STATUS_VERSION = 1
FLAG_RAW, FLAG_DEBOUNCED, FLAG_LATCHED, FLAG_INHIBIT = 0x01, 0x02, 0x04, 0x08
FLAG_ODRV_IDLE, FLAG_BUS_OFF, FLAG_ODRV_HB_OK, FLAG_DRIVE_ENABLED = 0x10, 0x20, 0x40, 0x80

UNKNOWN, ESTOP, ARMING, READY = 'UNKNOWN', 'ESTOP', 'ARMING', 'READY'
# What the bridge may write to 0x201 in each state.
SEND_NOTHING, SEND_ZERO, SEND_COMMAND = 'nothing', 'zero', 'command'


class InvalidStatus(ValueError):
    """A 0x202 frame that is not a valid version 1 status frame."""


def decode_estop_status(can_id, data, *, extended=False, remote=False, error=False, fd=False):
    """Return None for other traffic and a dict for 0x202; raise for a malformed 0x202."""
    if extended or error or fd or can_id != ESTOP_STATUS_ID:
        return None
    if remote or len(data) != 8:
        raise InvalidStatus('0x202 must be a DLC 8 data frame')
    version, flags, trip, sequence, release, rejected = struct.unpack('<BBHHBB', bytes(data))
    if version != ESTOP_STATUS_VERSION:
        raise InvalidStatus(f'unsupported 0x202 version {version}')
    return {
        'raw': bool(flags & FLAG_RAW),
        'debounced': bool(flags & FLAG_DEBOUNCED),
        'latched': bool(flags & FLAG_LATCHED),
        'inhibit': bool(flags & FLAG_INHIBIT),
        'odrv_idle': bool(flags & FLAG_ODRV_IDLE),
        'bus_off': bool(flags & FLAG_BUS_OFF),
        'odrv_hb_ok': bool(flags & FLAG_ODRV_HB_OK),
        'drive_enabled': bool(flags & FLAG_DRIVE_ENABLED),
        'stopped': bool(flags & (FLAG_LATCHED | FLAG_INHIBIT)),
        'flags': flags,
        'trip_count': trip, 'sequence': sequence,
        'release_count': release, 'reset_rejected': rejected,
    }


def counter_went_back(old, new, bits):
    """A small backward step suggests an MCU reboot; a wrap is a large backward step."""
    back = (old - new) & ((1 << bits) - 1)
    return 0 < back < (1 << (bits - 1))


# Status frames are enqueued about every 100 ms (twice while latched), so far fewer
# than this many numbers pass between two frames that arrive within the timeout.
MAX_SEQUENCE_STEP = 64


def sequence_discontinuous(old, new):
    """True when 0x202 sequence went back or jumped: an STM32 reboot or lost status."""
    return ((new - old) & 0xFFFF) > MAX_SEQUENCE_STEP


class EstopSupervisor:
    """UNKNOWN/ESTOP -clear-> ARMING -gate open-> READY; any E-stop -> ESTOP."""

    def __init__(self, status_timeout=0.50, arming_warn=5.0):
        if not 0 < status_timeout < float('inf'):
            raise ValueError('status_timeout must be finite and positive')
        if not 0 < arming_warn < float('inf'):
            raise ValueError('arming_warn must be finite and positive')
        self.timeout = status_timeout
        self.arming_warn = arming_warn
        self.state = UNKNOWN
        self.reason = 'awaiting_status'
        self.status = None
        self.received_at = None
        self.zero_sent_at = None
        self.arming_since = None
        self.epoch = 0  # Increments on every E-stop event.
        self.reboot_suspected = False
        self.invalid = 0

    def trip(self, reason):
        if self.state != ESTOP:
            self.epoch += 1
        self.state = ESTOP
        self.reason = reason
        self.zero_sent_at = None
        self.arming_since = None

    def fresh(self, now):
        return self.received_at is not None and 0 <= now - self.received_at < self.timeout

    def check(self, now):
        if self.received_at is not None and not self.fresh(now) and self.state != ESTOP:
            self.trip('status_timeout')

    def receive_invalid(self, reason):
        self.invalid += 1
        self.trip(reason)

    def receive(self, status, now):
        previous = self.status
        if previous is not None:
            if (counter_went_back(previous['sequence'], status['sequence'], 16)
                    or counter_went_back(previous['trip_count'], status['trip_count'], 16)
                    or counter_went_back(previous['release_count'], status['release_count'], 8)):
                self.reboot_suspected = True
            if status['trip_count'] != previous['trip_count']:
                # A short press can trip and release between two status frames.
                self.trip('trip_count_changed')
            elif sequence_discontinuous(previous['sequence'], status['sequence']):
                # An IWDG reset can finish inside the status timeout with trip_count
                # still 0; a rebooted STM32 must re-arm before new commands.
                self.trip('status_discontinuity')
        self.status = status
        self.received_at = now
        if status['stopped']:
            if self.state != ESTOP or self.reason != 'stm32_stopped':
                self.trip('stm32_stopped')
        elif self.state in (UNKNOWN, ESTOP):
            self.state, self.reason = ARMING, 'released'
            self.zero_sent_at = None
            self.arming_since = now
        if self.state == ARMING:
            if (status['drive_enabled'] and self.zero_sent_at is not None
                    and now > self.zero_sent_at):
                self.state, self.reason = READY, 'drive_enabled'
                self.arming_since = None
        elif self.state == READY and not status['drive_enabled']:
            self.state, self.reason = ARMING, 'drive_gate_closed'
            self.zero_sent_at = None
            self.arming_since = now

    def arming_slow(self, now):
        """ARMING for longer than arming_warn: the ODrives are probably not ready."""
        return (self.state == ARMING and self.arming_since is not None
                and now - self.arming_since >= self.arming_warn)

    def output(self, now):
        self.check(now)
        if self.state == READY:
            return SEND_COMMAND
        if self.state == ARMING:
            return SEND_ZERO
        return SEND_NOTHING

    def zero_sent(self, now):
        if self.state == ARMING and self.zero_sent_at is None:
            self.zero_sent_at = now

    def report(self, now):
        status = self.status or {}
        return {
            'state': self.state, 'reason': self.reason, 'epoch': self.epoch,
            'status_received': self.status is not None,
            'status_age': None if self.received_at is None else round(now - self.received_at, 3),
            'status_fresh': self.fresh(now),
            'reboot_suspected': self.reboot_suspected, 'invalid_frames': self.invalid,
            'arming_for': (None if self.state != ARMING or self.arming_since is None
                           else round(now - self.arming_since, 3)),
            'arming_slow': self.arming_slow(now),
            'stm32': {key: status[key] for key in (
                'raw', 'debounced', 'latched', 'inhibit', 'odrv_idle', 'bus_off', 'odrv_hb_ok',
                'drive_enabled', 'stopped', 'trip_count', 'sequence', 'release_count',
                'reset_rejected') if key in status},
        }
