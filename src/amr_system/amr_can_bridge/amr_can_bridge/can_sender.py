"""Bounded SocketCAN writes; dry-run never imports CAN or opens a socket."""

from math import isfinite
from pathlib import Path
from time import sleep

from amr_can_bridge.drive_protocol import DRIVE_CAN_ID, STOP_PAYLOAD


NET_CLASS_PATH = Path('/sys/class/net')


class CanInterfaceError(RuntimeError):
    """The requested live interface is unavailable before opening the bus."""


def require_interface_up(channel):
    # SocketCAN can bind to a DOWN interface. Check IFF_UP before starting TX;
    # operstate is unsuitable because valid CAN interfaces may report UNKNOWN.
    if '/' in channel or channel in ('.', '..'):
        raise CanInterfaceError(f'Invalid CAN interface name: {channel!r}')
    try:
        flags = int((NET_CLASS_PATH / channel / 'flags').read_text().strip(), 16)
    except (OSError, ValueError) as error:
        raise CanInterfaceError(
            f'Cannot read CAN interface {channel!r}; check ip -details link show {channel}'
        ) from error
    if not flags & 0x1:  # Linux IFF_UP
        raise CanInterfaceError(
            f'CAN interface {channel} is DOWN. With the stack stopped, verify the '
            f'CAN configuration, run sudo ip link set {channel} up, then restart the stack.'
        )


class CanSender:
    def __init__(self, channel='can0', can_id=DRIVE_CAN_ID, dry_run=True,
                 send_timeout=0.01, receive_ids=(), bus_factory=None):
        if not isinstance(channel, str) or not channel.strip():
            raise ValueError('channel must be a nonempty string')
        if type(can_id) is not int or not 0 <= can_id <= 0x7ff:
            raise ValueError('can_id must be an 11-bit standard identifier')
        if type(dry_run) is not bool:
            raise ValueError('dry_run must be a boolean')
        if not isfinite(send_timeout) or send_timeout < 0:
            raise ValueError('send_timeout must be finite and non-negative')
        if any(type(cid) is not int or not 0 <= cid <= 0x7ff for cid in receive_ids):
            raise ValueError('receive_ids must be 11-bit standard identifiers')
        self.channel = channel
        self.can_id = can_id
        self.dry_run = dry_run
        self.send_timeout = send_timeout
        self.bus = None
        self.closed = False
        if not dry_run:
            if bus_factory is None:
                require_interface_up(channel)
            try:
                import can
                from can.interfaces.socketcan import SocketcanBus
            except ImportError as error:
                raise RuntimeError('Install python3-can to use real CAN transmission') from error
            self.can = can
            # Direct backend supports Ubuntu's python-can 3.x and newer versions,
            # and avoids implicitly selecting an interface from user config.
            factory = bus_factory if bus_factory is not None else SocketcanBus
            options = {}
            if receive_ids:
                options['can_filters'] = [{'can_id': cid, 'can_mask': 0x7ff, 'extended': False}
                                          for cid in receive_ids]
            self.bus = factory(channel=channel, receive_own_messages=False, fd=False, **options)

    def send(self, payload):
        if self.closed:
            raise RuntimeError('CAN sender is closed')
        if len(payload) != 8:
            raise ValueError('Drive commands must have exactly 8 payload bytes')
        if self.bus is not None:
            message = self.can.Message(
                arbitration_id=self.can_id, data=payload, is_extended_id=False,
                is_remote_frame=False, is_error_frame=False, is_fd=False, check=True,
            )
            self.bus.send(message, timeout=self.send_timeout)

    def receive(self, limit=50):
        """Drain up to limit filtered frames without blocking."""
        if self.closed:
            raise RuntimeError('CAN sender is closed')
        frames = []
        while self.bus is not None and len(frames) < limit:
            message = self.bus.recv(timeout=0.0)
            if message is None:
                break
            frames.append(message)
        return frames

    def close(self, count=5, period=0.05, on_stop=None, sleep_fn=sleep):
        """Best-effort repeated stops even if an earlier write fails, then close."""
        if self.closed:
            return []
        errors = []
        try:
            for index in range(count):
                try:
                    self.send(STOP_PAYLOAD)
                    if on_stop is not None:
                        on_stop()
                except Exception as error:
                    errors.append(error)
                if index + 1 < count:
                    sleep_fn(period)
        finally:
            self.closed = True
            if self.bus is not None:
                try:
                    self.bus.shutdown()
                except Exception as error:
                    errors.append(error)
        return errors
