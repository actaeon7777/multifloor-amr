"""Exercise CAN framing and error cleanup without any physical CAN interface."""

import builtins

import pytest

from amr_can_bridge.can_sender import CanSender
from amr_can_bridge.drive_protocol import make_payload


class FakeBus:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.messages = []
        self.is_shutdown = False
        self.fail = False

    def send(self, message, timeout):
        self.messages.append((message, timeout))
        if self.fail:
            raise OSError('simulated disconnected CAN')

    def shutdown(self):
        self.is_shutdown = True


def test_dry_run_never_imports_can_or_opens_bus(monkeypatch):
    original_import = builtins.__import__

    def guard(name, *args, **kwargs):
        if name == 'can' or name.startswith('can.'):
            pytest.fail('dry_run imported CAN')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', guard)
    sender = CanSender(dry_run=True, bus_factory=lambda **_: pytest.fail('bus opened'))
    sender.send(make_payload(0.2, 0.4))
    stops = []
    sender.close(on_stop=lambda: stops.append(1), sleep_fn=lambda _: None)
    assert stops == [1] * 5
    assert sender.bus is None


def test_standard_frame_and_shutdown_stops():
    pytest.importorskip('can')
    bus = FakeBus()
    sender = CanSender(dry_run=False, bus_factory=lambda **kwargs: bus)
    sender.send(make_payload(0.2, 0.4))
    msg, timeout = bus.messages[0]
    assert msg.arbitration_id == 0x201
    assert msg.dlc == 8
    assert not msg.is_extended_id and not msg.is_remote_frame and not msg.is_fd
    assert bytes(msg.data).hex() == 'cdcc4c3ecdcccc3e'
    assert timeout == 0.01
    assert sender.close(sleep_fn=lambda _: None) == []
    assert len(bus.messages) == 6
    assert all(bytes(msg.data) == bytes(8) for msg, _ in bus.messages[1:])
    assert bus.is_shutdown
    sender.close(sleep_fn=lambda _: None)  # Idempotent.
    assert len(bus.messages) == 6


def test_shutdown_attempts_all_stops_and_closes_on_send_error():
    pytest.importorskip('can')
    bus = FakeBus()
    bus.fail = True
    sender = CanSender(dry_run=False, bus_factory=lambda **kwargs: bus)
    errors = sender.close(sleep_fn=lambda _: None)
    assert len(errors) == 5
    assert len(bus.messages) == 5
    assert bus.is_shutdown


@pytest.mark.parametrize('kwargs', [
    {'can_id': -1}, {'can_id': 0x800}, {'channel': ''},
    {'dry_run': 'false'}, {'send_timeout': -1},
])
def test_bad_sender_configuration(kwargs):
    with pytest.raises(ValueError):
        CanSender(**kwargs)


@pytest.mark.parametrize('flags', [None, '0x280', 'invalid'])
def test_live_start_rejects_unavailable_interface_before_can_import(
        monkeypatch, tmp_path, flags):
    from amr_can_bridge import can_sender
    monkeypatch.setattr(can_sender, 'NET_CLASS_PATH', tmp_path)
    interface = tmp_path / 'can0'
    interface.mkdir()
    if flags is not None:
        (interface / 'flags').write_text(flags)
    original_import = builtins.__import__

    def guard(name, *args, **kwargs):
        if name == 'can' or name.startswith('can.'):
            pytest.fail('unavailable interface reached CAN backend')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', guard)
    with pytest.raises(can_sender.CanInterfaceError) as error:
        CanSender(dry_run=False)
    if flags == '0x280':
        assert 'sudo ip link set can0 up' in str(error.value)
    # Dry run still works even with missing, DOWN, or unreadable flags.
    CanSender(dry_run=True)


def test_up_interface_with_unknown_operstate_is_accepted(monkeypatch, tmp_path):
    from amr_can_bridge import can_sender
    monkeypatch.setattr(can_sender, 'NET_CLASS_PATH', tmp_path)
    interface = tmp_path / 'can0'
    interface.mkdir()
    (interface / 'flags').write_text('0x281')
    (interface / 'operstate').write_text('unknown')
    can_sender.require_interface_up('can0')
