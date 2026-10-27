"""No ROS: hold duration never depends on a sender clock or link heartbeat."""
import pytest
from amr_control.operator_hold import OperatorHold


def test_hold_grace_release_and_rehold():
    hold = OperatorHold()
    assert not hold.blocking(100.)
    assert hold.receive(True, 100.)
    assert not hold.blocking(114.99)
    assert hold.blocking(115.)
    hold.receive({'held': True}, 120.)  # Duplicate does not restart the grace.
    assert hold.blocking(120.)
    hold.receive(False, 121.)
    assert not hold.blocking(121.)
    hold.receive('{"held": true}', 122.)
    assert not hold.blocking(136.99)
    assert hold.blocking(137.)
    assert hold.blocking(10000.)  # No link expiry.


@pytest.mark.parametrize('garbage', [None, {}, {'held': 1}, {'held': 'false'}, 'oops', '[]', 'null', 'true', 'false', 0])
def test_garbage_does_not_release_or_restart_hold(garbage):
    hold = OperatorHold(2.)
    assert not hold.receive(garbage, 0.)
    assert not hold.blocking(10.)
    hold.receive(True, 10.)
    assert not hold.receive(garbage, 11.)
    assert hold.blocking(12.)


@pytest.mark.parametrize('grace', [-1., float('nan'), float('inf')])
def test_invalid_grace(grace):
    with pytest.raises(ValueError):
        OperatorHold(grace)
