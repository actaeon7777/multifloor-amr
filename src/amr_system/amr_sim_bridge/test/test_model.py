import math
from amr_sim_bridge.model import CommandTransport


def test_watchdog_uses_wall_time_and_reset_drops_pending():
    model = CommandTransport(delay=.1)
    model.receive(.4,.2,1.,10.)
    assert model.step(1.05,10.05)==(0.,0.)
    assert model.step(1.11,10.11)==(.4,.2)
    # Wall time continues even when physics slows down.
    assert model.step(1.12,10.31)==(0.,0.)
    model.receive(.4,0.,2.,11.)
    model.reset()
    assert model.step(3.,12.)==(0.,0.)


def test_forwarding_period_and_invalid_input():
    model = CommandTransport()
    model.receive(.1,0.,0.,0.)
    assert model.step(0.,0.)==(.1,0.)
    model.receive(.2,0.,.01,.01)
    assert model.step(.01,.01)==(.1,0.)
    assert model.step(.05,.05)==(.2,0.)
    assert not model.receive(math.nan,0.,.06,.06)
    assert model.step(.07,.07)==(0.,0.)
