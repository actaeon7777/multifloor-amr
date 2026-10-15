import json

from amr_control.time_guard import ClockMonitor, HealthLease, SessionOrder


def test_real_clock_steps_both_directions_and_simulation_pause():
    monitor = ClockMonitor()
    assert not monitor.observe(100., 10.)
    assert not monitor.observe(100.1, 10.1)
    assert monitor.observe(99., 10.2)
    assert not monitor.observe(99.1, 10.3)
    assert monitor.observe(200., 10.4)
    monitor = ClockMonitor()
    assert not monitor.observe(300., 10., True)
    assert not monitor.observe(300., 12., True)
    assert not monitor.observe(310., 13., True)
    assert monitor.observe(1., 14., True)


def test_new_session_accepts_lower_generation_and_retires_old_writer():
    order = SessionOrder()
    assert order.accept('old', 999, 300.)
    assert order.accept('new', 0, 100.)
    assert not order.accept('old', 1000, 301.)
    assert not order.accept('new', 0, 99.)
    assert order.accept('new', 1, 90.)


def test_health_lease_expires_and_old_retained_status_cannot_reopen():
    lease = HealthLease()
    def status(session='a', sequence=1, stamp=100., healthy=True):
        return json.dumps(dict(session=session, sequence=sequence, stamp=stamp, epoch=0,
                               healthy=healthy, latched=False))
    assert lease.receive(status(), now=10., wall=100.)
    assert lease.healthy(10.5)
    assert not lease.healthy(10.61)
    assert not lease.receive(status(), now=11., wall=101.)
    assert not lease.healthy(11.)
    assert lease.receive(status('b', 0, 101.), now=11., wall=101.)
    assert not lease.receive(status('a', 9, 101.1), now=11.1, wall=101.1)
    assert lease.receive(status('b', 1, 101.2, False), now=11.2, wall=101.2)
    assert not lease.healthy(11.2)
    assert not lease.receive('{}', now=11.3, wall=101.3)
    assert lease.identity == ('b', 0)
