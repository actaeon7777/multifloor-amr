"""Shared helpers for opt-in ROS tests; never initialize ROS during import."""
import os
import signal
from threading import Timer
from time import monotonic

import rclpy


def assert_test_domain(*legacy_domains):
    assert os.environ['ROS_DOMAIN_ID'] in ('150', *legacy_domains)
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'


def run_main_sigterm(monkeypatch, module, class_name, factory, args=None):
    """Run main in the test's main thread; measure from SIGTERM through cleanup."""
    old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    nodes, sent_at, timers, closed_alive = [], [], [], []

    def terminate():
        sent_at.append(monotonic())
        os.kill(os.getpid(), signal.SIGTERM)

    def construct():
        node = factory()
        nodes.append(node)
        close = node.close

        def close_with_live_context():
            closed_alive.append(rclpy.ok(context=node.context))
            close()

        monkeypatch.setattr(node, 'close', close_with_live_context)
        timer = Timer(.1, terminate)
        timers.append(timer)
        timer.start()
        return node

    def deadline(*_):
        raise AssertionError('main did not return after SIGTERM (3 s watchdog)')

    monkeypatch.setattr(module, class_name, construct)
    old_alarm = signal.signal(signal.SIGALRM, deadline)
    signal.setitimer(signal.ITIMER_REAL, 3.)
    try:
        module.main(args=[] if args is None else args)
        returned_at = monotonic()
        assert len(sent_at) == 1
        assert returned_at - sent_at[0] < .5
        assert closed_alive == [True]
        assert not rclpy.ok()
        assert all(signal.getsignal(sig) == old for sig, old in old_handlers.items())
        return nodes[0]
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.)
        signal.signal(signal.SIGALRM, old_alarm)
        for timer in timers:
            timer.cancel()
            timer.join()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        if rclpy.ok():
            rclpy.shutdown()
