"""Executor selection without constructing ROS nodes, contexts or transports."""
import sys
from types import ModuleType
from unittest.mock import Mock

import pytest

from amr_can_bridge import can_telemetry, cmd_vel_to_can, elevator_can_link


@pytest.mark.parametrize('module', [cmd_vel_to_can, can_telemetry, elevator_can_link])
@pytest.mark.parametrize('available', [True, False])
def test_spin_executor_selects_events_when_importable(monkeypatch, module, available):
    experimental = ModuleType('rclpy.experimental')
    events, fallback = Mock(), Mock()
    if available:
        experimental.EventsExecutor = events
    monkeypatch.setitem(sys.modules, 'rclpy.experimental', experimental)
    monkeypatch.setattr('rclpy.executors.SingleThreadedExecutor', fallback)
    expected, unused = (events, fallback) if available else (fallback, events)
    assert module.spin_executor() is expected.return_value
    expected.assert_called_once_with()
    unused.assert_not_called()
