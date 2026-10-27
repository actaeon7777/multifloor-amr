"""Check the local executor helper without starting an executor or ROS node."""
import importlib.util
import inspect
from pathlib import Path
import sys
from types import ModuleType
from unittest.mock import Mock

import pytest
import rclpy.executors


@pytest.fixture
def teleop():
    path = (Path(__file__).resolve().parents[2] /
            'teleop_tools/joy_teleop/joy_teleop/joy_teleop.py')
    spec = importlib.util.spec_from_file_location('joy_teleop_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('available', [True, False])
def test_executor_selection(teleop, monkeypatch, available):
    experimental = ModuleType('rclpy.experimental')
    events, fallback = Mock(), Mock()
    if available:
        experimental.EventsExecutor = events
    monkeypatch.setitem(sys.modules, 'rclpy.experimental', experimental)
    monkeypatch.setattr(rclpy.executors, 'SingleThreadedExecutor', fallback)
    executor = teleop.spin_executor()
    chosen, unused = (events, fallback) if available else (fallback, events)
    assert executor is chosen.return_value
    chosen.assert_called_once_with()
    unused.assert_not_called()


def test_main_spins_selected_executor_with_single_argument_joy_callback(teleop, monkeypatch):
    assert list(inspect.signature(teleop.JoyTeleop.joy_callback).parameters) == ['self', 'msg']
    node, executor = Mock(), Mock()
    monkeypatch.setattr(teleop, 'JoyTeleop', Mock(return_value=node))
    monkeypatch.setattr(teleop, 'spin_executor', Mock(return_value=executor))
    monkeypatch.setattr(teleop.rclpy, 'init', Mock())
    monkeypatch.setattr(teleop.rclpy, 'shutdown', Mock())
    monkeypatch.setattr(teleop.rclpy, 'spin', Mock())
    teleop.main()
    executor.add_node.assert_called_once_with(node)
    executor.spin.assert_called_once_with()
    node.destroy_node.assert_called_once_with()
    teleop.rclpy.spin.assert_not_called()
