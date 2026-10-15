"""Check kill selection and SIGKILL using disposable child processes only."""
from pathlib import Path
import signal
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import ros2kill


@pytest.mark.parametrize('argv,maps,selected', [
    (['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch', 'amr_stack'], '', True),
    (['/opt/ros/jazzy/lib/nav2_amcl/amcl', '--ros-args'], '', True),
    (['python3', '/workspace/sim/scripts/run_ros_sim.py'], '', True),
    (['/usr/bin/python3', 'custom_node.py'], '/opt/ros/jazzy/lib/librcl.so\n', True),
    (['/workspace/component_container'], '/opt/ros/jazzy/lib/librclcpp.so\n', True),
    (['/bin/bash', '-c', 'ros2 launch amr_stack experiment.launch.py'], '', False),
    (['python3', 'ordinary_program.py'], '/usr/lib/libpython3.12.so\n', False),
])
def test_ros_identification_ignores_shell_text_and_unrelated_python(argv, maps, selected):
    process = ros2kill.Process(100, 1, '0', argv)
    assert bool(ros2kill.ros_reason(process, maps)) == selected


def test_children_selected_but_terminal_and_unrelated_processes_preserved():
    def process(pid, parent, reason=''):
        return ros2kill.Process(pid, parent, '0', ['example'], reason)
    processes = {
        1: process(1, 0, 'ROS ancestor'),
        2: process(2, 1),
        3: process(3, 2, 'ROS launch'),
        4: process(4, 3),
        5: process(5, 4),
        6: process(6, 2),
    }
    assert set(ros2kill.select_targets(processes, {1, 2})) == {3, 4, 5}


@pytest.mark.parametrize('dry_run', [True, False])
def test_command_only_kills_selected_disposable_process(monkeypatch, dry_run):
    sleep = [sys.executable, '-c', 'import time; time.sleep(60)']
    node = subprocess.Popen(sleep + ['--ros-args'])
    unrelated = subprocess.Popen(sleep)
    try:
        # Restrict discovery to the two processes created by this test. Never
        # let a test discover or signal an existing ROS session on the host.
        paths = [Path('/proc') / str(p.pid) for p in (node, unrelated)]
        monkeypatch.setattr(Path, 'iterdir', lambda self: iter(paths))
        monkeypatch.setattr(sys, 'argv', ['ros2kill'] + (['--dry-run'] if dry_run else []))
        assert ros2kill.main() == 0
        if dry_run:
            assert node.poll() is None
        else:
            assert node.wait(timeout=5) == -signal.SIGKILL
        assert unrelated.poll() is None
    finally:
        for process in (node, unrelated):
            if process.poll() is None:
                process.kill()
            process.wait(timeout=5)
