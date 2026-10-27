"""Kiosk logic and mocked HTTP/DOM checks; never starts a browser or ROS."""
from pathlib import Path
import shutil
import subprocess

import pytest


def test_kiosk_view():
    node = shutil.which('node')
    if not node:
        pytest.skip('Node is needed for kiosk JavaScript tests')
    script = Path(__file__).with_suffix('.js')
    result = subprocess.run([node, str(script)], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
