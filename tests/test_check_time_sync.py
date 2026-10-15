import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('clock_preflight', Path(__file__).resolve().parents[1]/'scripts/check_time_sync.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('offset,direction', [('.001', 'slow'), ('.002', 'fast')])
def test_accepts_synced_clock(offset, direction):
    assert module.check_tracking(f'System time : {offset} seconds {direction} of NTP time\nLeap status : Normal', .05) == float(offset)


@pytest.mark.parametrize('text', ['Leap status : Not synchronised',
    'System time : 2 seconds slow of NTP time\nLeap status : Normal',
    'System time : nan seconds slow of NTP time\nLeap status : Normal', ''])
def test_rejects_unsynced_large_offset_or_missing_measurement(text):
    with pytest.raises(ValueError):
        module.check_tracking(text, .05)
