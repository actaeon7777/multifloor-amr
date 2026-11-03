import pytest

pytest.importorskip('orbbec_camera_msgs')

from amr_stack.camera_settings import mismatches  # noqa: E402


def test_mismatches_reports_missing_and_different_values():
    target = {'exposure': 4167, 'gain': 16, 'white_balance': 5000, 'auto_white_balance': 0}
    assert mismatches(target, dict(target)) == []
    # Driver defaults right after start (2026-11-02 read-back).
    assert mismatches(target, {'exposure': 10000, 'gain': 101, 'white_balance': 5000,
                               'auto_white_balance': 0}) == ['exposure', 'gain']
    assert mismatches(target, {'exposure': 4167}) == ['gain', 'white_balance', 'auto_white_balance']
