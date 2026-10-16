"""Wheel feasibility and the actual STM32 conversion, without ROS or hardware."""

import ctypes
from math import pi
from pathlib import Path
import re
import shutil
import subprocess

import pytest

from amr_control.control_gate import ControlGate
from amr_control.wheel_limiter import WheelSpeedLimiter


@pytest.mark.parametrize('mode', ['MANUAL', 'AUTO'])
@pytest.mark.parametrize('v,w', [(1.0, 0.1), (1.0, 0.8), (-1.0, 0.8),
                                 (1.0, -0.8), (-1.0, -0.8)])
def test_turn_keeps_curvature_in_both_modes_and_straight_recovers(mode, v, w):
    gate = ControlGate(max_linear_speed=1.0, max_angular_speed=0.8)
    gate.receive_joy([0.0] * 3, [0] * 8, 0.0)
    gate.receive_joy([0.0] * 3, [0] * 6 + [int(mode == 'MANUAL'), int(mode == 'AUTO')], 0.12)
    receive = gate.receive_command if mode == 'MANUAL' else gate.receive_auto_command
    receive((v, 0, 0, 0, 0, w), 0.13)
    if mode == 'AUTO':
        assert gate.output(0.15) == (0.0, 0.0)
    limited_v, limited_w = gate.output(0.23)
    assert limited_w / limited_v == pytest.approx(w / v)
    assert max(abs(limited_v + limited_w * 0.2645),
               abs(limited_v - limited_w * 0.2645)) == pytest.approx(pi * 0.172 * 1.8)
    assert 0 < limited_v / v < 1
    assert 0 < limited_w / w < 1
    receive((v, 0, 0, 0, 0, 0), 0.24)
    assert abs(gate.output(0.25)[0]) == pytest.approx(pi * 0.172 * 1.8)
    assert gate.output(0.25)[1] == 0
    assert gate.output(0.70) == (0.0, 0.0)  # Joy timeout must still stop.


def test_small_input_is_unchanged_and_oversized_rotation_is_bounded():
    limiter = WheelSpeedLimiter()
    assert limiter.limit(0.4, 0.8) == (0.4, 0.8)
    assert limiter.limit(0.0, 0.0) == (0.0, 0.0)
    for sign in (-1, 1):
        v, w = limiter.limit(0.0, sign * 100.0)
        assert v == pytest.approx(0.0, abs=1e-12)
        assert w == pytest.approx(sign * 2 * pi * 0.172 * 1.8 / 0.529)


@pytest.mark.parametrize('geometry', [(0.172, 0.529, 1.8), (0.2, 0.5, 0.8)])
def test_all_quadrants_respect_both_wheel_limits(geometry):
    diameter, track, rev_limit = geometry
    limiter = WheelSpeedLimiter(*geometry)
    for v in (-2, -1, -0.4, 0, 0.4, 1, 2):
        for w in (-10, -0.8, -0.1, 0, 0.1, 0.8, 10):
            out_v, out_w = limiter.limit(v, w)
            for wheel_mps in (out_v + out_w * track / 2, out_v - out_w * track / 2):
                assert abs(wheel_mps / (pi * diameter)) <= rev_limit + 1e-12
            assert abs(out_v) <= abs(v) + 1e-12
            assert abs(out_w) <= abs(w) + 1e-12
            assert out_v * w == pytest.approx(out_w * v, abs=1e-12)
            assert limiter.limit(out_v, out_w) == pytest.approx((out_v, out_w), abs=1e-12)


@pytest.mark.parametrize('name', ['wheel_diameter', 'track_width', 'max_wheel_rev_s'])
@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_invalid_geometry_is_rejected(name, value):
    with pytest.raises(ValueError):
        ControlGate(**{name: value})


@pytest.fixture(scope='module')
def stm32_convert(tmp_path_factory):
    """Compile the real conversion functions, not a Python copy of the firmware."""
    compiler = shutil.which('gcc')
    if compiler is None:
        pytest.skip('Host gcc is required for STM32 conversion checks')
    source = (Path(__file__).resolve().parents[4] / 'firmware/stm32/main.c').read_text()

    def function(name):
        match = re.search(r'static (?:float|void) ' + name + r'\([^;]*?\)\s*\{', source)
        assert match is not None
        end, depth = match.end(), 1
        while depth:
            depth += (source[end] == '{') - (source[end] == '}')
            end += 1
        return source[match.start():end]

    functions = '\n'.join(function(name) for name in (
        'Drive_GetWheelCircumference', 'ODrive_UpdateTargetFromBodyVelocity'))
    names = set(re.findall(r'\b(?:drive_\w+|ik_\w+|odrv[01]_target_vel)\b', functions))
    declarations = []
    for name in sorted(names):
        declaration = re.search(r'^volatile float\s+' + name + r'\s*=.*?;', source, re.M)
        assert declaration is not None, name
        declarations.append(declaration.group())
    defines = '\n'.join(re.findall(r'^#define DRIVE_.*$', source, re.M))
    wrapper = '''
void convert(float v, float w, float *out) {
    ODrive_UpdateTargetFromBodyVelocity(v, w);
    out[0] = odrv0_target_vel;
    out[1] = odrv1_target_vel;
}
'''
    folder = tmp_path_factory.mktemp('stm32_kinematics')
    c_file = folder / 'kinematics.c'
    c_file.write_text(defines + '\n' + '\n'.join(declarations) + '\n' + functions + wrapper)
    library = folder / 'kinematics.so'
    subprocess.run([compiler, '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror',
                    str(c_file), '-o', str(library)], check=True, capture_output=True)
    lib = ctypes.CDLL(str(library))
    lib.convert.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
    lib.convert.restype = None

    def convert(v, w):
        output = (ctypes.c_float * 2)()
        lib.convert(v, w, output)
        return tuple(output)

    return convert


def test_stm32_converts_without_hidden_speed_limit(stm32_convert):
    # Deliberately exceed the old 1.8 rev/s limit in the host harness only.
    for v, w in [(2.0, 0.8), (-2.0, -0.8), (0.0, 8.0)]:
        right, left_motor = stm32_convert(v, w)
        assert right == pytest.approx((v + 0.2645 * w) / (pi * 0.172), abs=1e-6)
        assert left_motor == pytest.approx(-(v - 0.2645 * w) / (pi * 0.172), abs=1e-6)


def test_upper_output_stays_bounded_after_real_stm32_conversion(stm32_convert):
    limiter = WheelSpeedLimiter()
    for v in (-1.0, 0.0, 1.0):
        for w in (-0.8, -0.1, 0.0, 0.1, 0.8):
            target_v, target_w = limiter.limit(v, w)
            right, left_motor = stm32_convert(target_v, target_w)
            assert max(abs(right), abs(left_motor)) <= 1.8 + 1e-6
            right_mps, left_mps = right * pi * 0.172, -left_motor * pi * 0.172
            assert (right_mps + left_mps) / 2 == pytest.approx(target_v, abs=1e-6)
            assert (right_mps - left_mps) / 0.529 == pytest.approx(target_w, abs=1e-6)
