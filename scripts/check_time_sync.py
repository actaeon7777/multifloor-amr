#!/usr/bin/env python3
"""Read chrony's synchronization state before multi-computer real operation."""
import argparse
import math
import re
import subprocess


def check_tracking(text, max_offset):
    fields = dict(line.split(':', 1) for line in text.splitlines() if ':' in line)
    fields = {key.strip(): value.strip() for key, value in fields.items()}
    if fields.get('Leap status') != 'Normal':
        raise ValueError('chrony has not synchronized its clock')
    match = re.match(r'([0-9.eE+-]+) seconds (?:slow|fast)', fields.get('System time', ''))
    if not match or not math.isfinite(float(match[1])) or abs(float(match[1])) > max_offset:
        raise ValueError(f'System clock correction exceeds {max_offset:g} seconds or is unavailable')
    return float(match[1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-offset', type=float, default=.05)
    args = parser.parse_args()
    if not math.isfinite(args.max_offset) or args.max_offset <= 0:
        parser.error('--max-offset must be positive and finite')
    try:
        result = subprocess.run(['chronyc', '-n', 'tracking'], capture_output=True, text=True,
                                check=True, timeout=3, env={'PATH': '/usr/bin:/usr/sbin:/bin', 'LC_ALL': 'C'})
        offset = check_tracking(result.stdout, args.max_offset)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        raise SystemExit(f'Time synchronization preflight failed: {error}. Check chrony before starting real mode.')
    print(f'Time synchronization preflight passed: reported correction {offset:g} s')


if __name__ == '__main__':
    main()
