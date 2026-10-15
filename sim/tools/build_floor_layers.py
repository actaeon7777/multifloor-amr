#!/usr/bin/env python3
"""Generate every floor's USD map layer listed in config/floors.yaml.

Each floor's ``map`` + ``annotations`` is converted with the existing
generate_occupancy_usd.py into its ``sim_layer``.  Replacing a floor map is
therefore a config change followed by this command; no code changes.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / 'src/amr_system/amr_stack'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from amr_stack.floor_config import load_floor_config  # noqa: E402
from generate_occupancy_usd import generate_usda  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=PROJECT_ROOT / 'src/amr_system/amr_stack/config/floors.yaml')
    parser.add_argument('--floor', action='append', help='only this floor (repeatable)')
    arguments = parser.parse_args()
    config = load_floor_config(arguments.config)
    names = arguments.floor or sorted(config.floors, key=lambda n: config.floors[n].level)
    for name in names:
        floor = config.floor(name)
        map_yaml = (config.package_root / floor.map).resolve()
        annotations = (config.package_root / floor.annotations).resolve() if floor.annotations else None
        output = config.sim_layer_path(name, PROJECT_ROOT).resolve()
        stats = generate_usda(map_yaml, output, 3.0, 0.1, annotations, 1.0, 0.12, 2.0)
        print(f'{name}: {output.relative_to(PROJECT_ROOT)} '
              f'({stats["steel_doors"]} steel doors, {stats["elevator_doors"]} elevator doors)')


if __name__ == '__main__':
    main()
