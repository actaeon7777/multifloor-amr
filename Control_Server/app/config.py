"""Explicit host configuration with no ROS or system Python dependency."""
from dataclasses import dataclass
import os
from pathlib import Path

from amr_remote_bridge.common import identifier, positive, read_token
from amr_remote_bridge.server import find_assets


ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Settings:
    token_file: Path = Path.home() / '.config/amr_remote_bridge/token'
    assets_dir: Path = ROOT / 'vendor/amr_web_ui/web'
    fleet_dir: Path = ROOT / 'vendor/amr_remote_bridge/web'
    robot_id: str = 'amr-01'
    stale_after: float = 2
    command_timeout: float = 3
    lease_timeout: float = 3
    max_robots: int = 32
    body_timeout: float = 10

    @classmethod
    def from_env(cls):
        defaults = cls()
        return cls(
            token_file=Path(os.environ.get('AMR_TOKEN_FILE', defaults.token_file)).expanduser(),
            assets_dir=Path(os.environ.get('AMR_ASSETS_DIR', defaults.assets_dir)).expanduser().resolve(),
            fleet_dir=Path(os.environ.get('AMR_FLEET_DIR', defaults.fleet_dir)).expanduser().resolve(),
            robot_id=os.environ.get('AMR_ROBOT_ID', defaults.robot_id),
            stale_after=float(os.environ.get('AMR_STALE_AFTER', defaults.stale_after)),
            command_timeout=float(os.environ.get('AMR_COMMAND_TIMEOUT', defaults.command_timeout)),
            lease_timeout=float(os.environ.get('AMR_LEASE_TIMEOUT', defaults.lease_timeout)),
            max_robots=int(os.environ.get('AMR_MAX_ROBOTS', defaults.max_robots)),
            body_timeout=float(os.environ.get('AMR_BODY_TIMEOUT', defaults.body_timeout)),
        )

    def validate(self):
        identifier(self.robot_id)
        for value in (self.stale_after, self.command_timeout, self.lease_timeout, self.body_timeout):
            positive(value)
        if type(self.max_robots) is not int or self.max_robots < 1:
            raise ValueError('AMR_MAX_ROBOTS must be a positive integer')
        find_assets(self.assets_dir)
        for name in ('fleet.html', 'fleet.js', 'fleet.css'):
            if not (self.fleet_dir / name).is_file():
                raise ValueError(f'Missing fleet asset: {name}')
        return read_token(self.token_file)

