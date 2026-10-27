"""Configuration for the dependency-free central console."""
from dataclasses import dataclass
import os
from pathlib import Path

from amr_remote_bridge.common import identifier, positive, read_token


@dataclass(frozen=True)
class Settings:
    token_file: Path = Path.home() / '.config/amr_remote_bridge/token'
    slack_env: Path = Path.home() / '.config/amr_remote_bridge/slack.env'
    slack_rooms: Path = Path.home() / '.config/amr_remote_bridge/slack_rooms.json'
    data_dir: Path = Path(__file__).resolve().parents[1] / 'data'
    max_events: int = 5000
    max_image_bytes: int = 1024**3
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
            data_dir=Path(os.environ.get('AMR_DATA_DIR', defaults.data_dir)).expanduser(),
            max_events=int(os.environ.get('AMR_MAX_EVENTS', defaults.max_events)),
            max_image_bytes=int(os.environ.get('AMR_MAX_IMAGE_BYTES', defaults.max_image_bytes)),
            token_file=Path(os.environ.get('AMR_TOKEN_FILE', defaults.token_file)).expanduser(),
            slack_env=Path(os.environ.get('AMR_SLACK_ENV', defaults.slack_env)).expanduser(),
            slack_rooms=Path(os.environ.get('AMR_SLACK_ROOMS', defaults.slack_rooms)).expanduser(),
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
        if type(self.max_events) is not int or self.max_events < 1:
            raise ValueError('AMR_MAX_EVENTS must be a positive integer')
        if type(self.max_image_bytes) is not int or self.max_image_bytes < 0:
            raise ValueError('AMR_MAX_IMAGE_BYTES must be a nonnegative integer')
        return read_token(self.token_file)
