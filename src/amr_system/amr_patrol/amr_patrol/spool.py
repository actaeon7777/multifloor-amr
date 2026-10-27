"""Atomic, ROS-independent event queue. Only the uploader consumes queued events."""
import json
import math
import os
from pathlib import Path
import re
import time
import warnings


QUEUE_NAME = re.compile(r'\d{13}-[0-9a-f]{32}\.json\Z')


class EventSpool:
    def __init__(self, spool_dir=None, image_dir=None, spool_max_files=2000,
                 image_keep_days=7.0, warn=None):
        if type(spool_max_files) is not int or spool_max_files < 1:
            raise ValueError('spool_max_files must be a positive integer')
        if (type(image_keep_days) not in (int, float) or not math.isfinite(image_keep_days)
                or image_keep_days <= 0):
            raise ValueError('image_keep_days must be positive and finite')
        self.spool_dir = Path(spool_dir or Path.home() / 'amr_data/events/queue').expanduser().resolve()
        self.image_dir = Path(image_dir or Path.home() / 'amr_data/security').expanduser().resolve()
        self.spool_max_files, self.image_keep_days = spool_max_files, image_keep_days
        self.warn = warn or (lambda message: warnings.warn(message, RuntimeWarning))
        self.spool_dir.mkdir(parents=True, exist_ok=True)
        self.image_dir.mkdir(parents=True, exist_ok=True)

    def write(self, event, wall_time=None):
        if not isinstance(event, dict) or not isinstance(event.get('id'), str) or re.fullmatch(
                r'[0-9a-f]{32}', event['id']) is None:
            raise ValueError('event id must be 32 lowercase hex characters')
        required = {'kind', 'severity', 'text', 'floor', 'room', 'pose', 'image_path', 'stamp'}
        if not required <= event.keys() or event['severity'] not in ('info', 'warn', 'alert'):
            raise ValueError('invalid event envelope')
        payload = dict(event)
        payload.setdefault('extra', None)
        payload.setdefault('wall_time', time.time() if wall_time is None else wall_time)
        timestamp = payload['wall_time']
        if (type(timestamp) not in (int, float) or not math.isfinite(timestamp)
                or timestamp < 0 or timestamp >= 1e10):
            raise ValueError('wall_time must be a finite Unix timestamp')
        text = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        name = f'{int(timestamp * 1000):013d}-{payload["id"]}.json'
        destination = self.spool_dir / name
        temporary = self.spool_dir / (name + '.tmp')
        for attempt in range(2):
            try:
                with temporary.open('w', encoding='utf-8') as stream:
                    stream.write(text)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, destination)
                break
            except FileNotFoundError:
                if attempt or self.spool_dir.is_dir():
                    raise
                self.spool_dir.mkdir(parents=True, exist_ok=True)
            finally:
                temporary.unlink(missing_ok=True)
        self.enforce_bound()
        return destination

    def enforce_bound(self):
        files = sorted(path for path in self.spool_dir.glob('*.json')
                       if QUEUE_NAME.fullmatch(path.name))
        for path in files[:max(0, len(files) - self.spool_max_files)]:
            try:
                path.unlink()
            except FileNotFoundError:  # Uploader may have consumed it concurrently.
                continue
            self.warn(f'event spool limit {self.spool_max_files}: dropped oldest {path.name}')

    def prune_images(self, wall_time=None):
        referenced = set()
        for queued in self.spool_dir.glob('*.json'):
            try:
                payload = json.loads(queued.read_text(encoding='utf-8'))
                image_path = payload.get('image_path')
                if image_path is not None:
                    referenced.add(Path(image_path).expanduser().resolve())
            except FileNotFoundError:  # Consumed by uploader during the scan.
                continue
            except (OSError, ValueError, TypeError, AttributeError) as error:
                self.warn(f'image pruning skipped: cannot read queued references: {error}')
                return  # Preserve images if the queue cannot be fully inspected.
        cutoff = (time.time() if wall_time is None else wall_time) - self.image_keep_days * 86400
        for path in self.image_dir.glob('*.jpg'):
            try:
                if (not path.is_symlink() and path.resolve() not in referenced
                        and path.stat().st_mtime < cutoff):
                    path.unlink()
            except FileNotFoundError:
                pass
