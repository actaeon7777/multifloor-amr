"""Read only complete spool records; never remove the detector's images."""
import heapq
import logging
from pathlib import Path
import re

from .common import MAX_EVENT, loads, validate_event

LOG = logging.getLogger(__name__)
NAME = re.compile(r'\d{13}-([0-9a-f]{32})\.json')


def quarantine(path, reason):
    """Best effort: queue retention may remove a file while we inspect it."""
    try:
        bad = path.parent / 'bad'
        bad.mkdir(exist_ok=True)
        path.replace(bad / path.name)
    except FileNotFoundError:
        return
    except OSError as exc:
        LOG.warning('Could not quarantine spool file %s (%s): %s', path.name, reason, exc)
    else:
        LOG.warning('Quarantined spool file %s: %s', path.name, reason)


def read_spool(directory, limit=20):
    directory = Path(directory).expanduser()
    try:
        paths = heapq.nsmallest(min(20, max(0, limit)), directory.glob('*.json'))
    except FileNotFoundError:
        return []
    except OSError as exc:
        LOG.warning('Could not list spool directory %s: %s', directory, exc)
        return []
    pending = []
    for path in paths:
        try:
            match = NAME.fullmatch(path.name)
            if not match or path.is_symlink():
                raise ValueError('Invalid spool filename')
            with path.open('rb') as stream:
                body = stream.read(MAX_EVENT + 1)
            if len(body) > MAX_EVENT:
                raise ValueError('Event exceeds 64 KiB')
            event = validate_event(loads(body))
            if event['id'] != match[1]:
                raise ValueError('Spool filename/id mismatch')
            pending.append((path, event))
        except FileNotFoundError:
            continue
        except (ValueError, TypeError, RecursionError, OSError) as exc:
            quarantine(path, exc)
    return pending
