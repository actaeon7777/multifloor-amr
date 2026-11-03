"""Durable event history, independent of robot leases and command delivery."""
from pathlib import Path
import re
import sqlite3
import threading
import time

from .common import dumps, loads, identifier, validate_event, event_id, MAX_EVENT_IMAGE


class EventStore:
    def __init__(self, data_dir, max_events=5000, max_image_bytes=1024**3):
        if type(max_events) is not int or max_events < 1:
            raise ValueError('max_events must be a positive integer')
        if type(max_image_bytes) is not int or max_image_bytes < 0:
            raise ValueError('max_image_bytes must be a nonnegative integer')
        self.data_dir = Path(data_dir).expanduser()
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.max_events, self.max_image_bytes = max_events, max_image_bytes
        self.lock = threading.RLock()
        self.db = sqlite3.connect(str(self.data_dir / 'events.sqlite'), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        with self.db:
            self.db.execute('''CREATE TABLE IF NOT EXISTS events (
                seq INTEGER PRIMARY KEY, id TEXT UNIQUE, robot_id TEXT, kind TEXT,
                severity TEXT, text TEXT, floor TEXT, room TEXT, pose_json TEXT,
                stamp REAL, wall_time REAL, received_at REAL, extra_json TEXT,
                image_path TEXT NULL, image_bytes INT)''')
            self.db.execute('CREATE TABLE IF NOT EXISTS event_clock (value INTEGER NOT NULL)')
            if not self.db.execute('SELECT value FROM event_clock').fetchone():
                self.db.execute('INSERT INTO event_clock SELECT COALESCE(MAX(seq), 0) FROM events')
        with self.lock, self.db:
            self._prune()

    def close(self):
        with self.lock:
            self.db.close()

    def _tick(self):
        self.db.execute('UPDATE event_clock SET value=value+1')
        return self.db.execute('SELECT value FROM event_clock').fetchone()[0]

    def latest_seq(self):
        with self.lock:
            return self.db.execute('SELECT value FROM event_clock').fetchone()[0]

    def _prune(self):
        count, size = self.db.execute(
            'SELECT COUNT(*), COALESCE(SUM(image_bytes), 0) FROM events').fetchone()
        while count > self.max_events or size > self.max_image_bytes:
            row = self.db.execute('SELECT * FROM events ORDER BY received_at, seq LIMIT 1').fetchone()
            if row['image_path']:
                (self.data_dir / row['image_path']).unlink(missing_ok=True)
            self.db.execute('DELETE FROM events WHERE id=?', (row['id'],))
            count -= 1
            size -= row['image_bytes']

    def insert(self, robot_id, events):
        robot_id = identifier(robot_id)
        events = [validate_event(event) for event in events]
        with self.lock, self.db:
            for event in events:
                old = self.db.execute('SELECT robot_id FROM events WHERE id=?', (event['id'],)).fetchone()
                if old and old['robot_id'] != robot_id:
                    raise ValueError('Event id belongs to another robot')
            for e in events:
                if self.db.execute('SELECT 1 FROM events WHERE id=?', (e['id'],)).fetchone():
                    continue
                self.db.execute('INSERT INTO events VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)', (
                    self._tick(), e['id'], robot_id, e['kind'], e['severity'], e['text'],
                    e['floor'], e['room'], dumps(e['pose']).decode(), e['stamp'], e['wall_time'],
                    time.time(), dumps(e['extra']).decode(), None, 0))
            self._prune()
            accepted = list(dict.fromkeys(e['id'] for e in events))
            wanted = []
            for e in events:
                row = self.db.execute('SELECT image_path FROM events WHERE id=?', (e['id'],)).fetchone()
                if e['image_path'] and row and row['image_path'] is None and e['id'] not in wanted:
                    wanted.append(e['id'])
            return dict(accepted=accepted, want_images=wanted)

    def store_image(self, id, body):
        event_id(id)
        if not isinstance(body, bytes) or not 3 <= len(body) <= MAX_EVENT_IMAGE or not body.startswith(b'\xff\xd8\xff'):
            raise ValueError('Expected JPEG up to 400 KiB')
        with self.lock, self.db:
            row = self.db.execute('SELECT * FROM events WHERE id=?', (id,)).fetchone()
            if row is None:
                raise KeyError(id)
            if row['image_path']:
                return
            relative = Path('images') / identifier(row['robot_id']) / (id + '.jpg')
            target = self.data_dir / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_suffix('.tmp')
            temporary.write_bytes(body)
            temporary.replace(target)
            # Refresh the cursor so polling clients discover a delayed thumbnail.
            self.db.execute('UPDATE events SET image_path=?, image_bytes=?, seq=? WHERE id=?',
                            (str(relative), len(body), self._tick(), id))
            self._prune()

    def image(self, id):
        event_id(id)
        with self.lock:
            row = self.db.execute('SELECT image_path FROM events WHERE id=?', (id,)).fetchone()
            if row is None or row['image_path'] is None:
                raise KeyError(id)
            try:
                return (self.data_dir / row['image_path']).read_bytes()
            except FileNotFoundError:
                raise KeyError(id) from None

    def query(self, after=0, limit=50, robot=None, kind_prefix=None):
        if type(after) is not int or after < 0 or type(limit) is not int or not 1 <= limit <= 200:
            raise ValueError('Invalid event cursor or limit')
        if robot is not None:
            identifier(robot)
        conditions, params = ['seq > ?'], [after]
        if robot is not None:
            conditions.append('robot_id=?')
            params.append(robot)
        if kind_prefix is not None:
            if (not isinstance(kind_prefix, str) or len(kind_prefix) > 128 or
                    not re.fullmatch(r'[a-z_]+(?:\.[a-z_]+)*\.', kind_prefix)):
                raise ValueError('Invalid event kind_prefix')
            conditions.append('substr(kind, 1, ?)=?')
            params.extend((len(kind_prefix), kind_prefix))
        with self.lock:
            rows = self.db.execute('SELECT * FROM events WHERE ' + ' AND '.join(conditions) +
                ' ORDER BY wall_time DESC, id DESC LIMIT ?', (*params, limit)).fetchall()
            events = []
            for row in rows:
                e = {key: row[key] for key in ('seq', 'id', 'robot_id', 'kind', 'severity', 'text',
                                              'floor', 'room', 'stamp', 'wall_time')}
                e.update(pose=loads(row['pose_json']), has_image=row['image_path'] is not None)
                events.append(e)
            return dict(events=events, latest_seq=self.latest_seq())
