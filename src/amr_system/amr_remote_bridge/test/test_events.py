"""Event durability and retention, using temporary files only."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pytest
from amr_remote_bridge.events import EventStore
from amr_remote_bridge.common import MAX_EVENT_IMAGE, validate_event

JPEG = b'\xff\xd8\xff\xe0test image'


def event(n=1, **updates):
    return dict(id=f'{n:032x}', kind='security.motion', severity='alert', text='움직임 감지',
                floor='4F', room='401', pose=[1, 2, .3], image_path='/tmp/snapshot.jpg',
                stamp=123.5, wall_time=1700000000., extra={'source': 'camera'}) | updates


@pytest.fixture
def store(tmp_path):
    instance = EventStore(tmp_path)
    yield instance
    instance.close()


def test_insert_idempotent_and_reopen(tmp_path):
    store = EventStore(tmp_path)
    e = event()
    assert store.insert('amr-01', [e]) == {'accepted': [e['id']], 'want_images': [e['id']]}
    cursor = store.latest_seq()
    store.insert('amr-01', [event(text='duplicate must not overwrite')])
    assert store.latest_seq() == cursor
    assert store.query()['events'][0]['text'] == e['text']
    assert not store.query()['events'][0]['has_image']
    store.close()
    store = EventStore(tmp_path)
    assert store.latest_seq() == cursor and len(store.query()['events']) == 1
    store.close()


def test_query_order_after_robot_and_latest(store):
    assert store.latest_seq() == 0
    store.insert('amr-01', [event(1), event(2)])
    cursor = store.latest_seq()
    store.insert('amr-02', [event(3)])
    assert [e['id'] for e in store.query()['events']] == [event(n)['id'] for n in (3, 2, 1)]
    assert [e['id'] for e in store.query(after=cursor)['events']] == [event(3)['id']]
    assert len(store.query(limit=1)['events']) == 1
    assert len(store.query(robot='amr-01')['events']) == 2
    assert store.query(after=store.latest_seq())['events'] == []
    assert set(store.query()['events'][0]) == {'seq', 'id', 'robot_id', 'kind', 'severity',
        'text', 'floor', 'room', 'pose', 'stamp', 'wall_time', 'has_image'}
    for kwargs in ({'after': -1}, {'limit': 201}, {'limit': 0}, {'robot': '../robot'}):
        with pytest.raises(ValueError):
            store.query(**kwargs)


def test_image_unknown_duplicate_and_cursor(store, tmp_path):
    id = event()['id']
    with pytest.raises(KeyError):
        store.store_image(id, JPEG)
    with pytest.raises(ValueError):
        store.image('../image')
    store.insert('amr-01', [event()])
    cursor = store.latest_seq()
    with pytest.raises(KeyError):
        store.image(id)
    store.store_image(id, JPEG)
    assert store.image(id) == JPEG
    assert (tmp_path / 'images/amr-01' / (id + '.jpg')).read_bytes() == JPEG
    assert store.query(after=cursor)['events'][0]['has_image']
    cursor = store.latest_seq()
    store.store_image(id, JPEG + b'duplicate')
    assert store.latest_seq() == cursor and store.image(id) == JPEG
    assert store.insert('amr-01', [event()])['want_images'] == []


def test_prune_event_count_and_monotonic_cursor(tmp_path):
    store = EventStore(tmp_path, max_events=1)
    store.insert('amr-01', [event()])
    store.store_image(event()['id'], JPEG)
    old = store.latest_seq()
    store.insert('amr-01', [event(2, image_path=None)])
    assert [e['id'] for e in store.query()['events']] == [event(2)['id']]
    assert not list((tmp_path / 'images').rglob('*.jpg'))
    assert store.latest_seq() > old
    store.close()


def test_prune_image_bytes_oldest_and_zero_budget(tmp_path):
    store = EventStore(tmp_path, max_image_bytes=len(JPEG))
    store.insert('amr-01', [event(1), event(2)])
    # Image arrival order does not change which event is oldest for retention.
    store.store_image(event(2)['id'], JPEG)
    store.store_image(event(1)['id'], JPEG)
    assert [e['id'] for e in store.query()['events']] == [event(2)['id']]
    assert len(list((tmp_path / 'images').rglob('*.jpg'))) == 1
    cursor = store.latest_seq()
    store.close()
    store = EventStore(tmp_path, max_image_bytes=0)
    assert store.query()['events'] == [] and store.latest_seq() == cursor
    assert not list((tmp_path / 'images').rglob('*.jpg'))
    store.insert('amr-01', [event(3)])
    assert store.latest_seq() > cursor
    store.close()


def test_batch_validation_is_atomic_and_robot_collision(store):
    with pytest.raises(ValueError):
        store.insert('amr-01', [event(1), event(2, kind='invalid')])
    assert store.query()['events'] == []
    store.insert('amr-01', [event()])
    with pytest.raises(ValueError):
        store.insert('amr-02', [event(2), event(1)])
    assert len(store.query()['events']) == 1


@pytest.mark.parametrize('updates', [
    {'id': '../bad'}, {'kind': 'motion'}, {'kind': 'Security.motion'}, {'kind': 'security.1'},
    {'severity': []}, {'severity': 'critical'}, {'text': 'x'*1025}, {'text': None},
    {'floor': []}, {'room': True}, {'pose': [1, 2]}, {'pose': [True, 2, 3]},
    {'stamp': float('nan')}, {'wall_time': True}, {'extra': []}, {'extra': {'x': 'x'*8192}},
    {'image_path': 'relative.jpg'}, {'image_path': '/tmp/\x00.jpg'}, {'unknown': 1},
])
def test_invalid_event_fields(updates):
    with pytest.raises((ValueError, TypeError)):
        validate_event(event(**updates))


def test_invalid_image_size_magic_and_limits(store, tmp_path):
    store.insert('amr-01', [event()])
    for data in (b'', b'not jpeg', b'\xff\xd8', JPEG + b'x'*MAX_EVENT_IMAGE):
        with pytest.raises(ValueError):
            store.store_image(event()['id'], data)
    for kwargs in ({'max_events': 0}, {'max_events': True}, {'max_image_bytes': -1}):
        with pytest.raises(ValueError):
            EventStore(tmp_path, **kwargs)


def test_optional_extra_and_unknown_keys(store):
    e = event(kind='patrol.started')
    e.pop('extra')
    assert validate_event(e)['extra'] is None
    store.insert('amr-01', [e])
    assert store.db.execute('SELECT extra_json FROM events').fetchone()[0] == 'null'
    with pytest.raises(ValueError):
        store.insert('amr-01', [dict(e, unexpected=1)])


def test_wall_time_order_survives_late_image(store):
    store.insert('amr-01', [event(1, wall_time=20), event(2, wall_time=10)])
    cursor = store.latest_seq()
    store.store_image(event(2)['id'], JPEG)
    assert [e['id'] for e in store.query()['events']] == [event(1)['id'], event(2)['id']]
    assert [e['id'] for e in store.query(after=cursor)['events']] == [event(2)['id']]


def test_kind_prefix_filters_before_limit_and_literal_underscore(store):
    store.insert('amr-01', [event(1, wall_time=1)])
    store.insert('amr-01', [event(n, wall_time=n, kind='patrol.started') for n in range(2, 62)])
    assert all(e['kind'] == 'patrol.started' for e in store.query(limit=50)['events'])
    assert [e['id'] for e in store.query(kind_prefix='security.', limit=50)['events']] == [event(1)['id']]
    store.insert('amr-01', [event(62, kind='security_other.motion'), event(63, kind='securityxother.motion')])
    assert [e['id'] for e in store.query(kind_prefix='security_other.')['events']] == [event(62)['id']]
    assert store.query(robot='another', kind_prefix='security.')['events'] == []
