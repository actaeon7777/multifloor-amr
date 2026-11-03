import json
import os
from uuid import uuid4

import pytest

from amr_patrol.spool import EventSpool, QUEUE_NAME


def event(**changes):
    return dict(dict(id=uuid4().hex, kind='security.motion', severity='alert',
                     text='4F에서 움직임 감지', floor='4F', room='', pose=[1., 2., 0.],
                     image_path=None, stamp=123., extra={'cells': 100}), **changes)


def spool(tmp_path, **kwargs):
    return EventSpool(tmp_path / 'queue', tmp_path / 'images', **kwargs)


def test_atomic_write_adds_wall_time_and_preserves_event(tmp_path, monkeypatch):
    subject = spool(tmp_path)
    original = event()
    real_replace = os.replace
    replaced = []

    def replace(source, destination):
        assert source.parent == destination.parent == subject.spool_dir
        assert source.name == destination.name + '.tmp'
        assert not destination.exists()
        payload = json.loads(source.read_text())
        assert payload == dict(original, wall_time=1700000000.125)
        replaced.append(destination)
        real_replace(source, destination)

    monkeypatch.setattr(os, 'replace', replace)
    result = subject.write(original, wall_time=1700000000.125)
    assert replaced == [result]
    assert QUEUE_NAME.fullmatch(result.name)
    assert result.name == f'1700000000125-{original["id"]}.json'
    assert not list(subject.spool_dir.glob('*.tmp'))
    assert 'wall_time' not in original


def test_existing_wall_time_preserved_and_same_event_path(tmp_path):
    subject = spool(tmp_path)
    payload = event(wall_time=1700000000.125)
    path = subject.write(payload)
    assert subject.write(payload) == path
    assert json.loads(path.read_text()) == payload
    assert len(list(subject.spool_dir.glob('*.json'))) == 1


def test_bounded_queue_warns_and_drops_oldest_not_by_mtime(tmp_path):
    warnings = []
    subject = spool(tmp_path, spool_max_files=2, warn=warnings.append)
    newest = subject.write(event(), wall_time=30)
    oldest = subject.write(event(), wall_time=10)
    middle = subject.write(event(kind='patrol.finished', severity='info'), wall_time=20)
    assert not oldest.exists()
    assert middle.exists() and newest.exists()
    assert len(warnings) == 1 and oldest.name in warnings[0]


def test_image_pruning_uses_mtime_and_never_prunes_queue(tmp_path, monkeypatch):
    subject = spool(tmp_path)
    old, fresh = subject.image_dir / 'old.jpg', subject.image_dir / 'fresh.jpg'
    old.write_bytes(b'old')
    fresh.write_bytes(b'fresh')
    path = subject.write(event(), wall_time=1)
    original_stat = type(old).stat

    def stat(path, *args, **kwargs):
        value = original_stat(path, *args, **kwargs)
        fields = list(value)
        if path == old:
            fields[8] = 1.0
        elif path == fresh:
            fields[8] = 864000.0
        return os.stat_result(fields)

    monkeypatch.setattr(type(old), 'stat', stat)
    subject.prune_images(wall_time=864000)
    assert not old.exists() and fresh.exists() and path.exists()


def test_failed_replace_cleans_temporary_file(tmp_path, monkeypatch):
    subject = spool(tmp_path)

    def fail(*_):
        raise OSError('disk failed')

    monkeypatch.setattr(os, 'replace', fail)
    with pytest.raises(OSError, match='disk failed'):
        subject.write(event())
    assert list(subject.spool_dir.iterdir()) == []


@pytest.mark.parametrize('changes', [
    {'id': '../bad'}, {'id': 'z' * 32}, {'severity': 'bad'},
    {'wall_time': float('nan')}, {'stamp': float('inf')},
])
def test_bad_event_does_not_create_file(tmp_path, changes):
    subject = spool(tmp_path)
    with pytest.raises(ValueError):
        subject.write(event(**changes))
    assert list(subject.spool_dir.iterdir()) == []


@pytest.mark.parametrize('params', [{'spool_max_files': 0}, {'spool_max_files': True},
                                   {'image_keep_days': -1}, {'image_keep_days': float('nan')}])
def test_invalid_limits(tmp_path, params):
    with pytest.raises(ValueError):
        spool(tmp_path, **params)


def test_extra_is_present_for_patrol_event(tmp_path):
    subject = spool(tmp_path)
    payload = event(kind='patrol.finished', severity='info')
    del payload['extra']
    saved = json.loads(subject.write(payload).read_text())
    assert saved['extra'] is None
    assert 'extra' not in payload


def test_referenced_image_kept_until_queue_consumed(tmp_path):
    subject = spool(tmp_path)
    image = subject.image_dir / 'old.jpg'
    image.write_bytes(b'jpeg')
    os.utime(image, (1., 1.))
    queued = subject.write(event(image_path=str(image)), wall_time=1.)
    subject.prune_images(wall_time=864000.)
    assert image.exists()
    queued.unlink()  # Simulated uploader consumption.
    subject.prune_images(wall_time=864000.)
    assert not image.exists()


def test_unreadable_queue_prevents_image_deletion(tmp_path):
    warnings = []
    subject = spool(tmp_path, warn=warnings.append)
    image = subject.image_dir / 'old.jpg'
    image.write_bytes(b'jpeg')
    os.utime(image, (1., 1.))
    (subject.spool_dir / 'broken.json').write_text('{')
    subject.prune_images(wall_time=864000.)
    assert image.exists() and len(warnings) == 1


def test_write_recreates_vanished_directory(tmp_path):
    subject = spool(tmp_path)
    subject.spool_dir.rmdir()
    saved = subject.write(event())
    assert saved.exists() and not list(subject.spool_dir.glob('*.tmp'))


def test_directory_vanishing_during_replace_retried_only_once(tmp_path, monkeypatch):
    subject = spool(tmp_path)
    attempts = []

    def disappear(source, destination):
        attempts.append(destination)
        source.unlink()
        source.parent.rmdir()
        raise FileNotFoundError('queue vanished')

    monkeypatch.setattr(os, 'replace', disappear)
    with pytest.raises(FileNotFoundError, match='queue vanished'):
        subject.write(event())
    assert len(attempts) == 2
