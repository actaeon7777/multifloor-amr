from datetime import datetime
from io import BytesIO
import json
import os
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from amr_floor_vision.dataset import write_sample
from amr_floor_vision.recognizer import Gallery
from amr_floor_vision.session import ArrivalSample, build_gallery


def jpeg():
    stream = BytesIO()
    Image.new('RGB', (8, 8), 'red').save(stream, format='JPEG')
    return stream.getvalue()


class FakeEmbedder:
    calls = 0

    def embed(self, images):
        self.calls += 1
        return np.array([[1., 0.]] * len(images), dtype=np.float32)


def test_dataset_json_paths_and_collision(tmp_path):
    args = dict(candidate_id=7, guess=dict(floor='4F', score=.94, margin=.03),
                asked_floor='4F', source='VISION', accepted=False, actual_floor='3F',
                label='3F', model_id='fake', target_floor='4F', time=datetime(2026, 11, 2, 12, 30, 15))
    record = write_sample(tmp_path, [jpeg()], **args)
    path = tmp_path / 'arrival/3F/20261102_123015_c7_0.jpg'
    assert record['image_paths'] == [str(path)] and path.read_bytes() == jpeg()
    assert record['label'] == '3F' and record['guess']['floor'] == '4F'
    second = write_sample(tmp_path, [jpeg()], **args)
    assert second['image_paths'] != record['image_paths']
    assert [json.loads(line) for line in (tmp_path / 'labels.jsonl').read_text().splitlines()] == [record, second]
    with pytest.raises(ValueError):
        write_sample(tmp_path, [], **{**args, 'label':'../bad'})


@pytest.mark.parametrize('early_answer', [False, True])
@pytest.mark.parametrize('accepted,actual,label,learn', [(True, '', '4F', True),
    (False, '3F', '3F', True), (False, '1F', '1F', True), (False, '', 'unknown', False)])
def test_bind_label_and_learning(tmp_path, early_answer, accepted, actual, label, learn):
    gallery = Gallery()
    gallery.add(np.eye(2), ['3F', '4F'], ['a', 'b'])
    sample = ArrivalSample(gallery, tmp_path, 'fake', {'1F', '3F', '4F'}, '4F')
    images = [jpeg()] * 3
    vectors = FakeEmbedder().embed(images)
    if not early_answer:
        sample.complete(images, vectors, .85, .01)
    sample.bind(7, '4F', 'TARGET')
    assert not sample.confirm(8, '4F', True)
    assert not sample.confirm(7, '3F', True)
    assert sample.confirm(7, '4F', accepted, actual)
    assert not (tmp_path / 'labels.jsonl').exists()   # an answer alone never labels
    assert not sample.resolve(8, label)
    assert sample.resolve(7, '' if label == 'unknown' else label)   # the monitor's verdict
    if early_answer:
        assert not (tmp_path / 'labels.jsonl').exists()
        sample.complete(images, vectors, .85, .01)
    assert sample.record['label'] == label and sample.record['guess']['floor'] == '3F'
    assert sample.record['accepted'] is accepted
    assert not sample.resolve(7, label)
    assert len((tmp_path / 'labels.jsonl').read_text().splitlines()) == 1
    assert len(gallery.labels) == (5 if learn else 2)
    if learn:
        assert Gallery.load(tmp_path / 'gallery_cache.npz').labels[-3:] == [label] * 3


def test_cache_reuses_paths_mtimes_and_invalidates_model(tmp_path):
    root = tmp_path / 'arrival'
    for floor in ('3F', '4F', 'unknown'):
        (root / floor).mkdir(parents=True)
        (root / floor / 'a.jpg').write_bytes(jpeg())
    embedder = FakeEmbedder()
    cache = tmp_path / 'gallery_cache.npz'
    def build(model='fake'):
        return build_gallery([root, root], cache, embedder, model, {'3F', '4F'})
    assert len(build().labels) == 2 and embedder.calls == 2
    assert len(build().labels) == 2 and embedder.calls == 2
    path = root / '3F/a.jpg'
    os.utime(path, ns=(path.stat().st_atime_ns, path.stat().st_mtime_ns + 1))
    build()
    assert embedder.calls == 3
    build('new-model')
    assert embedder.calls == 5
    path.unlink()
    assert build('new-model').labels == ['4F']


def test_samples_do_not_rebind_or_leak_across_openings(tmp_path):
    gallery = Gallery()
    first = ArrivalSample(gallery, tmp_path, 'fake', {'3F', '4F'})
    first.bind(7, '4F', 'VISION')
    first.bind(8, '3F', 'TARGET')
    assert not first.confirm(8, '3F', True)
    second = ArrivalSample(gallery, tmp_path, 'fake', {'3F', '4F'})
    second.bind(8, '3F', 'TARGET')
    assert not second.confirm(7, '4F', True)
    assert not second.confirm(8, '3F', False, 'bad')
    assert not second.confirm(8, '3F', False, '3F')


def test_ignored_answer_does_not_label_frames(tmp_path):
    """Review 2026-11-02: "yes 4F" ignored by the monitor (door closing), then "no -> 3F" verified."""
    gallery = Gallery()
    gallery.add(np.eye(2), ['3F', '4F'], ['a', 'b'])
    sample = ArrivalSample(gallery, tmp_path, 'fake', {'3F', '4F'}, '3F')
    images = [jpeg()] * 3
    sample.complete(images, FakeEmbedder().embed(images), .85, .01)
    sample.bind(7, '4F', 'VISION')
    assert sample.confirm(7, '4F', True)
    assert sample.confirm(7, '4F', False, '3F')
    assert sample.resolve(7, '3F')
    assert sample.record['label'] == '3F' and gallery.labels[-1] == '3F'


def test_automatic_decision_is_saved_but_not_learned(tmp_path):
    gallery = Gallery()
    gallery.add(np.eye(2), ['3F', '4F'], ['a', 'b'])
    sample = ArrivalSample(gallery, tmp_path, 'fake', {'3F', '4F'}, '3F')
    images = [jpeg()] * 3
    sample.complete(images, FakeEmbedder().embed(images), .85, .01)
    sample.bind(9, '3F', 'VISION_AUTO')
    assert sample.resolve(9, '3F')
    assert sample.record['label'] == '3F' and sample.record['source'] == 'VISION_AUTO'
    assert len(gallery.labels) == 2
