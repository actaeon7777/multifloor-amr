import numpy as np
import pytest

from amr_floor_vision.recognizer import Gallery, decide, floor_scores


def test_gallery_round_trip(tmp_path):
    gallery = Gallery()
    gallery.add([[2, 0], [0, 3]], ['3F', '4F'], ['a.jpg', 'b.jpg'], [123, 456])
    gallery.model_id = 'test-model'
    gallery.save(tmp_path / 'gallery.npz')
    loaded = Gallery.load(tmp_path / 'gallery.npz')
    np.testing.assert_allclose(loaded.embeddings, np.eye(2))
    assert (loaded.labels, loaded.paths, loaded.mtimes, loaded.model_id) == (
        gallery.labels, gallery.paths, gallery.mtimes, gallery.model_id)
    Gallery().save(tmp_path / 'empty.npz')
    assert Gallery.load(tmp_path / 'empty.npz').labels == []


def test_votes_choose_floor_and_score_uses_all_frames():
    gallery = Gallery()
    gallery.add(np.eye(2), ['3F', '4F'], ['a', 'b'])
    frames = [[1, 0], [1, 0], [0, 1]]
    floor, score, margin = decide(frames, gallery, .6, .1)
    assert floor == '3F'
    assert score == pytest.approx(2/3)
    assert margin == pytest.approx(1/3)
    assert floor_scores(frames, gallery)['4F'] == pytest.approx(1/3)


def test_unknown_rules_and_close_floor_scores():
    gallery = Gallery()
    assert decide([[1, 0]], gallery) == ('', 0., 0.)
    gallery.add([[1, 0]], '3F', ['a'])
    assert decide([[1, 0]], gallery)[0] == ''
    gallery.add([[.995, .1]], '4F', ['b'])
    floor, score, margin = decide([[1, 0]], gallery)
    assert floor == '' and score == pytest.approx(1.) and 0 < margin < .01
    assert decide([[0, 1]], gallery)[0] == ''  # unseen floor below .85
    assert decide([], gallery) == ('', 0., 0.)


def test_feasibility_score_scale():
    gallery = Gallery()
    # A 0.945 confusion with 0.95 correct-floor similarity cannot pass a .01 margin.
    gallery.add([[.95, np.sqrt(1-.95**2), 0], [.945, 0, np.sqrt(1-.945**2)]],
                ['3F', '4F'], ['a', 'b'])
    floor, score, margin = decide([[1, 0, 0]] * 3, gallery)
    assert floor == '' and score == pytest.approx(.95) and margin == pytest.approx(.005)
    assert decide([[1, 0, 0]], gallery, min_margin=.004)[0] == '3F'


def test_majority_is_not_replaced_by_highest_aggregate_score():
    gallery = Gallery()
    gallery.add(np.eye(2), ['3F', '4F'], ['a', 'b'])
    assert decide([[.72, .70], [.72, .70], [0, 1]], gallery, 0, 0)[0] == ''
    with pytest.raises(ValueError):
        gallery.add([[0, 0]], '3F', ['bad'])
