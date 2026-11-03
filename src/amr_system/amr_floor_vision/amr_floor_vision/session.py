"""Arrival binding/learning; all methods run on the node's single worker."""
from pathlib import Path
import numpy as np

from .dataset import write_sample
from .recognizer import Gallery, decide, floor_scores


def build_gallery(directories, cache_path, embedder, model_id, known_floors):
    from PIL import Image
    cached = Gallery()
    if Path(cache_path).is_file():
        try:
            cached = Gallery.load(cache_path)
        except (OSError, ValueError, KeyError):
            pass  # Corrupt/incompatible cache can be rebuilt from the JPEGs.
    lookup = {(p, m): i for i, (p, m) in enumerate(zip(cached.paths, cached.mtimes))}
    gallery = Gallery()
    gallery.model_id = model_id
    paths = sorted({p.resolve() for root in directories for p in Path(root).glob('*/*.jpg')
                    if p.parent.name in known_floors})
    for path in paths:
        mtime = path.stat().st_mtime_ns
        index = lookup.get((str(path), mtime)) if cached.model_id == model_id else None
        if index is None:
            with Image.open(path) as image:
                embedding = embedder.embed([image.convert('RGB')])
        else:
            embedding = cached.embeddings[index:index + 1]
        gallery.add(embedding, path.parent.name, [path], [mtime])
    gallery.save(cache_path)
    return gallery


class ArrivalSample:
    """A door opening owns its frames even when binding precedes inference completion."""
    def __init__(self, gallery, dataset_dir, model_id, known_floors, target_floor=''):
        self.gallery, self.dataset_dir, self.model_id = gallery, Path(dataset_dir), model_id
        self.known_floors, self.target_floor = set(known_floors), target_floor
        self.candidate = self.answer = self.verdict = self.record = None
        self.ready = False
        self.images, self.embeddings = [], np.empty((0, 0), dtype=np.float32)
        self.guess = dict(floor='', score=0., margin=0.)
        self.scores = {}

    def bind(self, candidate_id, asked_floor, source):
        if self.candidate is None:
            self.candidate = dict(candidate_id=candidate_id, asked_floor=asked_floor, source=source)

    def complete(self, images, embeddings, min_score, min_margin):
        self.images, self.embeddings = list(images), embeddings
        floor, score, margin = decide(embeddings, self.gallery, min_score, min_margin)
        self.guess = dict(floor=floor, score=score, margin=margin)
        self.scores = floor_scores(embeddings, self.gallery)
        self.ready = True
        self._save()
        return self.guess

    def confirm(self, candidate_id, floor, accepted, actual_floor=''):
        """Remember the latest valid answer for the record; the label comes from ``resolve``."""
        if (self.candidate is None or self.record is not None
                or candidate_id != self.candidate['candidate_id']
                or floor != self.candidate['asked_floor']):
            return False
        if (type(accepted) is not bool or (accepted and actual_floor)
                or (actual_floor and (actual_floor not in self.known_floors or actual_floor == floor))):
            return False
        self.answer = dict(accepted=accepted, actual_floor=actual_floor)
        return True

    def resolve(self, candidate_id, verified_floor):
        """The monitor's verdict for the bound candidate: the VERIFIED floor, or '' (rejected,
        "don't know", or the door closed unanswered). An answer the monitor ignored (door
        closing, stale) must not label the frames."""
        if self.candidate is None or self.verdict is not None or candidate_id != self.candidate['candidate_id']:
            return False
        self.verdict = verified_floor if verified_floor in self.known_floors else ''
        self._save()
        return True

    def _save(self):
        if not self.ready or self.verdict is None or self.record is not None:
            return
        label = self.verdict or 'unknown'
        answer = self.answer or dict(accepted=None, actual_floor='')
        self.record = write_sample(self.dataset_dir, self.images, **self.candidate, **answer,
                                   guess=self.guess, label=label, model_id=self.model_id,
                                   target_floor=self.target_floor)
        # Only a person's answer teaches the gallery: learning from its own automatic decisions
        # would let one confident mistake reinforce itself. Those samples are kept on disk.
        if (label in self.known_floors and len(self.embeddings)
                and self.candidate.get('source') != 'VISION_AUTO'):
            paths = self.record['image_paths']
            self.gallery.add(self.embeddings, label, paths, [Path(p).stat().st_mtime_ns for p in paths])
            self.gallery.save(self.dataset_dir / 'gallery_cache.npz')
