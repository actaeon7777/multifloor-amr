"""NumPy-only cosine nearest-reference voting and persistent gallery."""
from pathlib import Path
import numpy as np


def normalized(values):
    values = np.asarray(values, dtype=np.float32)
    if values.ndim != 2 or not np.isfinite(values).all():
        raise ValueError('embeddings must be a finite (N, D) matrix')
    norms = np.linalg.norm(values, axis=1, keepdims=True)
    if np.any(norms == 0):
        raise ValueError('zero embeddings are not valid')
    return values / norms


class Gallery:
    def __init__(self):
        self.embeddings = np.empty((0, 0), dtype=np.float32)
        self.labels, self.paths, self.mtimes = [], [], []
        self.model_id = ''

    def add(self, embeddings, labels, paths, mtimes=None):
        values = normalized(embeddings)
        labels = [labels] * len(values) if isinstance(labels, str) else list(labels)
        paths = [str(p) for p in paths]
        mtimes = list(mtimes) if mtimes is not None else [0] * len(values)
        if not len(values) == len(labels) == len(paths) == len(mtimes):
            raise ValueError('one label, path and mtime per embedding required')
        if not len(values):
            return
        self.embeddings = np.concatenate((self.embeddings, values)) if self.labels else values.copy()
        self.labels.extend(labels)
        self.paths.extend(paths)
        self.mtimes.extend(mtimes)

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        with temporary.open('wb') as stream:
            np.savez_compressed(stream, embeddings=self.embeddings, labels=np.array(self.labels, dtype=str),
                                paths=np.array(self.paths, dtype=str), mtimes=np.array(self.mtimes, dtype=np.int64),
                                model_id=self.model_id)
        temporary.replace(path)

    @classmethod
    def load(cls, path):
        result = cls()
        with np.load(path, allow_pickle=False) as data:
            result.add(data['embeddings'], data['labels'].tolist(), data['paths'].tolist(),
                       data['mtimes'].tolist())
            result.model_id = str(data['model_id'].item())
        return result


def floor_scores(frame_embeddings, gallery):
    if not len(frame_embeddings) or not gallery.labels:
        return {}
    similarities = normalized(frame_embeddings) @ gallery.embeddings.T
    labels = np.asarray(gallery.labels)
    return {label: float(similarities[:, labels == label].max(axis=1).mean())
            for label in sorted(set(gallery.labels))}


def decide(frame_embeddings, gallery, min_score=0.85, min_margin=0.01):
    scores = floor_scores(frame_embeddings, gallery)
    if not scores:
        return '', 0., 0.
    similarities = normalized(frame_embeddings) @ gallery.embeddings.T
    votes = [gallery.labels[i] for i in similarities.argmax(axis=1)]
    # Equal vote counts use the aggregate similarity, then a stable floor name.
    winner = max(sorted(scores), key=lambda label: (votes.count(label), scores[label]))
    score = scores[winner]
    margin = score - max((v for k, v in scores.items() if k != winner), default=score)
    known = len(scores) >= 2 and score >= min_score and margin >= min_margin
    return winner if known else '', score, margin
