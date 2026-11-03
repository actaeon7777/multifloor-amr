"""Write a labelled arrival without ROS or ML dependencies."""
from datetime import datetime
import json
from pathlib import Path


def write_sample(dataset_dir, images, *, candidate_id, guess, asked_floor, source,
                 accepted, actual_floor, label, model_id, target_floor='', time=None):
    instant = time or datetime.now().astimezone()
    label = label or 'unknown'
    if label in ('.', '..') or '/' in label or '\\' in label:
        raise ValueError('label must be a folder name')
    root = Path(dataset_dir)
    folder = root / 'arrival' / label
    folder.mkdir(parents=True, exist_ok=True)
    prefix = f'{instant:%Y%m%d_%H%M%S}_c{candidate_id}'
    paths = []
    for index, image in enumerate(images):
        path = folder / f'{prefix}_{index}.jpg'
        # Candidate IDs restart with the monitor; never overwrite an earlier visit.
        while path.exists():
            index += max(1, len(images))
            path = folder / f'{prefix}_{index}.jpg'
        if isinstance(image, bytes):
            path.write_bytes(image)
        else:
            image.convert('RGB').save(path, format='JPEG')
        paths.append(str(path.resolve()))
    record = dict(time=instant.isoformat(), candidate_id=candidate_id, guess=dict(guess),
                  asked_floor=asked_floor, source=source, accepted=accepted,
                  actual_floor=actual_floor, label=label, target_floor=target_floor,
                  image_paths=paths, model_id=model_id)
    with (root / 'labels.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
    return record
