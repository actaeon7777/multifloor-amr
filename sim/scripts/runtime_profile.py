"""Opt-in wall-time measurements; no change to physics or sensor rates."""
from collections import defaultdict
import gc
import json
import os
from pathlib import Path
import time


class RuntimeProfile:
    def __init__(self):
        self.path = os.environ.get('AMR_SIM_PROFILE')
        self.samples = defaultdict(list)
        self.next_save = 5.
        self.spikes = []
        self.metadata = {}
        self.gc_events = []
        self.sim_time = 0.
        self.gc_started = 0.
        if self.path:
            gc.callbacks.append(self._gc_event)

    def _gc_event(self, phase, info):
        if phase == 'start':
            self.gc_started = time.perf_counter()
        else:
            duration = time.perf_counter() - self.gc_started
            self.samples['gc_generation_' + str(info['generation'])].append(duration)
            if duration > .025:
                self.gc_events.append({'generation': info['generation'], 'wall_time': self.gc_started,
                                       'sim_time_s': self.sim_time, 'duration_ms': 1000*duration})

    def start(self):
        return time.perf_counter() if self.path else 0.

    def record(self, name, start, sim_time):
        self.sim_time = sim_time
        if self.path and sim_time > 2.:
            duration = time.perf_counter() - start
            self.samples[name].append(duration)
            if duration > .025:
                self.spikes.append({'phase': name, 'sim_time_s': sim_time,
                                    'wall_time': start, 'duration_ms': 1000*duration})

    def save(self, sim_time, force=False):
        if not self.path or (not force and sim_time < self.next_save):
            return
        self.next_save = sim_time + 5.
        result = {'sim_time_s': sim_time, 'timings_ms': {}, 'spikes': self.spikes,
                  'metadata': self.metadata, 'gc_events': self.gc_events}
        for name, values in list(self.samples.items()):
            ordered = sorted(values)
            result['timings_ms'][name] = {
                'count': len(values), 'total_s': sum(values),
                'mean': 1000*sum(values)/len(values),
                'p95': 1000*ordered[int(.95*(len(values)-1))],
                'p99': 1000*ordered[int(.99*(len(values)-1))],
                'max': 1000*ordered[-1]}
        path = Path(self.path)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(result, indent=2)+'\n')
        temporary.replace(path)
