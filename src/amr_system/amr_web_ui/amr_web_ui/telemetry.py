"""Cache common vehicle status and the web goal's distance baseline; no source IO."""
from copy import deepcopy
import time


class DisplayTelemetry:
    def __init__(self):
        self.vehicle = self.distance = self.journey = None
        self.vehicle_received = 0.

    def reset(self):
        self.vehicle = self.distance = self.journey = None

    def receive_vehicle(self, samples, distance, active, now=None):
        now = time.monotonic() if now is None else now
        if self.vehicle and (self.vehicle['vehicle']['session'] != samples['vehicle']['session'] or
                             self.vehicle['vehicle']['epoch'] != samples['vehicle']['epoch']):
            self.journey = None
        self.vehicle,self.distance,self.vehicle_received = deepcopy(samples),dict(distance),now
        if self.journey and active:
            base = self.journey.get('distance_base')
            if (base is None or not distance['valid'] or
                    distance['epoch'] != self.journey.get('distance_epoch') or distance['value'] < base):
                self.journey['complete'] = False
            else:
                self.journey['travelled'] = distance['value']-base

    def begin_journey(self, request_id):
        fresh = (self.vehicle is not None and self.snapshot()['vehicle']['age'] < .5
                 and self.distance['valid'])
        self.journey = dict(request_id=request_id,travelled=0.,complete=bool(fresh),
                            distance_base=self.distance['value'] if fresh else None,
                            distance_epoch=self.distance['epoch'] if fresh else None)

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        result = deepcopy(self.vehicle) if self.vehicle is not None else {}
        extra = max(0.,now-self.vehicle_received)
        for sample in result.values():
            sample['age'] += extra
        if result and result['vehicle']['age'] >= .5:
            for sample in result.values():
                sample.update(fresh=False,hz=None)
        result['journey'] = deepcopy(self.journey)
        return result
