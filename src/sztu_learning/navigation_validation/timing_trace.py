"""Small metadata trace; no sensor payload copies and no control decisions."""
import json
import time


class TimingTrace:
    def __init__(self, path):
        self.handle = path.open('x', buffering=65536)
        self.previous = {}
        self.counts = {}
        self.flushed = time.monotonic()

    def record(self, kind, stamp=None, **details):
        now = time.monotonic()
        previous = self.previous.get(kind)
        self.previous[kind] = now
        self.counts[kind] = self.counts.get(kind, 0) + 1
        # IMU continues at 200 Hz after startup: retain counters, one sample/s,
        # and callback gaps. A gap is evidence, not a new acceptance threshold.
        if kind == 'imu' and self.counts[kind] % 200 and (previous is None or now-previous < .04):
            return
        self.handle.write(json.dumps(dict(epoch=time.time(), monotonic=now, kind=kind,
                                         stamp=stamp, callback_gap_s=None if previous is None else now-previous,
                                         count=self.counts[kind], **details))+'\n')
        if now-self.flushed > 1 or kind == 'lost':
            self.handle.flush()
            self.flushed = now

    def close(self):
        self.handle.close()
