"""Steering play compensation at the output stage (play-comp-r1).

User 2026-11-05/06: the straights zig-zag (12 evening runs: mean |error| 0.134, 65 % of the straight commands inside the
1340-1490 us dead zone, side changes every 3.5 s) and half of the flat lap runs at LOW speed because of it.
Measured 2026-11-05: commands 1340-1490 give no yaw (turn-map-20261105); the record-only centre estimator gives the straight
point about 1500 us when the command arrives from the left and about 1360 us from the right (saved centre 1390), i.e. the
linkage play is about 140 us and the saved centre sits inside it.
The controller's own compensation (visual_pid backlash, off) is scaled down by the gain schedule at speed and caps the
steering rate at 500-800 us/s (play-comp-20261105/findings.json), so it is not used. This module runs on the FINAL command
right before it is sent: it remembers the direction the command last moved in (reversal detector with THRESHOLD_US of
hysteresis on the raw command) and adds +right_us when the last move was to the right, -left_us to the left. No rate cap
(the servo's own 1500 us/s applies). Values come from play_comp.json, read at each run start, so they can be tuned without
a restart. Experiment values, not a measured mechanical play.
"""
import json
from pathlib import Path

REVISION = 'play-comp-r1'
CONFIG_FILE = Path(__file__).with_name('play_comp.json')
DEFAULTS = dict(enabled=False, right_us=110., left_us=30., threshold_us=30.)


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


class PlayComp:
    def __init__(self, config=None):
        self.c = dict(config) if config is not None else load_config()
        self.direction = 0; self.extreme = None; self.reversals = 0

    def reset(self):
        self.direction = 0; self.extreme = None; self.reversals = 0

    def apply(self, steer_us, center_us, min_us, max_us):
        """Returns (compensated command, info). info is None when disabled."""
        c = self.c
        if not c.get('enabled'): return float(steer_us), None
        raw = float(steer_us); th = float(c.get('threshold_us', 30.))
        if self.extreme is None: self.extreme = float(center_us)
        switched = False
        if self.direction == 0:
            if raw - self.extreme >= th: self.direction = 1; self.extreme = raw; switched = True
            elif self.extreme - raw >= th: self.direction = -1; self.extreme = raw; switched = True
        elif self.direction > 0:
            if raw >= self.extreme: self.extreme = raw
            elif self.extreme - raw >= th: self.direction = -1; self.extreme = raw; switched = True; self.reversals += 1
        else:
            if raw <= self.extreme: self.extreme = raw
            elif raw - self.extreme >= th: self.direction = 1; self.extreme = raw; switched = True; self.reversals += 1
        off = float(c.get('right_us', 0.)) if self.direction > 0 else (-float(c.get('left_us', 0.)) if self.direction < 0 else 0.)
        out = max(float(min_us), min(float(max_us), raw + off))
        return out, dict(revision=REVISION, raw_us=round(raw, 1), offset_us=round(off, 1), direction=self.direction,
                         reversal=switched, reversals=self.reversals, steer_us=round(out, 1))
