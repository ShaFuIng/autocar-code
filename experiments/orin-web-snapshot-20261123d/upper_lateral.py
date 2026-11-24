"""Camera lateral correction on the upper straight before the slope (upper-lateral-r1).

User 2026-11-05 (raw-20261105e e358c5fd / 1410ae24): while following the dashed straight the car went towards the slope right of the
lane centre and then went down too far right. In both runs, ~3 s before the slope the lane segment steered on the heading-hold
heading alone (source 'heading': side lines dropped by the direction gates, no dash fit); the normal camera path error went
+0.3 -> -0.6 (e358) / -0.2 -> -0.65 (1410), i.e. the car drifted right of centre, while the steering stayed 1290-1330 us
(centre 1390, the straight point arriving from the right ~1348 us: barely left). crest-marks-r2.1 (applied) found its marks only
in the last ~2 s and gave -12 .. -107 us.

Here, on those heading / gyro frames of the upper section (route OPEN, after the dashes were followed), the low-passed camera error
adds a LEFT-only offset (like crest-marks):  off = -min(CAP_US, K_US * (-e_f - DEAD)) for e_f < -DEAD, else 0.
e_f: exponential low-pass (TAU_S) of the image error of normal-path frames; a gap longer than GAP_S clears it.
live_pid uses the stronger of this offset and the crest-marks offset (they do not add up).
K_US / DEAD / CAP_US / TAU_S are experiment values (-0.35 -> -60 us, -0.5 -> -105, -0.65 -> -150), not a verified limit.
"""
import math

REVISION = 'upper-lateral-r1'
K_US, DEAD, CAP_US, TAU_S, GAP_S = 300., .15, 150., .3, .5
SOURCES = ('heading', 'gyro')


class UpperLateral:
    def __init__(self):
        self.e = None; self.t = None

    def offset(self, now, err, source, route_open, followed):
        """err: image error of this frame when it is a normal-path frame, else None. Returns (off_us, info) or (0., None)."""
        if not (route_open and followed):
            self.e = None; self.t = None; return 0., None
        if err is not None:
            err = max(-1., min(1., float(err)))
            if self.e is None or self.t is None or now - self.t > GAP_S: self.e = err
            else: self.e += (1. - math.exp(-max(0., now - self.t) / TAU_S)) * (err - self.e)
            self.t = now
        elif self.t is not None and now - self.t > GAP_S:
            self.e = None
        if source not in SOURCES: return 0., None
        info = dict(revision=REVISION, source=source, err=None if err is None else round(err, 3), err_f=None if self.e is None else round(self.e, 3))
        if self.e is None or self.e >= -DEAD:
            info.update(reason='CENTRED' if self.e is not None else 'NO_ERROR', off_us=0.); return 0., info
        off = -min(CAP_US, K_US * (-self.e - DEAD)); info.update(reason='RIGHT_OF_CENTRE', off_us=round(off, 1))
        return off, info
