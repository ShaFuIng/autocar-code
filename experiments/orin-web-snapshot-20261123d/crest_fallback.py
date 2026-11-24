"""Ramp crest fallback for the live PID (crest-fallback-r1).

At the crest the car is pitched ~14 deg while the road ahead is flat: the
visible road's horizon drops ~140 px, the corridor (+-20 px) and the baseline
pairing (fixed width geometry) both lose it although both white lines are in
the mask (live cee4254f 44.1-44.5 s, 0e343cb4 53.1 s, 5c400352 51.0 s), and
the run stops on the invalid path.

When the selected path is invalid and the IMU says the car is (or was within
RECENT_S) pitched up by at least ON_DEG above flat, use the centre between
the two observed white lines, followed row by row from the bottom of the
mask (no width or horizon model), extended at most EXT_PX up to the control
row. Limited to MAX_S of consecutive use; any valid normal path resets it.
Stale / missing IMU -> no fallback (the normal stop rule applies).
crest-20261101/crest_midpoint.py: target at all three live stop frames
(398/396/398 px), hand-push 9a2c6570 crest gap reduced to one frame.
Experiment values, not safety limits.
"""
import base64
import numpy as np
import cv2

REVISION = 'crest-fallback-r3'
ON_DEG, RECENT_S, MAX_S = 8., 2., 3.
AXIS = 338; STEP_PX = 14; MIN_GAP = 24; MIN_ROWS = 15; EXT_PX = 24


def _runs(row):
    d = np.diff(np.r_[0, row.astype(np.int8), 0]); a = np.flatnonzero(d == 1); b = np.flatnonzero(d == -1)
    return [(lo, hi - 1) for lo, hi in zip(a, b) if hi - lo >= 2]


def midpoint_path(mask, row_y, axis=AXIS):
    """mask: bool 360x640; row_y: control row (640x360 scale). Path in 1280x720."""
    pts = []; left = right = None; c = float(axis)
    for y in range(340, 40, -2):
        rs = _runs(mask[y - 1:y + 2].any(axis=0))
        L = [r for r in rs if (r[0] + r[1]) / 2 < c]; R = [r for r in rs if (r[0] + r[1]) / 2 > c]
        if not L or not R:
            if pts: break
            continue
        l = max(L, key=lambda r: r[1])[1]; r = min(R, key=lambda r: r[0])[0]
        if left is not None and (abs(l - left) > STEP_PX or abs(r - right) > STEP_PX): break
        if r - l < MIN_GAP: break
        left, right, c = l, r, (l + r) / 2; pts.append([c * 2, y * 2])
    pts = sorted(pts, key=lambda p: p[1]); extended = False
    if len(pts) >= MIN_ROWS and row_y < pts[0][1] / 2 <= row_y + EXT_PX:
        # r2: OffscreenPID's continuous_paths splits a path at gaps > 24 px (1280
        # scale); r1 added one point 33 px above the rest (live 2302a9fa 43.97 s),
        # which was split off and the control row was not covered. Insert points
        # every <= 4 px (640 scale) and end slightly above the control row.
        top = np.array(pts[:10], float) / 2; k = np.polyfit(top[:, 1], top[:, 0], 1)
        y0 = pts[0][1] / 2; ys = np.arange(y0 - 4, row_y - 1, -4)
        ys = np.r_[ys, row_y - 1] if not len(ys) or ys[-1] > row_y - 1 else ys
        pts = [[float(np.polyval(k, y)) * 2, float(y) * 2] for y in sorted(ys)] + pts; extended = True
    ok = len(pts) >= MIN_ROWS and pts[0][1] / 2 <= row_y <= pts[-1][1] / 2
    return dict(path=[[float(x), float(y)] for x, y in pts] if ok else [], valid=ok, rows=len(pts), extended=extended)


class CrestFallback:
    def __init__(self, boost):
        self.boost = boost; self.last_pitched = None; self.since = None

    def normal_valid(self):
        self.since = None

    # r3: live f6c02054 42.6-44.9 s: at the crest the corridor was selected and
    # 'valid' but pointed far left while the two-line midpoint was correct every
    # frame; r2 only replaced invalid paths. live_pid now also calls this with
    # mode='corridor' when the selected source is the corridor; the MAX_S limit
    # applies to the invalid case only (the pitch gate bounds the corridor case).
    def try_path(self, now, result, row_ratio, mode='invalid'):
        info = dict(revision=REVISION, used=False, mode=mode)
        try:
            t, p = self.boost._read()
        except Exception as e:
            info.update(reason='IMU_UNAVAILABLE', error=type(e).__name__); return None, info
        age = now - t; rel = p - self.boost.c['flat_pitch_deg']
        info.update(pitch_rel_deg=round(rel, 2), imu_age_s=round(age, 3))
        if not -.05 <= age <= self.boost.c['max_age_s']:
            info.update(reason='IMU_STALE'); return None, info
        if rel >= ON_DEG: self.last_pitched = now
        if self.last_pitched is None or now - self.last_pitched > RECENT_S:
            info.update(reason='NOT_PITCHED'); return None, info
        if mode == 'invalid' and self.since is not None and now - self.since > MAX_S:
            info.update(reason='MAX_TIME'); return None, info
        png = result.get('mask_png')
        if not png:
            info.update(reason='NO_MASK'); return None, info
        m = cv2.imdecode(np.frombuffer(base64.b64decode(png), np.uint8), 0)
        if m is None:
            info.update(reason='MASK_DECODE'); return None, info
        if m.shape != (360, 640): m = cv2.resize(m, (640, 360), interpolation=cv2.INTER_NEAREST)
        mid = midpoint_path(m > 0, row_ratio * 719 / 2)
        info.update(rows=mid['rows'], extended=mid['extended'])
        if not mid['valid']:
            info.update(reason='NO_TWO_LINES'); return None, info
        if mode == 'invalid': self.since = now if self.since is None else self.since
        info.update(reason='OK', used=True, used_s=round(now - self.since, 3) if self.since is not None else None); return mid['path'], info
