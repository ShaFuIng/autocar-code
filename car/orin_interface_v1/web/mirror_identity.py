"""Mirror gate by boundary identity (downhill-mirror-identity-r1, offline candidate, not in the payload).

Problem (downhill-right-arrival-20261105): downhill_entry.mirror() takes a lone 'right+width' line on the first 1.5 s of
the slope as the outer LEFT edge of the right curve and mirrors the path about it. When the car arrives on the right edge
the lone line IS the right edge, the mirror puts the target outside the road and the car is steered off to the right.
Codex review: x < axis alone is not enough (after drifting right the real right edge is left of the axis, 23ec1c35 233 px);
carry the left/right identity from the earlier frames, use the axis only as a fallback.

Tracker on the V4 mask (640x360), rows ROWS, white runs >= MIN_W px (3-row band, as crest_marks):
 - start: a row-wise pair (nearest run left of the axis, nearest run right of it) on >= MIN_ROWS common rows sets both
   tracks (left track = inner edge of the left line = run end, right track = inner edge of the right line = run start);
 - every later frame: each live track takes, per row, the matching edge nearest to its last x within TOL px; >= MIN_ROWS
   rows -> new x (median). A track not matched for MAX_AGE_S is dropped. Both dropped -> start again.
 - decide(x): the lone line's x on the same rows. Nearest live track within TOL -> 'RIGHT' (do not mirror) or 'LEFT'
   (mirror). Otherwise None -> fallback: mirror only when the line is left of the axis at the sample row.
All numbers are experiment values (TOL from the crest-to-slope image shift in the replay); not a verified limit.
"""
import numpy as np

REVISION = 'downhill-mirror-identity-r1'
AXIS = 338
ROWS = tuple(range(250, 345, 10))
MIN_W, MIN_ROWS = 6, 4
TOL, MAX_AGE_S = 60., 1.0


def _runs(row):
    d = np.diff(np.r_[0, row.astype(np.int8), 0]); a = np.flatnonzero(d == 1); b = np.flatnonzero(d == -1)
    return [(int(lo), int(hi - 1)) for lo, hi in zip(a, b) if hi - lo >= MIN_W]


def row_runs(mask):
    return {y: _runs(mask[y - 1:y + 2].any(axis=0)) for y in ROWS}


def boundary_x(points_1280):
    """Median x (640 frame) of a 1280x720 boundary polyline on ROWS, or None."""
    by = {int(round(p[1] / 2.)): p[0] / 2. for p in points_1280 or [] if p and len(p) >= 2}
    xs = [by[y] for y in ROWS if y in by]
    return float(np.median(xs)) if len(xs) >= MIN_ROWS else None


class BoundaryIdentity:
    def __init__(self):
        self.x = {'L': None, 'R': None}; self.t = {'L': None, 'R': None}

    def live(self, side, now):
        return self.x[side] is not None and self.t[side] is not None and now - self.t[side] <= MAX_AGE_S

    def update(self, mask, now):
        rr = row_runs(mask)
        for s in 'LR':
            if not self.live(s, now): self.x[s] = None
        if self.x['L'] is None and self.x['R'] is None:
            pairs = []
            for y, rs in rr.items():
                lft = [hi for lo, hi in rs if (lo + hi) / 2. < AXIS]; rgt = [lo for lo, hi in rs if (lo + hi) / 2. >= AXIS]
                if lft and rgt: pairs.append((max(lft), min(rgt)))
            if len(pairs) >= MIN_ROWS:
                self.x['L'] = float(np.median([a for a, _ in pairs])); self.x['R'] = float(np.median([b for _, b in pairs])); self.t['L'] = self.t['R'] = now
            return dict(self.x)
        for s in 'LR':
            if self.x[s] is None: continue
            got = []
            for y, rs in rr.items():
                edges = [hi for lo, hi in rs] if s == 'L' else [lo for lo, hi in rs]
                if edges:
                    e = min(edges, key=lambda v: abs(v - self.x[s]))
                    if abs(e - self.x[s]) <= TOL: got.append(e)
            if len(got) >= MIN_ROWS: self.x[s] = float(np.median(got)); self.t[s] = now
        return dict(self.x)

    def decide(self, x, now):
        if x is None: return None, None
        best = None
        for s in 'LR':
            if self.live(s, now):
                d = abs(x - self.x[s])
                if d <= TOL and (best is None or d < best[1]): best = (s, d)
        if best is None: return None, None
        return ('LEFT' if best[0] == 'L' else 'RIGHT'), round(best[1], 1)
