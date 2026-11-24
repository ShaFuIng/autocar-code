"""Downhill road = the two rightmost white lines (downhill-pair-r1).

User 2026-11-04 (screenshot e55523e7 27.05 s): at the slope entry the image shows the flat track's lines on the left, the
downhill's left edge in the middle and its right edge on the right. The line estimator rejected the correct pair (middle +
right, width ratio 0.27-0.46 against the expected ~0.7: the road curves right and looks narrow) and paired the far-left flat
line with the right edge; the target fell on the middle line and the car steered hard left (e55523e7 1145 us,
d86fc9cc 27.20-27.31 s 1135-1153 us). The downhill is one right curve and its road is the gap between the two rightmost lines.

On the slope (dash route DONE, pitch below PITCH_DOWN) live_pid passes the final path and the V4 mask. Around the control row
the two rightmost white-line runs of the mask are followed row by row (each row continues within STEP_PX of the previous
one, gap >= MIN_GAP). If the path's target at the control row lies LEFT of that gap (beyond EDGE_PX), the path is replaced
by the centre of the gap. A target inside the gap, a target RIGHT of the rightmost line and a frame without two such lines
are left alone. Right-of-pair is kept on purpose: when the downhill's right edge is out of view the rightmost visible line is
its left edge and the road is to its right (downhill-entry mirror, checked on 20aa810f / 952133fa); a first version that also
replaced those pulled the target 300-500 px left (1f3d7ba7 26.8 s, 373328c8 21.6 s, cfafa5c5 29.3 s; check.py). Not used in the two-lane section (route OPEN): there the two
rightmost lines are the right lane. Values are experiment values from these frames.
"""
import base64
import numpy as np
import cv2

REVISION = 'downhill-pair-r1'
PITCH_DOWN = -2.
MIN_RUN, MIN_GAP, STEP_PX, EDGE_PX = 3, 40, 14, 4   # 640x360 px
UP_ROWS, DOWN_ROWS, MIN_ROWS = 40, 60, 12            # rows followed above / below the control row; rows needed (2-px steps)


def _runs(row):
    d = np.diff(np.r_[0, row.astype(np.int8), 0]); a = np.flatnonzero(d == 1); b = np.flatnonzero(d == -1)
    return [(int(lo), int(hi - 1)) for lo, hi in zip(a, b) if hi - lo >= MIN_RUN]


def _pair(mask, y):
    rs = _runs(mask[max(0, y - 1):y + 2].any(axis=0))
    if len(rs) < 2: return None
    l, r = rs[-2][1], rs[-1][0]
    return (l, r) if r - l >= MIN_GAP else None


def rightmost_gap(mask, row_y):
    """mask bool 360x640, row_y control row (640 scale). Returns {y: (left_edge, right_edge)} or None."""
    y0 = int(round(row_y)); start = _pair(mask, y0)
    if start is None: return None
    out = {y0: start}
    for step in (-2, 2):
        prev = start; lim = y0 - UP_ROWS if step < 0 else min(mask.shape[0] - 2, y0 + DOWN_ROWS)
        for y in range(y0 + step, lim, step):
            p = _pair(mask, y)
            if p is None or abs(p[0] - prev[0]) > STEP_PX or abs(p[1] - prev[1]) > STEP_PX: break
            out[y] = prev = p
    return out if len(out) >= MIN_ROWS else None


def _x_at(path, y):   # path x (1280 scale) at y (1280 scale), linear between the nearest points
    P = sorted((float(p[1]), float(p[0])) for p in path or [])
    for (ya, xa), (yb, xb) in zip(P, P[1:]):
        if ya <= y <= yb: return xa if yb == ya else xa + (xb - xa) * (y - ya) / (yb - ya)
    return None


def apply(result, row_ratio, pitch, route_done):
    """Returns (path, info) when the path is replaced, else (None, info or None)."""
    if not route_done or pitch is None or pitch >= PITCH_DOWN: return None, None
    if not result.get('valid') or result.get('selected_source') is not None: return None, None
    info = dict(revision=REVISION)
    png = result.get('mask_png')
    if not png: info.update(reason='NO_MASK'); return None, info
    m = cv2.imdecode(np.frombuffer(base64.b64decode(png), np.uint8), 0)
    if m is None: info.update(reason='MASK_DECODE'); return None, info
    if m.shape != (360, 640): m = cv2.resize(m, (640, 360), interpolation=cv2.INTER_NEAREST)
    row_y = float(row_ratio) * 359.; gap = rightmost_gap(m > 0, row_y)
    if gap is None: info.update(reason='NO_TWO_LINES'); return None, info
    l, r = gap[int(round(row_y))]; xt = _x_at(result.get('path'), row_y * 2.)
    info.update(left_px=l, right_px=r, rows=len(gap), target_px=None if xt is None else round(xt / 2., 1))
    if xt is None: info.update(reason='NO_TARGET'); return None, info
    if xt / 2. > r + EDGE_PX: info.update(reason='RIGHT_OF_PAIR'); return None, info
    if xt / 2. >= l - EDGE_PX: info.update(reason='INSIDE'); return None, info
    path = [[float(a + b), float(y * 2)] for y, (a, b) in sorted(gap.items())]   # centre (a + b) / 2, 1280 scale
    info.update(reason='REPLACED', centre_px=round((l + r) / 2., 1)); return path, info
