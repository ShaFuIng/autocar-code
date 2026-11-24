"""Line up with the downhill entrance before the slope (downhill-approach-r1).

User 2026-11-04 (live bd03c14e / 23ec1c35): the dashed line is now followed correctly, but both runs went down the slope on
the right side and could not turn back; the car has to line up with the entrance before going down. At the crest the
downhill road is the gap between its left line and its thick right edge (downhill-pair-r1 uses the same pair on the slope).
Measured on the V4 masks, 640x360, row 300, in the last 0.6 s before the slope (pitch < -5):
  went down well  0fb475b7 27.05 s, 9c02d7da 28.22 s: right edge 560-612 px or out of view, left line 160-260 px
  went down right bd03c14e 30.97 s, 23ec1c35 30.41 s: right edge 366-455 px (just right of the car axis 338)
so the car was about half a road width too far right. In those runs the right edge is in view from ~2.5 s before the slope
while lane-segment steers on the heading alone (source heading: no lateral term at pitch > 1 deg, r11).

On the upper section (dash route OPEN) after the dashed line has been followed, on frames where the lane segment steers on
the heading (source heading / heading_lat), the rightmost white run right of RIGHT_MIN_X is followed over rows ROWS
(left edge, row to row within STEP_PX). A straight fit with at least MIN_ROWS rows, RMS <= RMS_MAX and a slope
dx/dy >= SLOPE_MIN (the edge runs out to the right toward the car; the dashes ahead are nearly vertical, slope < 0.3) is
taken as the downhill's right edge. When its x at row X_ROW is left of X_TARGET the steering gets a LEFT offset
K_US * (X_TARGET - x) / 320, at most CAP_US, on top of the heading-hold steering (heading damping still applies, so the
car moves left and the heading hold turns it back). Right of X_TARGET or no such edge: nothing. One-sided on purpose: the
reported failure is entering on the right; a left entry has the right-curve room. Experiment values from these four runs.
"""
import base64
import numpy as np
import cv2

REVISION = 'downhill-approach-r1'
RIGHT_MIN_X, X_ROW, X_TARGET = 280, 300., 560.
ROWS = tuple(range(240, 345, 4))
MIN_RUN, STEP_PX, MIN_ROWS, RMS_MAX, SLOPE_MIN, SLOPE_MAX = 3, 16, 8, 4., .6, 3.
K_US, CAP_US = 400., 200.
SOURCES = ('heading', 'heading_lat')


def _runs(row):
    d = np.diff(np.r_[0, row.astype(np.int8), 0]); a = np.flatnonzero(d == 1); b = np.flatnonzero(d == -1)
    return [(int(lo), int(hi - 1)) for lo, hi in zip(a, b) if hi - lo >= MIN_RUN]


def right_edge(mask):
    """mask bool 360x640. Returns dict(a, b, rows, rms, x) of the downhill right edge (x = a * y + b) or None."""
    pts = {}
    for y in ROWS:
        rs = [r for r in _runs(mask[y - 1:y + 2].any(axis=0)) if r[0] >= RIGHT_MIN_X]
        if rs: pts[y] = rs[-1][0]
    if len(pts) < MIN_ROWS: return None
    ys = sorted(pts); best = []
    for i in range(len(ys)):   # longest row-to-row continuous chain
        ch = [ys[i]]
        for y in ys[i + 1:]:
            if y - ch[-1] > 8 or abs(pts[y] - pts[ch[-1]]) > STEP_PX * (y - ch[-1]) / 4.: break
            ch.append(y)
        if len(ch) > len(best): best = ch
    if len(best) < MIN_ROWS: return None
    Y = np.array(best, float); X = np.array([pts[y] for y in best], float)
    a, b = np.polyfit(Y, X, 1); rms = float(np.sqrt(np.mean((a * Y + b - X) ** 2)))
    if rms > RMS_MAX or not SLOPE_MIN <= a <= SLOPE_MAX: return None
    return dict(a=round(float(a), 3), b=round(float(b), 1), rows=len(best), rms=round(rms, 2), x=round(float(a * X_ROW + b), 1))


def decode(png):
    m = cv2.imdecode(np.frombuffer(base64.b64decode(png), np.uint8), 0)
    if m is None: return None
    if m.shape != (360, 640): m = cv2.resize(m, (640, 360), interpolation=cv2.INTER_NEAREST)
    return m > 0


def offset(result, lane_info, route_open, followed):
    """Left offset (us, <= 0) for this frame and info, or (0., None) when the rule does not apply."""
    if not route_open or not followed or (lane_info or {}).get('reason') != 'OK' or lane_info.get('source') not in SOURCES: return 0., None
    info = dict(revision=REVISION)
    png = result.get('mask_png')
    m = decode(png) if png else None
    if m is None: info.update(reason='NO_MASK'); return 0., info
    e = right_edge(m)
    if e is None: info.update(reason='NO_EDGE'); return 0., info
    info.update(edge=e)
    if e['x'] >= X_TARGET: info.update(reason='CLEAR', off_us=0.); return 0., info
    off = -min(CAP_US, K_US * (X_TARGET - e['x']) / 320.)
    info.update(reason='LEFT', off_us=round(off, 1)); return off, info
