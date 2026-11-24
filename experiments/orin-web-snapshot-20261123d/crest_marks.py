"""Line up between the two crest marks before the downhill (crest-marks-r1, candidate, offline only).

User 2026-11-04: after the dashed line the car sees nothing it uses, but the top of the downhill has two marks (the thick
white starts of the downhill road's left and right edges on the crest platform); can they be the reference?
Video check (linewidth-20261104/bars.py, 20 eye-checked laps, user-confirmed colouring): about 1 s before the slope the
midpoint of the nearest white line left and right of the car axis was 321-395 px in the laps that went down, 225-323 px in
the laps that arrived on the right edge (axis 338); 3.0-2.5 s before, 8 of 10 failed laps saw only the right mark.

Per frame on the V4 mask (640x360), rows ROWS: the inner edge of the nearest white run (width >= MIN_W, 3-row band) left of
the axis (xl) and right of the axis (xr); a side counts when it is seen on at least MIN_ROWS rows (median x).
 - both sides: mid = (xl + xr) / 2; mid < MID_MIN -> LEFT offset K_US * (MID_MIN - mid) / 320;
 - right side only: xr < XR_MIN -> LEFT offset K_US * (XR_MIN - xr) / 320;
 - otherwise (mid >= MID_MIN, left side only, nothing): no offset.
Offset at most CAP_US, on top of the heading-hold steering (as downhill-approach-r1). Active only on the upper section
after the dashed line was followed, on frames where the lane segment steers on the heading (source heading /
heading_lat), and only after the crest ramp: the run is armed once the pitch reached ARM_PITCH on the upper section
(replay of 47 laps: the ramp 4-2 s before the slope reached 2-3.5 deg; on the straight the pitch stayed 0.5-1.1 deg and a
1 deg gate fired from 10 s before the slope). The arming is cleared when the route leaves OPEN. One-sided on purpose: every failure seen arrived on the right; a right correction has no data.
All thresholds are experiment values from the replay in crest-marks-20261105; not a verified safety limit.
"""
import base64
import numpy as np
import cv2

# Offline r2: require >=4 SAME rows containing a left and right inner edge,
# with the axis strictly between them. Median of row-wise midpoints, never
# midpoint of independent side medians. Disjoint supported sides yield no offset.
REVISION = 'crest-marks-r2.1-common-rows'
AXIS = 338
ROWS = tuple(range(250, 345, 10))
MIN_W, MIN_ROWS = 6, 4
MID_MIN, XR_MIN = 320., 500.
K_US, CAP_US = 400., 200.
ARM_PITCH = 2.2
SOURCES = ('heading', 'heading_lat')


def _runs(row):
    d = np.diff(np.r_[0, row.astype(np.int8), 0]); a = np.flatnonzero(d == 1); b = np.flatnonzero(d == -1)
    return [(int(lo), int(hi - 1)) for lo, hi in zip(a, b) if hi - lo >= MIN_W]


def marks(mask):
    """mask bool 360x640 -> dict(xl, xr, nl, nr, mid) (None where a side is not seen on MIN_ROWS rows)."""
    L, R, pairs = [], [], []
    for y in ROWS:
        rs = _runs(mask[y - 1:y + 2].any(axis=0))
        lft = [hi for lo, hi in rs if (lo + hi) / 2. < AXIS]; rgt = [lo for lo, hi in rs if (lo + hi) / 2. >= AXIS]
        if lft: L.append(max(lft))
        if rgt: R.append(min(rgt))
        if lft and rgt:
            xl, xr = max(lft), min(rgt)
            # r2.1 (2026-11-05): a line under the axis still pairs (the car is on that line, the offset is wanted most);
            # only the same-row pairing of r2 is kept. Left/right runs are disjoint, so xl < xr always holds.
            if xl < xr:
                pairs.append((y, xl, xr))
    xl = float(np.median(L)) if len(L) >= MIN_ROWS else None
    xr = float(np.median(R)) if len(R) >= MIN_ROWS else None
    # Pair on the SAME image row before taking a median. Side medians remain
    # diagnostic/single-side inputs, never a substitute for common support.
    mid = round(float(np.median([(a + b) / 2. for _, a, b in pairs])), 1) if len(pairs) >= MIN_ROWS else None
    return dict(xl=xl, xr=xr, nl=len(L), nr=len(R), mid=mid,
                paired_rows=len(pairs), pairs=[dict(y=y, xl=a, xr=b) for y, a, b in pairs])


def decode(png):
    m = cv2.imdecode(np.frombuffer(base64.b64decode(png), np.uint8), 0)
    if m is None: return None
    if m.shape != (360, 640): m = cv2.resize(m, (640, 360), interpolation=cv2.INTER_NEAREST)
    return m > 0


class CrestMarks:
    """Per run: arming state (pitch reached ARM_PITCH on the current upper section)."""
    def __init__(self): self.armed = False

    def arm(self, route_open, pitch):
        """Every control frame (also frames where the offset is not asked): clear off OPEN, arm at ARM_PITCH."""
        if not route_open: self.armed = False
        elif pitch is not None and float(pitch) >= ARM_PITCH: self.armed = True
        return self.armed

    def offset(self, result, lane_info, route_open, followed, pitch):
        """Left offset (us, <= 0) for this frame and info, or (0., None) when the rule does not apply."""
        if not route_open: self.armed = False; return 0., None
        self.arm(route_open, pitch)
        if not self.armed or not followed or (lane_info or {}).get('reason') != 'OK' or lane_info.get('source') not in SOURCES: return 0., None
        info = dict(revision=REVISION, pitch=None if pitch is None else round(float(pitch), 2))
        png = result.get('mask_png')
        m = decode(png) if png else None
        if m is None: info.update(reason='NO_MASK'); return 0., info
        k = marks(m); info.update(marks=k)
        if k['mid'] is not None:
            if k['mid'] >= MID_MIN: info.update(reason='CENTRED', off_us=0.); return 0., info
            off = -min(CAP_US, K_US * (MID_MIN - k['mid']) / 320.); info.update(reason='LEFT_MID', off_us=round(off, 1)); return off, info
        if k['xl'] is not None and k['xr'] is not None:
            info.update(reason='INSUFFICIENT_COMMON_ROWS', off_us=0.); return 0., info
        if k['xr'] is not None and k['xl'] is None:
            if k['xr'] >= XR_MIN: info.update(reason='RIGHT_FAR', off_us=0.); return 0., info
            off = -min(CAP_US, K_US * (XR_MIN - k['xr']) / 320.); info.update(reason='LEFT_RIGHT_ONLY', off_us=round(off, 1)); return off, info
        info.update(reason='LEFT_ONLY' if k['xl'] is not None else 'NO_MARKS', off_us=0.); return 0., info
