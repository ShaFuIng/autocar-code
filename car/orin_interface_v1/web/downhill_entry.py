"""Downhill entry after the two-lane section (downhill-entry-r1).

User 2026-11-03, live 20aa810f (the downhill is one big right curve):
 1. 158.0-158.9 s the image had no target for 0.8 s; heading-hold's blind step pulled the car back to the two-lane
    heading (1290 -> 1136, left) although the lane segment had just started the right curve (1700-1950).
    -> blind_hold(): for up to BLIND_HOLD_S after the last lane-segment output, keep the mean lane command of the
       last LANE_MEAN_S instead of the heading pull-back.
 2. 159.3-160.1 s the only line seen was the near outer (left) edge of the right curve; the path took it as the
    RIGHT boundary (source 'right+width') and put the centre one road width to its left: error -1.0, 1120 us for
    ~1 s, yaw +52 deg left.
    -> mirror(): in the entry window (lane segment used within ENTRY_S and the downhill latch at most LATCH_S old),
       pitched down by more than 2 deg, a path whose only source is 'right+width' is mirrored about that boundary (x' = 2 * x_R - x, same rows),
       i.e. the line is taken as the left boundary. Mid-slope single right lines are left alone: in c933191a
       167.9-169.0 s (latched 1.6-2.7 s) such a path correctly pointed left.
r2 (live 952133fa, 2026-11-03): r1 never mirrored. The slope started 4 s after the last lane output (> ENTRY_S) and a
crest-edge bump (pitch -6.1 at 139.4 s) latched the downhill 3 s early (> LATCH_S); at 143.2-144.2 s the lone
'right+width' path again gave -1.0 / 1037 us and the car turned 100 deg left off the ramp.
    -> the window is now timed from the real slope start: past the dashed section (dash route DONE) or lane segment
       used within ENTRY_S, pitched down < PITCH_DOWN, and at most SLOPE_S after the pitch first went below
       SLOPE_START (-8 deg; the bump reached -6.1). Not yet below -8 counts as inside the window. The slope start
       resets after the pitch is back above PITCH_DOWN for RESET_S.
    Needed: 20aa810f 0.84 s, 952133fa 1.42 s after the -8 deg crossing; c933191a's correct left single-right path
    starts ~1.6 s after -> SLOPE_S 1.5 leaves a small margin.
Window lengths are experiment values from these runs.
"""

REVISION = 'downhill-entry-r3'
BLIND_HOLD_S, LANE_MEAN_S, ENTRY_S = 1.0, 1.0, 3.0   # 1 s mean: 20aa810f last 0.5 s was a 1950 spike, PID ~1770
PITCH_DOWN = -2.   # mirror only once pitched down (in the two-lane section a lone right line is the real right edge)
SLOPE_START, SLOPE_S, RESET_S = -8., 1.5, 1.0
# r3 (user 2026-11-04: the first big right curve of the downhill often presses the inner line). On the downhill the real lane
# width at the control rows is narrower than the width the single-line estimate uses: rows 400-432 (1280x720) measured 683-776 px
# on two-line frames of 8d889766 / bb013bf3 / c28b751e / 952133fa (1f3d7ba7 836-926), the 'left+width' paths implied 900-1114 px
# (flat ground 870-966). The far rows, where the width ratio is learnt, stay as wide as on flat ground. -> on the downhill a
# path from a single line keeps WIDTH_K of its distance from that line (median measured / implied). Experiment value.
WIDTH_K = .75
NARROW_SOURCE = {'left+width': 'L', 'right+width': 'R'}


def narrow(result):
    """Returns (path, info) with a single-line path moved towards its line by WIDTH_K, or None."""
    le = ((result.get('path_diagnostic') or {}).get('line_estimate') or {})
    src = set((le.get('sources') or {}).keys())
    if len(src) != 1 or next(iter(src)) not in NARROW_SOURCE: return None
    side = NARROW_SOURCE[next(iter(src))]
    bx = {int(round(y)): float(x) for x, y in ((((le.get('offscreen_center') or {}).get('current_boundaries') or {}).get(side)) or [])}
    path = result.get('path') or []
    out = [[bx[int(round(p[1]))] + WIDTH_K * (float(p[0]) - bx[int(round(p[1]))]), float(p[1])] for p in path if int(round(p[1])) in bx]
    if len(out) < max(3, len(path) // 2): return None
    return out, dict(revision=REVISION, reason='DOWNHILL_WIDTH', side=side, k=WIDTH_K, points=len(out))


class DownhillEntry:
    def __init__(self):
        self.lane = []          # (t, steer_us) lane-segment outputs
        self.blind_t0 = None
        self.slope_t0 = None    # first pitch < SLOPE_START of this slope
        self.up_t0 = None       # pitch back above PITCH_DOWN since

    def note_lane(self, now, steer_us):
        self.lane.append((now, float(steer_us)))
        self.lane = [x for x in self.lane if now - x[0] <= ENTRY_S]
        self.blind_t0 = None

    def note_pitch(self, now, pitch):
        if pitch is None: return
        if pitch < SLOPE_START and self.slope_t0 is None: self.slope_t0 = now
        if pitch >= PITCH_DOWN:
            if self.up_t0 is None: self.up_t0 = now
            if now - self.up_t0 >= RESET_S: self.slope_t0 = None
        else: self.up_t0 = None

    def _lane_recent(self, now, span):
        return bool(self.lane) and now - self.lane[-1][0] <= span

    def blind_hold(self, now):
        if not self._lane_recent(now, BLIND_HOLD_S): self.blind_t0 = None; return None
        if self.blind_t0 is None: self.blind_t0 = now
        if now - self.blind_t0 > BLIND_HOLD_S: return None
        last = self.lane[-1][0]; xs = [s for t, s in self.lane if last - t <= LANE_MEAN_S]
        steer = sum(xs) / len(xs)
        return steer, dict(revision=REVISION, reason='LANE_HOLD', steer_us=round(steer, 1), n=len(xs),
                           blind_s=round(now - self.blind_t0, 3), lane_age_s=round(now - last, 3))

    def mirror(self, now, result, pitch, route_done):
        if pitch is None or pitch >= PITCH_DOWN: return None
        if not (route_done or self._lane_recent(now, ENTRY_S)): return None
        if self.slope_t0 is not None and now - self.slope_t0 > SLOPE_S: return None
        le = ((result.get('path_diagnostic') or {}).get('line_estimate') or {})
        if set((le.get('sources') or {}).keys()) != {'right+width'}: return None
        R = (((le.get('offscreen_center') or {}).get('current_boundaries') or {}).get('R')) or []
        rx = {int(round(y)): float(x) for x, y in R}
        path = result.get('path') or []
        out = [[2. * rx[int(round(p[1]))] - float(p[0]), float(p[1])] for p in path if int(round(p[1])) in rx]
        if len(out) < max(3, len(path) // 2): return None
        return out, dict(revision=REVISION, reason='MIRROR_SINGLE_RIGHT', points=len(out),
                         slope_s=None if self.slope_t0 is None else round(now - self.slope_t0, 3))
