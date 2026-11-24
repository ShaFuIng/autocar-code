"""track-pos-r1 (user 2026-11-08): RECORD ONLY - position along the lap from wheel odometry, re-anchored at reliable landmarks.
Nothing here changes steering or PWM; live_pid only writes the result to proposal['track_pos'].

Lap coordinate s (m): 0 = downhill start (DashRoute -> DONE, pitch landmark). Segment table from track_pos.json, built from runs
97ed / 46db (2026-11-08) and the user's measured radii (arc = radius x angle): curve 4 (11-12) 2.275 m x 180 deg, curve 1 (01-02)
1.275 m x 150 deg, curve 2 (03-04) 1.175 m x 210 deg; curve 3 (06-09) has no measured radius (start to the ramp).
Anchors: DONE (s = 0), CLIMBED (ramp), OPEN (upper section); a curve-map FLOOR start within snap_m of a table curve start snaps s to
that start. Before each snap the predicted-minus-anchor error is kept in 'last_anchor' so the accuracy can be checked offline.
Before the first anchor the run start is taken as start_s (flag position)."""
import json
from pathlib import Path

REVISION = 'track-pos-r1'


def load_config(path=None):
    try: return json.loads(Path(path or Path(__file__).with_name('track_pos.json')).read_text())
    except Exception: return {}


class TrackPos:
    def seed(self, plan):
        from ops07_route import seed_track
        return seed_track(self, plan)

    def __init__(self, config=None):
        self.c = config if config is not None else load_config(); self.reset()

    def reset(self):
        self.d0 = None; self.s0 = None; self.last_d = None; self.last_t = None; self.route = None; self.reason = None
        self.anchor = None; self.cand = None; self.anchors = 0; self.lap = 0; self.dbl = 0; self.single = 0

    def _set(self, d, s, name, now):
        pred = None if self.d0 is None else self.s_at(d)
        err = None if pred is None else round(self._wrap(pred - s), 2)
        self.d0 = d; self.s0 = s; self.anchors += 1
        self.anchor = dict(name=name, s=round(s, 2), predicted_s=None if pred is None else round(pred, 2), error_m=err, t=round(now, 2))

    def _wrap(self, x):
        L = float(self.c.get('lap_m', 66.8)); x = x % L
        return x - L if x > L / 2 else x

    def s_at(self, d):
        return (self.s0 + (d - self.d0)) % float(self.c.get('lap_m', 66.8))

    def segment(self, s):
        for sg in self.c.get('segments', []):
            if sg['s0'] <= s < sg['s1']: return sg
        return None

    def step(self, now, dist, route, cm=None, sources=None):
        c = self.c
        if not c.get('enabled'): return None
        if dist is None: return dict(revision=REVISION, reason='NO_ODOMETRY')
        d = float(dist); cm = cm or {}
        if self.last_d is not None and (d < self.last_d - .5 or (self.last_t is not None and now - self.last_t > float(c.get('reset_gap_s', 3.)))): self.reset()
        if self.d0 is None: self.d0 = d; self.s0 = float(c.get('start_s', 17.6)) % float(c.get('lap_m', 66.8)); self.anchor = dict(name='START', s=round(self.s0, 2), t=round(now, 2))
        if route != self.route and self.route is not None:
            if route == 'DONE': self._set(d, 0., 'DOWNHILL_START', now); self.lap += 1
            elif route == 'CLIMBED': self._set(d, float(c['climbed_s']), 'CLIMBED', now)
            elif route == 'OPEN': self._set(d, float(c['open_s']), 'OPEN', now)
        self.route = route
        rs = cm.get('reason')
        if rs == 'FLOOR' and self.reason not in ('FLOOR', 'RELEASED'):   # a curve-map start: a candidate, confirmed once the car really turns
            s_now = self.s_at(d); best = None
            for sg in c.get('segments', []):
                if sg.get('kind') == 'curve' and sg.get('snap', True):
                    e = self._wrap(s_now - sg['s0'])
                    if abs(e) <= float(c.get('snap_m', 2.5)) and (best is None or abs(e) < abs(best[1])): best = (sg, e)
            if best is not None: self.cand = (best[0], d, now)
            else: self.cand = None; self.anchor = dict(self.anchor or {}, unmatched_floor=dict(s=round(s_now, 2), seg=cm.get('seg'), idx=cm.get('idx'), t=round(now, 2)))
        cd = getattr(self, 'cand', None)
        if cd is not None:
            if rs not in ('FLOOR', 'RELEASED') or cm.get('released') == 'FAR_OTHER_WAY': self.cand = None   # false start: dropped
            elif abs(float(cm.get('turned_deg') or 0.)) >= float(c.get('confirm_deg', 30.)):
                d_now = d; self._set(cd[1], cd[0]['s0'], 'CURVE_START_' + cd[0]['name'], cd[2]); self.cand = None
        self.reason = rs
        if sources:
            if 'double' in sources: self.dbl += 1; self.single = 0
            else: self.single += 1; self.dbl = 0
        s = self.s_at(d); sg = self.segment(s); nxt = None
        for cand in sorted([x for x in c.get('segments', []) if x.get('kind') == 'curve'], key=lambda x: (x['s0'] - s) % float(c.get('lap_m', 66.8))):
            if cand is not sg: nxt = cand; break
        self.last_d = d; self.last_t = now
        out = dict(revision=REVISION, record_only=True, s_m=round(s, 2), lap=self.lap, since_anchor_m=round(d - self.d0, 2), anchor=self.anchor,
                   segment=None if sg is None else sg['name'], kind=None if sg is None else sg.get('kind'),
                   progress=None if sg is None else round((s - sg['s0']) / max(1e-3, sg['s1'] - sg['s0']), 2),
                   next_curve=None if nxt is None else nxt['name'], to_next_m=None if nxt is None else round((nxt['s0'] - s) % float(c.get('lap_m', 66.8)), 2),
                   double_frames=self.dbl)
        if sg is not None and sg.get('kind') == 'curve' and sg.get('r_m'): out['radius_m'] = sg['r_m']
        return out
