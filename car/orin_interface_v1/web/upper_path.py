"""upper-path-r1 (user 2026-11-08): on the upper section (route OPEN, from the crest) follow the recorded heading profile by
odometry. 10 upper sections (7fbf/ecda/3a65/d763/30e0): length 7.10-7.35 m, the dashed left turn passes 60 deg at 3.39-3.83 m,
end heading 78-102 deg. Steering = (1 - w_cam) * path + w_cam * camera, path = recorded steering (turn window only, else centre)
- k_head * (h_ref - h) + k_rate * (gz - r_ref); h = gyro yaw since entering OPEN (deg, + = left), s = mean wheel distance.
Off when disabled, outside OPEN or past s_max_m; frames that do not reach here (blind holds) keep the existing behaviour."""
import json, math
from pathlib import Path
try: import single_centre as _SC   # single-centre-r1: recorded steer_us table follows the shared centre when single_centre.json is on
except Exception: _SC = None
REVISION = 'upper-path-r2'
def load_config(path=None):
    try: return json.loads(Path(path or Path(__file__).with_name('upper_path.json')).read_text())
    except Exception: return {}
def interp(xs, ys, x):
    if x <= xs[0]: return float(ys[0])
    if x >= xs[-1]: return float(ys[-1])
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if xs[m] <= x: lo = m
        else: hi = m
    f = (x - xs[lo]) / (xs[hi] - xs[lo]); return float(ys[lo]) + f * (float(ys[hi]) - float(ys[lo]))
class UpperPath:
    def __init__(self, config=None):
        self.c = dict(config) if config is not None else load_config(); self.reset()
    def reset(self): self.s0 = None; self.h = 0.; self.t = None; self.gz = None; self.d = None; self.v = 0.
    def _dist(self, vehicle):
        try:
            a = vehicle['enc_a']; b = vehicle['enc_b']
            return ((b['forward'] - b['reverse']) + (a['reverse'] - a['forward'])) / 2. * 2.22e-4
        except Exception: return None
    def step(self, now, route, vehicle, gz, cam_steer, center, lo, hi, err=None):
        c = self.c
        if not c.get('enabled') or not c.get('grid_m'): return None, None
        if route != 'OPEN': self.reset(); return None, None
        d = self._dist(vehicle or {})
        if d is None: return None, dict(revision=REVISION, reason='NO_ODOMETRY')
        g = None if gz is None or not math.isfinite(float(gz)) else float(gz)
        if self.s0 is None: self.s0 = d; self.t = now; self.d = d; self.gz = g; self.h = 0.
        else:
            dt = max(0., min(.5, now - self.t))
            g0 = self.gz if self.gz is not None else g
            if g is not None and g0 is not None: self.h += (g + g0) / 2. * dt
            if dt > 0: self.v = .6 * self.v + .4 * max(0., (d - self.d) / dt)
            self.t = now; self.d = d
            if g is not None: self.gz = g
        s = d - self.s0
        if s > float(c.get('s_max_m', 7.6)): return None, dict(revision=REVISION, reason='PAST_END', s_m=round(s, 2))
        xs, H, U = c['grid_m'], c['heading_deg'], c['steer_us']
        h_ref = interp(xs, H, s); dh_ds = (interp(xs, H, s + .2) - interp(xs, H, s - .2)) / .4; r_ref = dh_ds * self.v
        ff = interp(xs, U, s) if float(c.get('turn_from_m', 2.2)) <= s <= float(c.get('turn_to_m', 4.6)) else float(center)
        if _SC is not None and _SC.active() and float(c.get('turn_from_m', 2.2)) <= s <= float(c.get('turn_to_m', 4.6)): ff = _SC.shift(ff)   # single-centre-r1 (off: not run)
        eh = h_ref - self.h; th = max(-float(c['cap_head_us']), min(float(c['cap_head_us']), float(c['k_head_us_per_deg']) * eh))
        tr = 0. if self.gz is None else max(-float(c['cap_rate_us']), min(float(c['cap_rate_us']), float(c['k_rate_us_per_dps']) * (self.gz - r_ref)))
        path = ff - th + tr; w = float(c.get('w_cam', .3))
        if err is not None and c.get('cam_full_err') is not None:   # r2: a large camera error (crooked entry) gives the camera back its authority
            e0 = float(c.get('cam_start_err', .25)); e1 = float(c['cam_full_err']); a_ = min(1., max(0., (abs(float(err)) - e0) / max(1e-3, e1 - e0))); w = w + (1. - w) * a_
        out = max(float(lo), min(float(hi), (1. - w) * path + w * float(cam_steer)))
        return out, dict(revision=REVISION, reason='OK', s_m=round(s, 2), h_deg=round(self.h, 1), h_ref_deg=round(h_ref, 1), r_ref_dps=round(r_ref, 1),
                         ff_us=round(ff, 1), w_cam=round(w, 2), head_us=round(-th, 1), rate_us=round(tr, 1), camera_us=round(float(cam_steer), 1), steer_us=round(out, 1), v_mps=round(self.v, 2))
