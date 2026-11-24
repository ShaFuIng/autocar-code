"""Curve-exit damping (exit-damp-r1). Pure logic, standard library only.

User 2026-11-13 (763c / 8e39 / b44b, centre 1480): after curve 4 (curve_map after_down idx 0) the map releases at about 120 deg
(ANGLE_DONE) and the car then weaves on what is already a straight: period about 1.1 s, heading +-3..6 deg, command 1250 <-> 1870,
for about 5 s (11 of 14 curve-4 exits today). Per command half cycle the reversal is made of the yaw_inner term (median 39 %),
the play-compensation flip (31 %) and the PID D term (19 %); P is small (7 %). Inside the crossing window after curve 2 the
inner-loop term is already limited to +-term_cap_us (crossing_pass.json, layer xp_term_cap) and the car does not weave there.

This module decides, per frame, whether the frame is in an exit scope and which caps apply; live_pid applies them right after the
yaw_inner layer with the same mechanics as the crossing window's cap (command = yaw_inner.pid_steer_us + capped term).

Scopes (only while |pitch| <= level_pitch_deg, i.e. level ground):
  exit: from the first curve_map RELEASED frame of a curve listed in `curves` (released reason in `release_reasons`) for post_s
        seconds; any curve_map FLOOR frame (the next curve) ends it.
  c1:   optional (c1_window {start_m, end_m}); start_m..end_m metres of curve_map odometry after curve 1's CURVE_END (the
        diagonal V crossing between curve 1 and curve 2). Distance only, a FLOOR does not end it (as crossing-pass-r8 2026-11-11).
Caps: term_cap_us on the yaw_inner term (both scopes); d_cap_us (null = off) on the PID D contribution (exit scope; also c1 when
c1_d_cap is true).

Values from exit_damp.json, read once per run in LivePID.start() (edit -> next run, no service restart). enabled false = the
layer does not run (bit-identical to b5). Experiment values; closed loop not verified.
"""
import json, math
from pathlib import Path

REVISION = 'exit-damp-r1'
CONFIG_FILE = Path(__file__).with_name('exit_damp.json')
DEFAULTS = dict(enabled=False, post_s=6.0, term_cap_us=40., d_cap_us=None, curves=[1, 2, 4], release_reasons=['ANGLE_DONE'],
                level_pitch_deg=3.0, c1_window=None, c1_d_cap=False)
# curve_map (seg, idx) -> track curve number (AGENTS.md map: start idx0/1/2 = curve 1/2/3; after_down idx0 = curve 4, idx1/2/3 = curve 1/2/3)
CURVE = {('start', 0): 1, ('start', 1): 2, ('start', 2): 3, ('after_down', 0): 4, ('after_down', 1): 1, ('after_down', 2): 2, ('after_down', 3): 3}


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


class ExitDamp:
    def __init__(self, config=None):
        self.c = dict(DEFAULTS); self.c.update(dict(config) if config is not None else load_config())
        self.on = bool(self.c.get('enabled'))
        self.t0 = None; self.curve = None; self.armed = None; self.c1_d0 = None

    def scope(self, now, cmi, pitch):
        """Advance the scope state with this frame's curve_map info and return (scope, info) - scope None outside."""
        c = self.c; cmi = cmi or {}; rs = cmi.get('reason'); key = (cmi.get('seg'), cmi.get('idx')); cv = CURVE.get(key)
        if rs == 'FLOOR': self.t0 = None; self.curve = None; self.armed = None   # the next curve starts: exit scope over
        elif rs == 'CURVE_END':
            self.armed = None
            if cv == 1 and _num(cmi.get('dist_m')): self.c1_d0 = float(cmi['dist_m'])
        elif rs == 'RELEASED' and self.armed != key:
            self.armed = key   # one start per curve instance (RELEASED repeats until CURVE_END)
            if cv in (c.get('curves') or ()) and cmi.get('released') in (c.get('release_reasons') or ()): self.t0 = now; self.curve = cv
        level = _num(pitch) and abs(float(pitch)) <= float(c['level_pitch_deg'])
        sc = None; info = None
        if self.t0 is not None:
            el = now - self.t0
            if el > float(c['post_s']): self.t0 = None; self.curve = None
            elif level: sc = 'exit'; info = dict(curve=self.curve, since_s=round(el, 2))
        w = c.get('c1_window')
        if sc is None and isinstance(w, dict) and self.c1_d0 is not None and _num(cmi.get('dist_m')):
            dd = float(cmi['dist_m']) - self.c1_d0
            if dd > float(w.get('end_m', 6.5)): self.c1_d0 = None
            elif dd >= float(w.get('start_m', 3.0)) and level: sc = 'c1'; info = dict(curve=1, dist_after_m=round(dd, 2))
        return sc, info

    def apply(self, sc, info, steer, yii, pid, rp, xp_capped=None):
        """Caps for a frame in scope sc. steer = the command after yaw_inner / xp_term_cap; yii = yaw_inner info (or None);
        pid = dict(p, i, d, k) with k = gain-schedule factor; rp = run parameters (center_us, min_us, max_us, right_gain, left_gain).
        xp_capped = the crossing window's capped term this frame (it keeps ownership when it is not larger than ours).
        Returns (new steer, log dict)."""
        c = self.c; lo, hi = float(rp['min_us']), float(rp['max_us']); out = float(steer)
        log = dict(revision=REVISION, scope=sc, steer_in_us=round(out, 1)); log.update(info or {})
        tc = c.get('term_cap_us')
        ok = isinstance(yii, dict) and yii.get('reason') == 'OK' and yii.get('mode') is None and _num(yii.get('term_us')) and _num(yii.get('pid_steer_us'))
        if tc is not None and ok:
            t = float(yii['term_us']); log['term_us'] = round(t, 1)
            if xp_capped is not None and abs(float(xp_capped)) <= float(tc): log['term_by'] = 'xp_term_cap'
            elif abs(t) > float(tc):
                tcx = math.copysign(float(tc), t); out = max(lo, min(hi, float(yii['pid_steer_us']) + tcx)); log['term_capped_us'] = round(tcx, 1)
        dc = c.get('d_cap_us')
        if dc is not None and ok and (sc == 'exit' or c.get('c1_d_cap')) and isinstance(pid, dict) and all(_num(pid.get(k)) for k in ('p', 'i', 'd')):
            p_, i_, d_ = float(pid['p']), float(pid['i']), float(pid['d']); log['d'] = round(d_, 1)
            if abs(d_) > float(dc) and not pid.get('limited'):
                k = float(pid['k']) if _num(pid.get('k')) else 1.; c0 = float(rp['center_us'])
                def raw(u): return c0 + u * (float(rp.get('right_gain', 1.)) if u >= 0 else float(rp.get('left_gain', 1.)))
                dcx = math.copysign(float(dc), d_); delta = k * (raw(p_ + i_ + dcx) - raw(p_ + i_ + d_))
                out = max(lo, min(hi, out + delta)); log['d_capped'] = round(dcx, 1); log['d_delta_us'] = round(delta, 1)
            elif abs(d_) > float(dc): log['d_skip'] = 'PID_RATE_LIMITED'
        log['steer_us'] = round(out, 1)
        return out, log
