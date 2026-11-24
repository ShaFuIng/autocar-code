"""Heading term from the vision path (heading-term-r1).

User 2026-11-07 (live cf17afe7, kd 60; 24cae5a8 / e1b12e2b, kd 200): the lateral PID sees only the error at the sample row.
A straight weave restarts as a heading offset at near-zero yaw rate (cf17 18.1-19.9 s: error 0 -> -0.45 with gz within
+-3 deg/s), invisible to the lateral error until it has grown; by then the car is already heading back (path slope +0.8 at
19.9 s) while the PID, seeing -0.45, still steers left, and the car crosses to +0.84. The D term (kd 200) approximated the
heading through d(error)/dt with the 0.55 s command-to-yaw lag and camera noise; at kd 60 the amplitude grew (0.39 -> 0.51)
and the period lengthened (2.3 -> 3.3 s). The path slope between the sample row (0.6) and 0.85 of the image height leads the
error change over the next 1-1.5 s with correlation 0.58-0.64 in cf17 (0.38 in 24ca).

heading = -(dx/dy) of the path points with row_lo*H <= y <= row_hi*H (native 1280x720 px; dimensionless, positive = the lane
ahead runs to the right = the car is heading left of the lane), low-passed over tau_s; term = k_us * heading, limited to
+-cap_us, added to the steering before the gyro damp / curve feed-forward (the feed-forward floor still wins in right curves).
Only on normal flat-path frames (same eligibility as curve_ff); other frames reset the filter. Offline replay (K 100, cap 120):
term P95 130-150 us (kd 200's D term P95 204), per-frame jitter P95 18-28 us, mean term on curve_ff frames within +-21 us.
Values from heading_term.json, read at each run start. Experiment values; closed loop not verified.

r2 (user 2026-11-07, live 592237ec, the first r1 run): through the first right curve and its exit (5.6-8.6 s, yaw -22..-34 deg/s,
error +1.0 -> -0.5) the near-path slope read +0.3 .. +2.4 ("lane runs right") while the car was over-rotating RIGHT, so the term
(+30 .. +120 us) held the command 100 us right of the PID's unwind (PID 1284, sent 1404 at 8.6 s); the car kept rotating, the
vision path then jumped (error -0.5 -> +0.92) and the feed-forward locked right again for 1.5 s: the car turned past the curve.
A quadratic tangent at the near end gave the same numbers, so it is not far-path curvature: in a curve the near path (often a
single-line fit plus the prior width) does not represent the car's heading. r2 keeps the term for the straight drift it was
made for and gates it: |gz| (low-passed over gz_tau_s) <= gz_max, |error| <= err_max, |heading| <= h_max (larger values are
degenerate paths); outside the gate the term is 0 and the filter resets (GATE_YAW / GATE_ERROR / GATE_HEADING). In 592237ec
the gate is closed through 5.6-8.6 and 12.0-14.2 s and open in the 4.6-5.4 s drift; in cf17 it is open through the 18-21 s drift.
"""
import json, math
from pathlib import Path

REVISION = 'heading-term-r2'
CONFIG_FILE = Path(__file__).with_name('heading_term.json')
DEFAULTS = dict(enabled=False, k_us=100., cap_us=120., tau_s=.2, row_lo=.6, row_hi=.85, min_points=4,
                gz_max=12., gz_tau_s=.3, err_max=.5, h_max=1.5)


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


def heading_of(path, row_lo=.6, row_hi=.85, min_points=4):
    """-(dx/dy) least-squares slope of the path points between row_lo and row_hi of the image height (None when too few)."""
    if not path: return None
    ys = [float(p[1]) for p in path]
    H = 720. if max(ys) > 400. else 360.
    sel = [(float(p[0]), float(p[1])) for p in path if row_lo * H <= float(p[1]) <= row_hi * H]
    if len(sel) < int(min_points): return None
    mx = sum(x for x, y in sel) / len(sel); my = sum(y for x, y in sel) / len(sel)
    den = sum((y - my) ** 2 for x, y in sel)
    if den <= 0: return None
    h = -sum((x - mx) * (y - my) for x, y in sel) / den
    return h if math.isfinite(h) else None


class HeadingTerm:
    def __init__(self, config=None):
        self.c = dict(config) if config is not None else load_config(); self.lp = None; self.t = None; self.gz_lp = None; self.gz_t = None

    def reset(self): self.lp = None; self.t = None

    def step(self, now, path, steer, lo, hi, eligible=True, gz=None, err=None):
        c = self.c
        if not c.get('enabled'): return steer, None
        if gz is not None and math.isfinite(float(gz)):   # r2: yaw-rate gate, low-passed so one quiet frame inside a curve does not open it
            dg = 0. if self.gz_t is None else max(0., min(.5, now - self.gz_t)); self.gz_t = now
            self.gz_lp = float(gz) if self.gz_lp is None else self.gz_lp + (1. - math.exp(-dg / max(1e-3, float(c['gz_tau_s'])))) * (float(gz) - self.gz_lp)
        if not eligible: self.reset(); return steer, dict(revision=REVISION, reason='NOT_ELIGIBLE')
        if self.gz_lp is not None and abs(self.gz_lp) > float(c['gz_max']): self.reset(); return steer, dict(revision=REVISION, reason='GATE_YAW', gz_lp=round(self.gz_lp, 1))
        if err is not None and abs(float(err)) > float(c['err_max']): self.reset(); return steer, dict(revision=REVISION, reason='GATE_ERROR')
        h = heading_of(path, float(c['row_lo']), float(c['row_hi']), int(c['min_points']))
        if h is None: self.reset(); return steer, dict(revision=REVISION, reason='NO_HEADING')
        if abs(h) > float(c['h_max']): self.reset(); return steer, dict(revision=REVISION, reason='GATE_HEADING', heading=round(h, 3))
        dt = 0. if self.t is None else max(0., min(.5, now - self.t)); self.t = now
        self.lp = h if self.lp is None else self.lp + (1. - math.exp(-dt / max(1e-3, float(c['tau_s'])))) * (h - self.lp)
        cap = float(c['cap_us']); term = max(-cap, min(cap, float(c['k_us']) * self.lp))
        out = max(float(lo), min(float(hi), float(steer) + term))
        return out, dict(revision=REVISION, reason='OK', heading=round(h, 3), heading_lp=round(self.lp, 3), term_us=round(term, 1), pid_steer_us=round(float(steer), 1), steer_us=round(out, 1), gz_lp=None if self.gz_lp is None else round(self.gz_lp, 1))
