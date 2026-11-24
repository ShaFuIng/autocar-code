"""Curve feed-forward floor on the flat lap (curve-ff-r1 / r2).

User 2026-11-06 (c68f9a58, new steering linkage, kp 155): in the right curve the car sat on the outside (error +0.32 .. +0.89)
while the command stayed 1640-1827; the PID output is centre + right_gain x (P + I + D) with P capped at kp (error clipped to 1),
so it cannot ask for more than about 1800-1830 although the steering now reaches 2050. Lowering kp calmed the straights but took
the curve authority away. This module adds a floor only in curves: when the camera error is at least on_err to the right, the
command is raised to at least centre + frac x max_frac x (max_us - centre), frac growing from 0 at on_err to 1 at full_err; it
stays armed until the error falls below off_err. Left curves only with right_only false. Straights (|error| < on_err) are
untouched. Values come from curve_ff.json, read at each run start. Experiment values, closed loop not verified.

r2 (user 2026-11-06, live 4b534c7d / f67f1b69): the r1 floor armed on the image error alone, so a diagonal drift on a straight
(error >= 0.5 with the car NOT turning) was yanked back to the lock in 0.6 s and overshot to the other line: a sawtooth, not a
curve. A right curve shows up in two signals a diagonal straight does not have: the car is rotating right (yaw rate, low-passed
over yaw_tau_s, <= -yaw_on deg/s; left rotation is positive like gyro_damp) or the image path is bent to the right (the angle
between the far half and the near half of the path <= -bend_on deg; a straight ground line stays straight in the image). The
floor now arms only when the error is large AND (the path is bent that way, OR the car is rotating that way AND the error did
not come from a big excursion on the other side within swing_s: a swing that crossed the track from beyond swing_err is the
sawtooth overshoot, and the car rotates right during it too, so yaw alone cannot tell it apart). The gate is checked every frame
while the error is above on_err, so a curve entry that starts with a straight path arms as soon as the car begins to turn. Once
armed it stays on the error hysteresis as before. The bend is unreliable when only one line is visible (the path is then a
straight-line fit), which is why yaw is kept as the second way in.
"""
import json, math
from pathlib import Path

REVISION = 'curve-ff-r2'
BY_CURVE_KEYS = ('on_err', 'full_err', 'off_err', 'max_frac')   # curve-ff-bycurve-r1
CONFIG_FILE = Path(__file__).with_name('curve_ff.json')
DEFAULTS = dict(enabled=False, on_err=.5, full_err=.85, off_err=.4, max_frac=.94, right_only=True,
                gate=True, yaw_on=5., yaw_tau_s=.5, bend_on=15., min_path_points=8, swing_err=.45, swing_s=2.5)


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


def path_bend_deg(path, min_points=8):
    """Angle (deg) between the far half and the near half of the image path (x right, y down, rows ascending = nearer).
    Negative = the path bends to the right going away from the car. None when the path is too short."""
    try:
        P = sorted(((float(q[0]), float(q[1])) for q in path), key=lambda q: q[1])
    except (TypeError, ValueError, IndexError): return None
    if len(P) < max(4, int(min_points)): return None
    n = len(P); far, near = P[:n // 2], P[n // 2:]
    def slope(seg):
        ys = [q[1] for q in seg]; xs = [q[0] for q in seg]; my = sum(ys) / len(ys); mx = sum(xs) / len(xs)
        d = sum((y - my) ** 2 for y in ys)
        return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / d if d else 0.
    return math.degrees(math.atan(slope(far)) - math.atan(slope(near)))


class CurveFF:
    def __init__(self, config=None):
        self.c = dict(config) if config is not None else load_config(); self.side = 0; self.yaw_lp = None; self.t = None; self.last_opp = {1: None, -1: None}

    def _yaw(self, now, gz):
        if gz is None or not math.isfinite(float(gz)): return self.yaw_lp
        gz = float(gz); dt = 0. if self.t is None else max(0., min(.5, now - self.t)); self.t = now
        tau = max(1e-3, float(self.c.get('yaw_tau_s') or .5))
        self.yaw_lp = gz if self.yaw_lp is None else self.yaw_lp + (1. - math.exp(-dt / tau)) * (gz - self.yaw_lp)
        return self.yaw_lp

    def step(self, steer_us, error, center_us, min_us, max_us, eligible, now=None, gz=None, path=None, curve=None):
        """Returns (steer, info or None). r2: `now`, `gz` (deg/s, left positive) and `path` (image points) feed the curve gate.
        curve-ff-bycurve-r1: `curve` = lap_pos segment name; by_curve[curve] replaces on_err / full_err / off_err / max_frac for this frame."""
        c = self.c; bc = None
        if curve is not None and isinstance(c.get('by_curve'), dict) and isinstance(c['by_curve'].get(curve), dict):
            bc = {k: v for k, v in c['by_curve'][curve].items() if k in BY_CURVE_KEYS and isinstance(v, (int, float)) and not isinstance(v, bool)}
            if bc: c = dict(c, **bc)
            else: bc = None
        yaw = self._yaw(now if now is not None else 0., gz) if c.get('gate') else None
        if not c.get('enabled'): return float(steer_us), None
        if not eligible or error is None: self.side = 0; return float(steer_us), None
        e = float(error); on, full, off = float(c['on_err']), float(c['full_err']), float(c['off_err'])
        tnow = now if now is not None else 0.; sw = float(c.get('swing_err', .45))
        if e >= sw: self.last_opp[-1] = tnow      # a big right-side error is the 'other side' for a later left arming
        if e <= -sw: self.last_opp[1] = tnow      # a big left-side error is the 'other side' for a later right arming
        gate_info = None
        if self.side == 0:
            want = 1 if e >= on else (-1 if (e <= -on and not c.get('right_only')) else 0)
            if want:
                if c.get('gate'):
                    bend = path_bend_deg(path, c.get('min_path_points', 8)) if path is not None else None
                    yaw_ok = yaw is not None and (-want) * yaw >= float(c['yaw_on'])     # right curve: yaw negative
                    bend_ok = bend is not None and (-want) * bend >= float(c['bend_on'])  # right curve: bend negative
                    lo_ = self.last_opp[want]; swing = lo_ is not None and 0. <= tnow - lo_ <= float(c.get('swing_s', 2.5))
                    gate_info = dict(yaw_lp=None if yaw is None else round(yaw, 2), bend_deg=None if bend is None else round(bend, 1), yaw_ok=yaw_ok, bend_ok=bend_ok, swing=swing)
                    if not (bend_ok or (yaw_ok and not swing)):
                        return float(steer_us), dict(revision=REVISION, side=0, e=round(e, 3), applied=False, reason='GATE_SWING' if swing else 'GATE_NOT_TURNING', gate=gate_info)
                self.side = want
        elif self.side * e < off: self.side = 0
        if self.side == 0: return float(steer_us), None
        a = abs(e); frac = max(0., min(1., (a - on) / max(1e-6, full - on))) * float(c['max_frac'])
        s = float(steer_us)
        if self.side > 0:
            floor = float(center_us) + frac * (float(max_us) - float(center_us)); out = max(s, floor)
        else:
            floor = float(center_us) - frac * (float(center_us) - float(min_us)); out = min(s, floor)
        info = dict(revision=REVISION, side=self.side, e=round(e, 3), floor_us=round(floor, 1), pid_steer_us=round(s, 1), applied=out != s)
        if bc is not None: info['by_curve'] = curve
        if gate_info is not None: info['gate'] = gate_info
        return out, info
