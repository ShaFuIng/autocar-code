"""curve-trim-r1 (curves-20261120 offline candidate, NOT deployed): move the in-curve operating point of curves 1 / 3 outward.

Measured (11-20 runs 42e5 / 5726 / c540 / 93ad / 0bea, curves-20261120/out): in curves 1 and 3 the PID chain has no curvature
feed-forward, so it can only make the ~+200 us the curve needs from a standing camera error (control error ~0.68 in curve 1,
~0.5 in curve 3), and that standing error corresponds to an inside position (arc_lat: rear axle 7-17 cm inside the lane centre,
inner wheel 0-10 cm over the inner line; a centred car would read ~0.9-1.0 in curve 1 = next to the 1.0 clip). Curve 1 also
turns 5-15 % faster than the lane all the way (gz 45-55 vs 40-44 dps).
This layer subtracts a small distance-scheduled trim (us) from the right-curve command inside a lap_pos window (anchor C1s / C3s,
metres since the anchor), after lap_ff / arc2_flat and before arc_lat / the speed group: the car turns slightly less, drifts out,
and the camera error rises until the chain has made the trim back (static estimate: +0.07..0.11 cm per us). Same effect as an
error offset of trim / K in the chain (K ~ 300 us per unit error), but it also trims curve_ff and the arc2 c3 floor.
Rules (per curve block in curve_trim.json, read once per run; file absent / enabled not true / curve enabled not true = not run):
  - active only while lap_pos is not lost, its anchor is the curve's anchor and since_anchor_m is inside the table range;
  - trim = interp(table, x) us (>= 0); never takes the command below centre + min_rel_us (right curves only; a command already
    at or left of that line is untouched);
  - released for the rest of the pass when the near error reaches e_release (the car is already outside); after a release the
    trim fades out at fade_us per frame (no step);
  - eligibility = lap_ff's (flat, normal path, no downhill / crossing window) and no arc2 crossing-window ownership.
No clock / IMU / file read inside step(). Experiment values, closed loop not verified."""
import json
from pathlib import Path

REVISION = 'curve-trim-r1'
CONFIG_FILE = Path(__file__).with_name('curve_trim.json')
CURVE_DEFAULTS = dict(enabled=False, anchor=None, table=[], e_release=0.9, min_rel_us=60., fade_us=6.)


def _num(x):
    try: return x is not None and float(x) == float(x) and abs(float(x)) != float('inf')
    except (TypeError, ValueError): return False


def interp(tbl, x):
    if x <= tbl[0][0]: return float(tbl[0][1])
    for (x0, y0), (x1, y1) in zip(tbl, tbl[1:]):
        if x <= x1: return float(y0) + (float(y1) - float(y0)) * (x - float(x0)) / max(1e-9, float(x1) - float(x0))
    return float(tbl[-1][1])


def load_config(path=None):
    """dict or None (absent / broken / enabled not true / no usable enabled curve)."""
    try: c = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
    except (OSError, ValueError): return None
    if not isinstance(c, dict) or c.get('enabled') is not True or not isinstance(c.get('curves'), dict): return None
    cur = {}
    for k, v in c['curves'].items():
        if not isinstance(v, dict) or v.get('enabled') is not True: continue
        m = dict(CURVE_DEFAULTS); m.update(v)
        t = m.get('table')
        if not m.get('anchor') or not isinstance(t, list) or len(t) < 2: continue
        try: t = [[float(a), max(0., float(b))] for a, b in t]
        except (TypeError, ValueError): continue
        if any(b[0] <= a[0] for a, b in zip(t, t[1:])): continue
        m['table'] = t; cur[k] = m
    return dict(c, curves=cur) if cur else None


class CurveTrim:
    def __init__(self, cfg):
        self.c = cfg; self.curves = cfg['curves']; self.rel = {}; self.last = {}

    def step(self, lp, near, steer, centre, lo, hi, eligible):
        """lp: lap_pos state (dict lap, anchor, anchor_d, since_anchor_m, lost). Returns (steer, info or None); info only inside a window."""
        st = float(steer)
        if not lp or lp.get('lost') or not _num(lp.get('since_anchor_m')): return st, None
        for name, cc in self.curves.items():
            if lp.get('anchor') != cc['anchor']: continue
            x = float(lp['since_anchor_m']); t = cc['table']
            if x < t[0][0] or x > t[-1][0]: return st, None
            key = (name, lp.get('lap'), None if lp.get('anchor_d') is None else round(float(lp['anchor_d']), 3))
            d = interp(t, x); why = None
            if key in self.rel:
                d = max(0., self.last.get(key, 0.) - float(cc['fade_us'])); why = 'RELEASED'
            elif _num(near) and float(near) >= float(cc['e_release']):
                self.rel[key] = True; d = max(0., self.last.get(key, 0.) - float(cc['fade_us'])); why = 'RELEASE'
            if not eligible: d = 0.; why = 'NOT_ELIGIBLE'
            floor = float(centre) + float(cc['min_rel_us'])
            out = st if st <= floor else max(floor, st - d)
            out = max(float(lo), min(float(hi), out))
            self.last[key] = d
            info = dict(revision=REVISION, curve=name, x_m=round(x, 3), trim_us=round(d, 1), pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=bool(out != st))
            if why: info['state'] = why
            return out, info
        return st, None


def build(path=None):
    c = load_config(path)
    return CurveTrim(c) if c else None
