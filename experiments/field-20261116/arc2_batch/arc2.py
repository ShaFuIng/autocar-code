"""arc2-r1 (arc2-20261116, offline candidate, NOT deployed): three position-anchored steering owners, each keyed in arc2.json
(file absent / enabled false / key absent = layer not built = live_pid frame-identical).

land   (item A) landing -> curve-4 entry as ONE profile owner on valid frames, from the lap_pos LAND8 point (DOWN + 2.32 m fallback).
       centre(x) = max(straight_us, c_eff + kappa(x + v*lead_s) / g(v)) + clip(k_head * (turn_ref(x) - turned), +-head_lim)
       out = clip(PID, centre - m_lo * y, centre + hi_m(x))   (y = inside-yield 1 at near >= e_hi .. 0 at near <= e_lo, rate limited)
       hi_m = pre_hi before the bend start, m_hi after; after x_end the band fades over hand_m (rate limited by max_step_us).
       kappa table / x_bend from the logs (out/land.txt) and the user's tape: lane straight for 0.6-0.7 m after the seam, the rear axle
       passes the seam at LAND8 + ~0.22 m (pitch -2 deg crossing) -> full arc from x_bend 0.85 m; a small early component before it.
       turned = landing heading (_bm_yaw0, 50 Hz IMU) - IMU heading now; no IMU -> no heading term (band still applies).
       Lost frames are not touched (bottom_owner RATE_HOLD / bottom_guide PROFILE_HOLD keep them).
blind  (item D) V-crossing on the diagonal after curve 1: lap_pos C1e + [x0, x1] m. Heading reference = mean IMU heading over
       [ref_from, x0) (averages the post-curve-1 weave); command = straight_us + k_psi*(yaw - ref) + k_gz*gz + clip(k_cam * e_lp, +-trim_us),
       e_lp = low-passed near error, frames with |de| >= jump_e are not fed to the filter; limited to straight_us +- lim_us,
       blended in over in_m and out over out_m, per-frame change <= max_step_us. PWM untouched.
c3     (item C) curve-3 piecewise floor: lap_ff.LapFF4 instance (name 'c3', anchor C3s, heading reference at the anchor crossing,
       kappa table = smoothed measured turn-vs-distance of 70 passes, pitch end before the ramp, inside-yield). Built by live_pid.
Standard library only; no clock / file reads except load_config()."""
import json, math
from pathlib import Path

REVISION = 'arc2-r1'
CONFIG_FILE = Path(__file__).with_name('arc2.json')

LAND_DEFAULTS = dict(enabled=False, anchors=[['LAND8', 0.0], ['DOWN', 2.32]], win_m=[-0.1, 0.6], max_age_s=0.25, rearm_m=20.,
                     straight_us=1513., c_eff_us=1490., gain_table=[[0.80, 0.192], [0.93, 0.167], [1.06, 0.159], [1.22, 0.150], [1.37, 0.130]],
                     v_default=0.95, lead_s=0.25,
                     kappa_table=[[0.0, 5.0], [0.75, 5.0], [0.95, 30.0], [1.5, 30.0], [1.8, 24.0], [99.0, 24.0]], x_bend=0.85,
                     turn_ref=[[0.0, 0.0], [0.85, 2.0], [1.5, 21.0], [2.0, 36.0], [2.5, 48.0], [3.0, 60.0]],
                     k_head=4.0, head_lim=80., pre_lo=None, m_lo=90., m_hi=90., pre_hi=150., e_lo=0.0, e_hi=0.25, y_rate=0.25,
                     x_end=2.0, hand_m=0.5, max_step_us=60.)
BLIND_DEFAULTS = dict(enabled=False, anchor='C1e', ref_from=4.8, x0=6.0, x1=8.5, in_m=0.3, out_m=0.5, straight_us=1513.,
                      k_psi=10., k_gz=1.5, k_cam=100., trim_us=50., tau_s=0.5, jump_e=0.25, lim_us=150., max_step_us=60., max_age_s=0.25)


def load_config(path=None):
    p = Path(path) if path else CONFIG_FILE
    try: raw = json.loads(p.read_text(encoding='utf8'))
    except (OSError, ValueError): return None
    if not isinstance(raw, dict) or not raw.get('enabled'): return None
    return raw


def _num(x): return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


def interp(t, x):
    t = sorted((float(a), float(b)) for a, b in t)
    if x <= t[0][0]: return t[0][1]
    if x >= t[-1][0]: return t[-1][1]
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x0 <= x <= x1: return y0 + (y1 - y0) * (x - x0) / max(1e-9, x1 - x0)
    return t[-1][1]


def _sub(raw, key, defaults):
    s = (raw or {}).get(key)
    if not isinstance(s, dict) or not s.get('enabled'): return None
    c = dict(defaults); c.update(s); return c


class LandArc:
    REV = 'arc2-land-r1'

    def __init__(self, c): self.c = c; self.act = None; self.last_d0 = None; self.last = None; self.y = 1.

    @classmethod
    def from_config(cls, raw):
        c = _sub(raw, 'land', LAND_DEFAULTS); return cls(c) if c else None

    def _tail(self, st, lo, hi):
        if self.last is None: return st, None
        md = float(self.c['max_step_us'])
        if abs(st - self.last) <= md: self.last = None; return st, None
        out = self.last + max(-md, min(md, st - self.last)); out = max(float(lo), min(float(hi), out)); self.last = out
        return out, dict(revision=self.REV, state='TAIL', pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=True)

    def step(self, lp, near, v, yaw_imu, yaw_land, steer, lo, hi, eligible, now=None):
        c = self.c; st = float(steer)
        if not eligible: return st, None   # lost / non-normal frames: untouched (state kept)
        if self.act is None:
            if not lp or lp.get('lost') or not _num(lp.get('since_anchor_m')) or not _num(lp.get('odo_m')): return self._tail(st, lo, hi)
            if now is not None and _num(lp.get('t')) and float(now) - float(lp['t']) > float(c['max_age_s']): return self._tail(st, lo, hi)
            gaps = {str(a): float(g) for a, g in c['anchors']}
            if lp.get('anchor') not in gaps: return self._tail(st, lo, hi)
            x = float(lp['since_anchor_m']) - gaps[lp['anchor']]; D = float(lp['odo_m'])
            if not (float(c['win_m'][0]) <= x <= float(c['win_m'][1])) or (self.last_d0 is not None and D - self.last_d0 <= float(c['rearm_m'])): return self._tail(st, lo, hi)
            self.act = dict(d_land=D - x, anchor=lp['anchor']); self.last_d0 = D; self.y = None   # first frame: y = target (no rate limit from 1)
        A = self.act
        if not lp or lp.get('lost') or not _num(lp.get('odo_m')):
            self.act = None; o, _ = self._tail(st, lo, hi)
            return o, dict(revision=self.REV, state='END', reason='LAP_POS_LOST', steer_us=round(o, 1), applied=o != st)
        x = float(lp['odo_m']) - A['d_land']
        if x > float(c['x_end']) + float(c['hand_m']):
            self.act = None; o, _ = self._tail(st, lo, hi)
            return o, dict(revision=self.REV, state='END', reason='HANDED_OVER', x_m=round(x, 3), steer_us=round(o, 1), applied=o != st)
        vv = float(v) if _num(v) and float(v) > 0.2 else float(c['v_default'])
        g = interp(c['gain_table'], vv); kap = interp(c['kappa_table'], x + vv * float(c['lead_s']))
        base = max(float(c['straight_us']), float(c['c_eff_us']) + kap / g)
        turned = (float(yaw_land) - float(yaw_imu)) if _num(yaw_land) and _num(yaw_imu) else None
        head = 0.
        if turned is not None and x >= 0.: head = max(-float(c['head_lim']), min(float(c['head_lim']), float(c['k_head']) * (interp(c['turn_ref'], x) - turned)))
        cen = base + head
        ytg = 1. if not _num(near) else max(0., min(1., (float(near) - float(c['e_lo'])) / max(1e-3, float(c['e_hi']) - float(c['e_lo']))))
        r = float(c['y_rate']); self.y = ytg if self.y is None else self.y + max(-r, min(r, ytg - self.y))
        lower = cen - float(c['m_lo']) if x >= float(c['x_bend']) or c.get('pre_lo') is None else cen - float(c['pre_lo']); upper = cen + (float(c['pre_hi']) if x < float(c['x_bend']) else float(c['m_hi']))
        lower_eff = st + self.y * (lower - st) if (st < lower and (x >= float(c['x_bend']) or c.get('pre_lo') is not None)) else st   # no floor before the bend unless pre_lo is set
        out = min(max(st, lower_eff), upper)
        a = 1.
        if x > float(c['x_end']): a = max(0., 1. - (x - float(c['x_end'])) / max(1e-3, float(c['hand_m'])))
        out = st + a * (out - st)
        if self.last is not None:
            md = float(c['max_step_us'])
            if a < 1.: out = self.last + max(-md, min(md, out - self.last))   # hand-over: rate-limited both directions
        out = max(float(lo), min(float(hi), out))
        self.last = out if out != st else (self.last if a < 1. and self.last is not None else None)
        return out, dict(revision=self.REV, anchor=A['anchor'], state='OWN' if a >= 1. else 'HAND', x_m=round(x, 3), v=round(vv, 3), kappa=round(kap, 1),
                         base_us=round(base, 1), turned_deg=None if turned is None else round(turned, 1), head_us=round(head, 1), y=round(self.y, 3),
                         lower_us=round(lower_eff, 1), upper_us=round(upper, 1), a=round(a, 3), pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=out != st)


class Blind:
    REV = 'arc2-blind-r1'

    def __init__(self, c): self.c = c; self.ref = []; self.psi = None; self.elp = None; self.ep = None; self.last = None; self.key = None; self.tp = None

    @classmethod
    def from_config(cls, raw):
        c = _sub(raw, 'blind', BLIND_DEFAULTS); return cls(c) if c else None

    def step(self, lp, near, valid, yaw, gz, steer, lo, hi, eligible, now=None):
        c = self.c; st = float(steer)
        if not lp or lp.get('lost') or lp.get('anchor') != c['anchor'] or not _num(lp.get('since_anchor_m')):
            return self._release(st, lo, hi)
        if now is not None and _num(lp.get('t')) and float(now) - float(lp['t']) > float(c['max_age_s']): return self._release(st, lo, hi)
        key = (lp.get('lap'), None if not _num(lp.get('anchor_d')) else round(float(lp['anchor_d']), 2))
        if key != self.key: self.key = key; self.ref = []; self.psi = None; self.elp = None; self.ep = None
        x = float(lp['since_anchor_m'])
        x0, x1 = float(c['x0']), float(c['x1'])
        if x < x0:
            if x >= float(c['ref_from']) and _num(yaw): self.ref.append(float(yaw))
            return self._release(st, lo, hi)
        if x > x1 + float(c['out_m']) or not _num(yaw): return self._release(st, lo, hi)
        if self.psi is None:
            if not self.ref: return self._release(st, lo, hi)
            self.psi = sum(self.ref) / len(self.ref)
        if not eligible: return st, None
        # camera trim: slow low-pass of the near error, jumps ignored
        if valid and _num(near):
            e = float(near)
            if self.ep is None or abs(e - self.ep) < float(c['jump_e']):
                dt = 0.1 if self.tp is None or now is None else max(0.02, min(0.3, float(now) - self.tp))
                w = dt / (float(c['tau_s']) + dt); self.elp = e if self.elp is None else self.elp + w * (e - self.elp)
            self.ep = e
        if now is not None: self.tp = float(now)
        trim = 0. if self.elp is None else max(-float(c['trim_us']), min(float(c['trim_us']), float(c['k_cam']) * self.elp))
        g = float(gz) if _num(gz) else 0.
        cmd = float(c['straight_us']) + float(c['k_psi']) * (float(yaw) - self.psi) + float(c['k_gz']) * g + trim
        cmd = max(float(c['straight_us']) - float(c['lim_us']), min(float(c['straight_us']) + float(c['lim_us']), cmd))
        a = min(1., (x - x0) / max(1e-3, float(c['in_m'])))
        if x > x1: a = min(a, max(0., 1. - (x - x1) / max(1e-3, float(c['out_m']))))
        out = st + a * (cmd - st)
        md = float(c['max_step_us'])
        if self.last is not None: out = self.last + max(-md, min(md, out - self.last))
        out = max(float(lo), min(float(hi), out)); self.last = out
        return out, dict(revision=self.REV, state='HOLD' if a >= 1. else 'BLEND', x_m=round(x, 3), psi_ref=round(self.psi, 2), dpsi=round(float(yaw) - self.psi, 2),
                         gz=round(g, 1), e_lp=None if self.elp is None else round(self.elp, 3), trim_us=round(trim, 1), cmd_us=round(cmd, 1), a=round(a, 3),
                         pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=out != st)

    def _release(self, st, lo, hi):
        if self.last is None: return st, None
        md = float(self.c['max_step_us'])
        if abs(st - self.last) <= md: self.last = None; return st, None
        out = self.last + max(-md, min(md, st - self.last)); out = max(float(lo), min(float(hi), out)); self.last = out
        return out, dict(revision=self.REV, state='TAIL', pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=True)
