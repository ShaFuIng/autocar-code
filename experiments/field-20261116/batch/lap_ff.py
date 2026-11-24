"""lap-ff-r1: position-armed, camera-triggered steering floor into the right curves 1 / 2 / 3 (offline candidate lap-ff-20261115, NOT deployed).

Why (lap65-20261115, lap-ff-20261115 measure.txt): the camera PID turns in from a look-ahead of about 0.85 m (rear axle -> sample row)
with about 0.36 s scene -> wheel delay; the servo command rises only from about the curve's geometric start, reaches 1800-2048 at the
landmark (heading 10 deg past the straight) and then drops to about 1560-1570 (curve 1 / 2, 1.0-1.2 m after the landmark) although the
curve needs about 1700-1750 there: a pulse pattern, entry error 0.74 / 0.87 (median max, c1 / c2), servo 2048 in curve 2 every pass.
The steady curve command is supplied by the camera itself (a centred car in a curve sees e ~ +0.5: look-ahead geometry), so a plain
additive feed-forward would make the car cut in. This module therefore adds a FLOOR, not an offset: from the trigger the command is
not allowed below the steady curve command for the curve's driven curvature at the current speed (times frac < 1); where the camera
asks for more (entry peak, outside drift) the camera wins unchanged.

Arming / trigger (lap_pos, camera-free position):  armed while lap_pos is not lost, its last anchor is the curve's previous
landmark (C4e / C1e / C2e) and the distance since that anchor is within [gap + win[0], gap + win[1]] (gap = nominal landmark distance
of the curve start, lap_pos.json nominal_m). Triggered by the first valid frame in the window whose far-preview error is >= far_on
(toward the right curve) and whose near error is >= near_min; for curves with pos_m set, also when the distance reaches gap + pos_m
without a camera trigger (curve 2 only in the candidate: lap_pos C1e->C2s sd 0.19-0.21 m). One trigger per anchor instance.
Profile over distance from the trigger (odometer of lap_pos): 0 for delay_m, linear ramp over ramp_m to 1, held until the gyro turn
since the trigger reaches end_deg (or max_m), then a linear fade over fade_m. floor = centre + a * (target - centre),
target = clip(c_eff + frac * kappa_need / g(v), centre, cap_us); g(v) = measured deg/m per us of steer_us (gain table).
In the curve (gyro turn since the trigger >= gate_after_deg) the floor is scaled by the look-ahead error: 1 at near >= gate_e_hi, 0 at
near <= gate_e_lo (a centred car in these curves sees near ~ +0.5; a falling near error means the car is moving inside, and the floor
must not hold it there; at the entry the near error is still small, so the gate is not applied before gate_after_deg).
Cancelled (faded over cancel_fade_m) when: the camera contradicts (valid near error <= e_cancel: the lane is LEFT of the look-ahead
point, which never happens in a right curve the car is following), the gyro shows no turn (turned < min_turn_deg at check_m after the trigger; check_m per curve = beyond the latest camera-only
landmark seen in the logs: trigger -> C*s at most 1.04 m (c1 / c2) and 2.28 m (c3), so a camera-only turn always passes and only a
trigger with no curve behind it is cancelled), the pitch says climb (curve 3 only: abs pitch > pitch_max, it ends on the ramp), lap_pos lost (immediately, no fade).
Inputs are values the frame already has (lap_pos state of the previous update, far / near error, wheel speed, pitch): no clock, no IMU,
no file reads except load_config(). Standard library only.
"""
import json, math
from pathlib import Path

REVISION = 'lap-ff-r1'   # lap-ff-r2 additions (max_drop_us, c4: curve4-ff-20261116) keep this string; their keys absent = r1
CONFIG_FILE = Path(__file__).with_name('lap_ff.json')

# per curve: anchor = previous lap_pos landmark, gap_m = nominal distance anchor -> curve start landmark (lap_pos.json nominal_m:
# C1s 67.72 - C4e 56.98 = 10.74; C2s 15.27 - C1e 3.33 = 11.94; C3s 30.21 - C2e 19.77 = 10.44); win_m = arming window relative to the
# predicted start; kappa_need = driven steady curvature in encoder units (deg/m); end_deg = gyro turn after which the floor fades.
DEFAULTS_CURVES = {
    'c1': dict(enabled=True, anchor='C4e', gap_m=10.74, win_m=[-1.8, 1.2], pos_m=None, kappa_need=40.8, end_deg=110., max_m=3.2, cap_us=1800., check_m=1.3, pitch_max=None),
    'c2': dict(enabled=True, anchor='C1e', gap_m=11.94, win_m=[-1.2, 0.6], pos_m=-0.35, kappa_need=43.0, end_deg=160., max_m=4.2, cap_us=1800., check_m=1.3, pitch_max=None),
    'c3': dict(enabled=True, anchor='C2e', gap_m=10.44, win_m=[-2.0, 0.8], pos_m=None, kappa_need=34.0, end_deg=200., max_m=6.0, cap_us=1750., check_m=2.5, pitch_max=3.0),
}
DEFAULTS = dict(enabled=False, far_on=0.4, near_min=-0.05, delay_m=0.0, ramp_m=0.25, frac=1.0, c_eff_us=1490.,
                gain_table=[[0.80, 0.192], [0.93, 0.167], [1.06, 0.159], [1.22, 0.150], [1.37, 0.130]], v_default=0.95,
                fade_m=0.3, e_cancel=-0.1, min_turn_deg=10., cancel_fade_m=0.1,
                gate_after_deg=45., gate_e_lo=0.05, gate_e_hi=0.35)


def load_config(path=None):
    """lap_ff.json merged over DEFAULTS; None when the file is absent / unreadable (= layer off)."""
    p = Path(path) if path else CONFIG_FILE
    try: raw = json.loads(p.read_text(encoding='utf8'))
    except (OSError, ValueError): return None
    if not isinstance(raw, dict): return None
    c = dict(DEFAULTS); c.update({k: v for k, v in raw.items() if k != 'curves'})
    cur = {}
    for k, d in DEFAULTS_CURVES.items():
        x = dict(d); x.update((raw.get('curves') or {}).get(k) or {}); cur[k] = x
    c['curves'] = cur
    return c


def gain(table, v):
    """deg/m per us at wheel speed v (m/s), linear interpolation, clamped to the table ends."""
    t = sorted((float(a), float(b)) for a, b in table)
    if v <= t[0][0]: return t[0][1]
    if v >= t[-1][0]: return t[-1][1]
    for (v0, g0), (v1, g1) in zip(t, t[1:]):
        if v0 <= v <= v1: return g0 + (g1 - g0) * (v - v0) / (v1 - v0)
    return t[-1][1]


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


class Trigger:
    """Arming window + trigger rule (shared by LapFF and the offline evaluation). lp = dict(lap, anchor, since_anchor_m, lost, anchor_d)."""

    def __init__(self, curves, far_on=0.4, near_min=-0.05, mode='far+pos'):
        self.curves = curves; self.far_on = far_on; self.near_min = near_min; self.mode = mode; self.done = set()

    def armed(self, lp):
        if not lp or lp.get('lost') or not _num(lp.get('since_anchor_m')): return None
        for name, c in self.curves.items():
            if not c.get('enabled') or lp.get('anchor') != c['anchor']: continue
            pid = (name, lp.get('anchor'), None if lp.get('anchor_d') is None else round(float(lp['anchor_d']), 3), lp.get('lap'))
            if pid in self.done: return None
            x = float(lp['since_anchor_m']) - float(c['gap_m'])
            if x > float(c['win_m'][1]): self.done.add(pid); return None   # passed the window without a trigger (miss)
            if x >= float(c['win_m'][0]): return name, c, x, pid
        return None

    def step(self, lp, far=None, near=None, valid=True):
        a = self.armed(lp)
        if a is None: return None
        name, c, x, pid = a; kind = None
        if self.mode != 'pos' and valid and _num(far) and float(far) >= float(self.far_on) and _num(near) and float(near) >= float(self.near_min): kind = 'FAR'
        pm = c.get('pos_m') if self.mode != 'far' else None
        if self.mode == 'pos' and pm is None: pm = -0.35
        if kind is None and pm is not None and x >= float(pm): kind = 'POS'
        if kind is None: return None
        self.done.add(pid)
        return dict(curve=name, kind=kind, x_pred_m=round(x, 3))


class LapFF:
    def __init__(self, cfg):
        self.c = cfg; self.trig = Trigger(cfg['curves'], cfg['far_on'], cfg['near_min']); self.act = None; self.n = 0

    def _target(self, cc, v):
        c = self.c; vv = float(v) if _num(v) and float(v) > 0.2 else float(c['v_default'])
        g = gain(c['gain_table'], vv)
        return vv, g, float(c['c_eff_us']) + float(c['frac']) * float(cc['kappa_need']) / g

    def step(self, lp, far, near, valid, v, pitch, yaw, steer, centre, lo, hi, eligible):
        """lp: lap_pos state of the last update (dict: lap, anchor, anchor_d, since_anchor_m, lost, odo_m); yaw: lap_pos gyro heading (deg,
        left +); returns (steer, info or None). info only while a curve is armed-triggered / active."""
        c = self.c; st = float(steer)
        md = c.get('max_drop_us')   # lap-ff-r2: None / absent = r1 behaviour
        if self.act is None:
            tr = self.trig.step(lp, far, near, valid)
            if tr is None: return (st, None) if md is None else self._tail(st, eligible, lo, hi, md)
            if not _num(lp.get('odo_m')) or not _num(yaw): return st, dict(revision=REVISION, state='NO_INPUT', **tr)
            self.act = dict(tr, d0=float(lp['odo_m']), yaw0=float(yaw), end_d=None, cancel=None, cancel_d=None, lap=lp.get('lap'))
        A = self.act; cc = c['curves'][A['curve']]
        if lp is None or lp.get('lost') or not _num(lp.get('odo_m')) or not _num(yaw):
            info = dict(revision=REVISION, curve=A['curve'], kind=A['kind'], state='END', reason='LAP_POS_LOST' if (lp or {}).get('lost') else 'NO_INPUT', applied=False)
            self.act = None
            if md is not None:   # lap-ff-r2
                o2, _ = self._tail(st, eligible, lo, hi, md); info['applied'] = bool(o2 != st); info['steer_us'] = round(o2, 1); return o2, info
            return st, info
        D = float(lp['odo_m']); x = D - A['d0']; turned = A['yaw0'] - float(yaw)
        xi = x - float(c['delay_m']); a_in = 0. if xi <= 0. else min(1., xi / max(1e-3, float(c['ramp_m'])))
        # end / cancel decisions (once)
        if A['cancel'] is None:
            if valid and _num(near) and float(near) <= float(c['e_cancel']): A['cancel'] = 'CAMERA_LEFT'
            elif x >= float(cc['check_m']) and turned < float(c['min_turn_deg']): A['cancel'] = 'NOT_TURNING'
            elif cc.get('pitch_max') is not None and _num(pitch) and abs(float(pitch)) > float(cc['pitch_max']): A['cancel'] = 'PITCH'
            if A['cancel'] is not None: A['cancel_d'] = D
        if A['end_d'] is None and (turned >= float(cc['end_deg']) or x >= float(cc['max_m'])): A['end_d'] = D; A['end_why'] = 'ANGLE' if turned >= float(cc['end_deg']) else 'MAX_M'
        a_out = 1.
        if A['end_d'] is not None: a_out = min(a_out, max(0., 1. - (D - A['end_d']) / max(1e-3, float(c['fade_m']))))
        if A['cancel_d'] is not None: a_out = min(a_out, max(0., 1. - (D - A['cancel_d']) / max(1e-3, float(c['cancel_fade_m']))))
        a_cam = 1.
        if turned >= float(c['gate_after_deg']) and valid and _num(near):   # in the curve: the floor yields as the look-ahead error falls (car moving inside)
            a_cam = max(0., min(1., (float(near) - float(c['gate_e_lo'])) / max(1e-3, float(c['gate_e_hi']) - float(c['gate_e_lo']))))
        a = a_in * a_out * a_cam
        vv, g, tgt = self._target(cc, v)
        tgt = max(float(centre), min(float(cc['cap_us']), float(hi), tgt))
        floor = float(centre) + a * (tgt - float(centre))
        out = max(st, floor) if eligible else st
        if md is not None and eligible and getattr(self, 'last', None) is not None and out < self.last - float(md): out = self.last - float(md)   # lap-ff-r2
        out = max(float(lo), min(float(hi), out))
        if md is not None: self.last = out if (eligible and out > st) else None
        state = 'RAMP' if a_in < 1. and A['end_d'] is None and A['cancel'] is None else ('FADE' if a_out < 1. else 'HOLD')
        info = dict(revision=REVISION, curve=A['curve'], kind=A['kind'], state=state, x_m=round(x, 3), turned_deg=round(turned, 1), a=round(a, 3), a_cam=round(a_cam, 3),
                    v=round(vv, 3), g=round(g, 4), target_us=round(tgt, 1), floor_us=round(floor, 1), pid_steer_us=round(st, 1), steer_us=round(out, 1),
                    applied=bool(out != st), eligible=bool(eligible))
        if A['cancel'] is not None: info['cancel'] = A['cancel']
        if A['end_d'] is not None: info['end'] = A.get('end_why')
        if a_out <= 0. and (A['end_d'] is not None or A['cancel_d'] is not None):
            info['state'] = 'END'; self.act = None
        self.n += 1
        return out, info

    def _tail(self, st, eligible, lo, hi, md):   # lap-ff-r2: release of the floor after the curve ended, at most md us per frame
        last = getattr(self, 'last', None)
        if last is None or not eligible or last - float(md) <= st: self.last = None; return st, None
        out = max(float(lo), min(float(hi), last - float(md))); self.last = out
        return out, dict(revision=REVISION, state='TAIL', pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=True)


# ---- lap-ff-r2 additions (curve4-ff-20261116, offline candidate, NOT deployed). Both are keyed; absent = lap-ff-r1 unchanged.
# (1) max_drop_us (top level of lap_ff.json): the floor is released at most this many us per frame (cancel / end / lap_pos lost /
#     fade), continuing on frames after the curve ended (state TAIL) while the frame stays eligible and the released value is above the PID.
#     Run 8aba 11-16: a CAMERA_LEFT cancel dropped the command 340 us in one frame and threw the car left.
# (2) "c4" (top level of lap_ff.json): curve 4 from the slope landing. lap_pos keeps anchor LAND8 from the landing until C4e (C4s is never an
#     anchor in the logs); LAND8 is at -0.11 m (sd 0.1) from the bottom_guide landing on the current-geometry passes. DOWN (2.32 m before
#     LAND8, lap_pos nominal) is the fallback anchor. Position-triggered (no camera trigger: the camera is unreliable in the first 0.5 s
#     after the landing). Floor target = clip(c_eff + frac * kappa(x) / g(v), centre, cap_us), x = metres
#     since the (predicted) LAND8 point, kappa(x) from kappa_table (deg/m, encoder units). a = ramp * end/cancel fade * heading guard.
#     Unlike r1 (floor = centre + a*(target - centre), which clamps the PID at the centre while a = 0), c4 blends: out = PID + a*max(0, target - PID).
#     turned = heading at the landing minus the heading now, from the 50 Hz IMU yaw when live_pid supplies it (lp['yaw_imu'] now and
#     lp['yaw_land_imu'] = bottom_guide / bottom_map landing yaw _bm_yaw0); otherwise from lap_pos (yaw history at the LAND8 point, lp['yaw_at']).
#     The source is fixed at the trigger; lap_pos yaw integrates per-frame gz only on valid frames and was ~24 deg short after a lost
#     stretch at the landing (a163 last pass). A frame without the chosen source applies no floor.
#     Trigger only on a fresh lap_pos state (now - lp['t'] <= max_age_s): after lost / warm-up frames the state of the last update is stale.
#     Heading guard: turned minus the median-pass turn at the same x (prof_turn, 56 clean passes); ahead_lo..ahead_hi deg ahead of the
#     profile scales the floor 1 -> 0 (the 11-13 over-turn failures were 11-22 deg ahead at 0.5-1 m). Cancel (faded over cancel_fade_m and
#     rate limited by max_drop_us): near error <= e_cancel once x >= cancel_after_m; turned < min_turn_deg at check_m; lap_pos lost (immediate end).
C4_DEFAULTS = dict(enabled=False, anchors=[['LAND8', 0.0], ['DOWN', 2.32]], win_m=[-0.3, 1.0],
                   kappa_table=[[0.0, 30.0], [1.0, 30.0], [1.6, 21.0], [99.0, 21.0]], frac=1.0, cap_us=1720., delay_m=0.0, ramp_m=0.3,
                   end_deg=168., max_m=9.0, fade_m=0.6, e_cancel=-0.45, cancel_after_m=0.6, cancel_fade_m=0.5, check_m=2.0, min_turn_deg=20.,
                   prof_turn=[[0.1, 0.0], [0.6, 3.0], [0.85, 14.0], [1.1, 26.0], [1.35, 36.0], [1.6, 43.0], [2.1, 46.0], [2.6, 57.0], [3.1, 69.0], [4.1, 94.0],
                              [5.1, 115.0], [6.1, 142.0], [7.1, 165.0], [7.6, 175.0], [8.1, 188.0]],
                   ahead_lo=12., ahead_hi=25., max_drop_us=None, rearm_m=20., max_age_s=0.25)


def interp(table, x):
    t = sorted((float(a), float(b)) for a, b in table)
    if x <= t[0][0]: return t[0][1]
    if x >= t[-1][0]: return t[-1][1]
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x0 <= x <= x1: return y0 + (y1 - y0) * (x - x0) / max(1e-9, x1 - x0)
    return t[-1][1]


def load_c4(cfg):
    """cfg = load_config() result (or None); -> merged c4 dict when cfg is enabled and cfg['c4'] is a dict with enabled true, else None."""
    if not isinstance(cfg, dict) or not cfg.get('enabled'): return None
    raw = cfg.get('c4')
    if not isinstance(raw, dict) or not raw.get('enabled'): return None
    c = dict(C4_DEFAULTS); c.update(raw)
    c['_c_eff_us'] = float(raw.get('c_eff_us', cfg['c_eff_us'])); c['_gain_table'] = raw.get('gain_table', cfg['gain_table']); c['_v_default'] = float(raw.get('v_default', cfg['v_default']))
    return c


class LapFF4:
    REV = 'lap-ff-c4-r1'

    def __init__(self, c4):
        self.c = c4; self.act = None; self.last = None; self.last_d0 = None; self.n = 0

    @staticmethod
    def _yaw_land(lp, d_land, D, yaw):   # lap_pos heading at the (predicted) LAND8 point; the current heading when the history cannot give it
        f = lp.get('yaw_at')
        try:
            y = f(min(float(D), float(d_land))) if callable(f) else None
            return float(y) if _num(y) else float(yaw)
        except Exception:
            return float(yaw)

    def _tail(self, st, eligible, lo, hi):
        md = self.c.get('max_drop_us')
        if md is None or self.last is None: self.last = None; return st, None
        if not eligible or self.last - float(md) <= st: self.last = None; return st, None
        out = max(float(lo), min(float(hi), self.last - float(md))); self.last = out
        return out, dict(revision=self.REV, curve='c4', state='TAIL', pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=True)

    def step(self, lp, near, valid, v, yaw, steer, centre, lo, hi, eligible, now=None):
        """lp: lap_pos state of its last update (lap, anchor, anchor_d, since_anchor_m, lost, odo_m); yaw: lap_pos gyro heading (deg, left +)."""
        c = self.c; st = float(steer)
        if self.act is None:
            tr = None
            fresh_t = now is None or not _num(lp.get('t') if lp else None) or float(now) - float(lp['t']) <= float(c['max_age_s'])
            if eligible and fresh_t and lp and not lp.get('lost') and _num(lp.get('since_anchor_m')) and _num(lp.get('odo_m')) and _num(yaw):
                gaps = {str(a): float(g) for a, g in c['anchors']}
                if lp.get('anchor') in gaps:
                    x = float(lp['since_anchor_m']) - gaps[lp['anchor']]
                    D = float(lp['odo_m'])
                    fresh = self.last_d0 is None or D - self.last_d0 > float(c['rearm_m'])
                    if fresh and float(c['win_m'][0]) <= x <= float(c['win_m'][1]):
                        if _num(lp.get('yaw_imu')) and _num(lp.get('yaw_land_imu')): tr = dict(anchor=lp['anchor'], x_trig=round(x, 3), d_land=D - x, yaw0=float(lp['yaw_land_imu']), yaw_ok=True, ysrc='imu')
                        else: tr = dict(anchor=lp['anchor'], x_trig=round(x, 3), d_land=D - x, yaw0=self._yaw_land(lp, D - x, D, yaw), yaw_ok=x >= 0., ysrc='lap_pos')
            if tr is None: return self._tail(st, eligible, lo, hi)
            self.act = dict(tr, end_d=None, cancel=None, cancel_d=None); self.last_d0 = float(lp['odo_m'])
        A = self.act
        if lp is None or lp.get('lost') or not _num(lp.get('odo_m')) or not _num(yaw):
            self.act = None
            out, _ = self._tail(st, eligible, lo, hi)
            return out, dict(revision=self.REV, curve='c4', state='END', reason='LAP_POS_LOST' if (lp or {}).get('lost') else 'NO_INPUT', applied=bool(out != st), steer_us=round(out, 1))
        D = float(lp['odo_m']); x = D - A['d_land']; xt = A['x_trig']
        if not A['yaw_ok'] and D >= A['d_land']: A['yaw0'] = self._yaw_land(lp, A['d_land'], D, yaw); A['yaw_ok'] = True   # trigger before the LAND8 point (DOWN anchor)
        ycur = lp.get('yaw_imu') if A['ysrc'] == 'imu' else yaw
        if not _num(ycur):   # chosen heading source missing on this frame: no floor, state kept
            return self._tail(st, eligible, lo, hi)[0], dict(revision=self.REV, curve='c4', state='NO_YAW', applied=False)
        turned = A['yaw0'] - float(ycur)
        ahead = turned - interp(c['prof_turn'], x)
        xi = x - xt - float(c['delay_m']); a_in = 0. if xi <= 0. else min(1., xi / max(1e-3, float(c['ramp_m'])))
        if A['cancel'] is None:
            if x >= float(c['cancel_after_m']) and valid and _num(near) and float(near) <= float(c['e_cancel']): A['cancel'] = 'CAMERA_LEFT'
            elif x >= float(c['check_m']) and turned < float(c['min_turn_deg']): A['cancel'] = 'NOT_TURNING'
            if A['cancel'] is not None: A['cancel_d'] = D
        if A['end_d'] is None and (turned >= float(c['end_deg']) or x >= float(c['max_m'])): A['end_d'] = D; A['end_why'] = 'ANGLE' if turned >= float(c['end_deg']) else 'MAX_M'
        a_out = 1.
        if A['end_d'] is not None: a_out = min(a_out, max(0., 1. - (D - A['end_d']) / max(1e-3, float(c['fade_m']))))
        if A['cancel_d'] is not None: a_out = min(a_out, max(0., 1. - (D - A['cancel_d']) / max(1e-3, float(c['cancel_fade_m']))))
        lo_h, hi_h = float(c['ahead_lo']), float(c['ahead_hi'])
        a_head = max(0., min(1., 1. - (ahead - lo_h) / max(1e-3, hi_h - lo_h)))
        a = a_in * a_out * a_head
        vv = float(v) if _num(v) and float(v) > 0.2 else float(c['_v_default'])
        g = gain(c['_gain_table'], vv); kap = interp(c['kappa_table'], max(0., x))
        tgt = max(float(centre), min(float(c['cap_us']), float(hi), float(c['_c_eff_us']) + float(c['frac']) * kap / g))
        floor = st + a * max(0., tgt - st)   # blend toward the target only where the PID is below it: a = 0 leaves the PID untouched (no clamp at the centre)
        out = floor if eligible else st
        md = c.get('max_drop_us')
        if md is not None and eligible and self.last is not None and out < self.last - float(md): out = self.last - float(md)
        out = max(float(lo), min(float(hi), out))
        self.last = out if (eligible and out > st) else None
        state = 'RAMP' if a_in < 1. and A['end_d'] is None and A['cancel'] is None else ('FADE' if a_out < 1. else 'HOLD')
        info = dict(revision=self.REV, curve='c4', anchor=A['anchor'], ysrc=A['ysrc'], state=state, x_m=round(x, 3), x_trig=xt, turned_deg=round(turned, 1), ahead_deg=round(ahead, 1),
                    a=round(a, 3), a_head=round(a_head, 3), v=round(vv, 3), g=round(g, 4), kappa=round(kap, 1), target_us=round(tgt, 1), floor_us=round(floor, 1),
                    pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=bool(out != st), eligible=bool(eligible))
        if A['cancel'] is not None: info['cancel'] = A['cancel']
        if A['end_d'] is not None: info['end'] = A.get('end_why')
        if a_out <= 0. and (A['end_d'] is not None or A['cancel_d'] is not None):
            info['state'] = 'END'; self.act = None
        self.n += 1
        return out, info
