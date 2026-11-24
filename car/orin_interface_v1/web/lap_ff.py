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
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'lap-ff-r1'   # c1-band-r1 (c1-band-20261117: band.out_cm / e_per_cm / radius_m, curves.<cN>.delay_m / ramp_m, c4.band.out_cm) keeps this string; lap-ff-r2 additions (max_drop_us, c4: curve4-ff-20261116) keep this string; their keys absent = r1
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
            if lp.get('anchor') == 'START' and isinstance(c.get('start'), dict): c = dict(c, anchor='START', gap_m=c['start'].get('gap_m', c['gap_m']), win_m=c['start'].get('win_m', c['win_m']), pos_m=c['start'].get('pos_m'))   # c1exit-r1: lap from the start
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
        self._band_init()   # c2-band-r1: band_on False when no curve has a 'band' dict (r1 code path, no extra input)

    def _target(self, cc, v):
        c = self.c; vv = float(v) if _num(v) and float(v) > 0.2 else float(c['v_default'])
        g = gain(c['gain_table'], vv)
        return vv, g, float(c['c_eff_us']) + float(c['frac']) * float(cc['kappa_need']) / g

    def step(self, lp, far, near, valid, v, pitch, yaw, steer, centre, lo, hi, eligible):
        """lp: lap_pos state of the last update (dict: lap, anchor, anchor_d, since_anchor_m, lost, odo_m); yaw: lap_pos gyro heading (deg,
        left +); returns (steer, info or None). info only while a curve is armed-triggered / active."""
        c = self.c; st = float(steer)
        if getattr(self, 'a0', False): lp = self._lp0(lp)   # c1exit-r1 (no curve with a 'start' dict -> not called)
        bd_on = getattr(self, 'band_on', False)   # c2-band-r1
        if bd_on: self._band_acc(lp); self.lost_t0 = None
        md = c.get('max_drop_us')   # lap-ff-r2: None / absent = r1 behaviour
        md = md if md is None else _RI.step(md)   # rate-indep-r1
        if self.act is None:
            tr = self.trig.step(lp, self._band_far(lp, far) if bd_on else far, near, valid)   # c2-band-r1: trigger guard
            if tr is None: return (st, None) if md is None else self._tail(st, eligible, lo, hi, md)
            if not _num(lp.get('odo_m')) or not _num(yaw): return st, dict(revision=REVISION, state='NO_INPUT', **tr)
            self.act = dict(tr, d0=float(lp['odo_m']), yaw0=float(yaw), end_d=None, cancel=None, cancel_d=None, lap=lp.get('lap'))
            if bd_on: self._band_arm(lp)
        A = self.act; cc = c['curves'][A['curve']]
        if lp is None or lp.get('lost') or not _num(lp.get('odo_m')) or not _num(yaw):
            info = dict(revision=REVISION, curve=A['curve'], kind=A['kind'], state='END', reason='LAP_POS_LOST' if (lp or {}).get('lost') else 'NO_INPUT', applied=False)
            self.act = None
            if md is not None:   # lap-ff-r2
                o2, _ = self._tail(st, eligible, lo, hi, md); info['applied'] = bool(o2 != st); info['steer_us'] = round(o2, 1); return o2, info
            return st, info
        D = float(lp['odo_m']); x = D - A['d0']; turned = A['yaw0'] - float(yaw)
        dl_, rm_ = self._dr(cc)   # c1-band-r1: per-curve delay_m / ramp_m (absent = global)
        xi = x - dl_; a_in = 0. if xi <= 0. else min(1., xi / max(1e-3, rm_))
        # end / cancel decisions (once)
        if A['cancel'] is None:
            if not c.get('no_cam_cancel') and valid and _num(near) and float(near) <= float(c['e_cancel']): A['cancel'] = 'CAMERA_LEFT'   # arc2-r1: no_cam_cancel = yield only
            elif x >= float(cc['check_m']) and turned < float(c['min_turn_deg']): A['cancel'] = 'NOT_TURNING'
            elif cc.get('pitch_max') is not None and _num(pitch) and abs(float(pitch)) > float(cc['pitch_max']): A['cancel'] = 'PITCH'
            if A['cancel'] is not None: A['cancel_d'] = D
        tb = self._band_turned(A, lp) if bd_on else None   # c2-band-r1: IMU turn since the straight (None = r1 end rule)
        ea = (turned >= float(cc['end_deg'])) if tb is None else (tb >= float(self.band[A['curve']]['end_deg']))
        if A['end_d'] is None and (ea or x >= float(cc['max_m'])): A['end_d'] = D; A['end_why'] = 'ANGLE' if ea else 'MAX_M'
        a_out = 1.
        if A['end_d'] is not None: a_out = min(a_out, max(0., 1. - (D - A['end_d']) / max(1e-3, float(c['fade_m']))))
        if A['cancel_d'] is not None: a_out = min(a_out, max(0., 1. - (D - A['cancel_d']) / max(1e-3, float(c['cancel_fade_m']))))
        a_cam = 1.
        if turned >= float(c['gate_after_deg']) and valid and _num(near):   # in the curve: the floor yields as the look-ahead error falls (car moving inside)
            a_cam = max(0., min(1., (float(near) - float(c['gate_e_lo'])) / max(1e-3, float(c['gate_e_hi']) - float(c['gate_e_lo']))))
        if c.get('gate_rate') is not None:   # arc2-r1: the in-curve gate moves at most gate_rate per frame (both directions)
            gp = A.get('a_cam_prev', 1.); a_cam = gp + max(-_RI.step(float(c['gate_rate'])), min(_RI.step(float(c['gate_rate'])), a_cam - gp)); A['a_cam_prev'] = a_cam
        a = a_in * a_out * a_cam
        vv, g, tgt = self._target(cc, v)
        bi = self._band_tgt(A, lp, near, valid, vv, g) if bd_on else None   # c2-band-r1 (None = r1 target)
        if bi is None: tgt = max(float(centre), min(float(cc['cap_us']), float(hi), tgt))
        else: tgt = max(float(centre) + bi.get('own_lo', 0.), min(bi['cap_us'], float(hi), bi['tgt']))   # c1exit-r1: own_lo 0. unless band.own_lo_us in own mode
        floor = float(centre) + a * (tgt - float(centre))
        out = max(st, floor) if eligible else st
        if bi is not None and bi.get('own_up') is not None:   # c1-band-r1 own mode: the top follows the target (never left of the centre)
            bi['upper'] = max(float(centre) + bi.get('own_lo', 0.), bi['own_up'] if bi['upper'] is None else min(bi['upper'], bi['own_up']))   # c1exit-r1
        if bi is not None and eligible and bi['upper'] is not None and out > bi['upper']: out = out - a * (out - bi['upper'])   # c2-band-r1: band top
        if md is not None and eligible and getattr(self, 'last', None) is not None and out < self.last - float(md): out = self.last - float(md)   # lap-ff-r2
        out = max(float(lo), min(float(hi), out))
        if md is not None: self.last = out if (eligible and out > st) else None
        state = 'RAMP' if a_in < 1. and A['end_d'] is None and A['cancel'] is None else ('FADE' if a_out < 1. else 'HOLD')
        info = dict(revision=REVISION, curve=A['curve'], kind=A['kind'], state=state, x_m=round(x, 3), turned_deg=round(turned, 1), a=round(a, 3), a_cam=round(a_cam, 3),
                    v=round(vv, 3), g=round(g, 4), target_us=round(tgt, 1), floor_us=round(floor, 1), pid_steer_us=round(st, 1), steer_us=round(out, 1),
                    applied=bool(out != st), eligible=bool(eligible))
        if A['cancel'] is not None: info['cancel'] = A['cancel']
        if A['end_d'] is not None: info['end'] = A.get('end_why')
        if bi is not None: info['band'] = dict(revision='c2-band-r1', turned_deg=round(bi['tb'], 1), kappa=round(bi['kap'], 1), prof_us=round(bi['prof'], 1), e_c=round(bi['ec'], 3),
                                            e_us=round(bi['et'], 1), gz_us=round(bi['gt'], 1), top_us=None if bi['upper'] is None else round(bi['upper'], 1), plat=A.get('plat') is not None, guard_n=A.get('guard_n'))
        if bi is not None and bi.get('own_up') is not None: info['band']['own'] = True   # c1-band-r1
        if bi is not None and self.band[A['curve']].get('out_cm'): info['band']['out_cm'] = float(self.band[A['curve']]['out_cm'])   # c1-band-r1 (key absent = log unchanged)
        if a_out <= 0. and (A['end_d'] is not None or A['cancel_d'] is not None):
            info['state'] = 'END'; self.act = None
        self.n += 1
        return out, info

    # ---- c2-band-r1 (c2-crossing-20261117, offline candidate, NOT deployed). Key lap_ff.json curves.<cN>.band (dict; enabled false or
    # absent = r1 behaviour, no extra input used). Measured (c2-crossing-20261117/SUMMARY_FOR_REVIEW.txt): curve 2 is one arc (tape R 1.175 m,
    # ~44.5 deg per metre of the logged rear-wheel odometer); the r1 floor (kappa_need 39.5) is flat in the error: between near ~0 and ~0.6
    # nothing steers more, the car drifts out on the second half, then PID + curve_ff jump to 2048 at near >= 0.75 and the car over-rotates.
    # band: target = c_eff + frac * kappa(tb) / g(v) + clip(k_e * (near - e_c * kappa / kappa_max), -e_lo, +e_hi) + clip(k_gz * (gz + v * kappa), +-gz_lim)
    #   tb = heading turned (IMU, deg, right +) since the straight before the curve (mean IMU yaw over anchor + ref_win, e.g. C1e + 4..9 m);
    #   kappa(tb) from kappa_turn (deg/m, logged-odometer units); floor = centre + a * (target - centre) as r1 (a = ramp * end fade * gate);
    #   top = c_eff + frac * kappa / g + m_hi_us: out = out - a * max(0, out - top) (the camera may add, but not slam to the servo end).
    #   end: tb >= end_deg (instead of the lap_pos-yaw end_deg); trigger guard: no FAR trigger while tb < trig_head_min_deg (car pointing
    #   left of the straight; the position backup pos_m still triggers); lost frames (live_pid hook): target without the camera term for
    #   at most lost_max_s per lost stretch (lost_hold false = off). Inputs added by live_pid only while a band is configured:
    #   lp['yaw_imu'], lp['gz_imu'] (50 Hz IMU heading / yaw rate, deg, deg/s, left +).
    BAND_DEFAULTS = dict(enabled=True, kappa_turn=[[0., 44.5], [178., 44.5], [198., 0.]], ref_win=[4.0, 9.0], ref_min_n=3, turn0_deg=0., e_c=0.5,
                         k_e_us=220., e_lo_us=250., e_hi_us=150., k_gz_us=1.5, gz_lim_us=60., m_hi_us=200., cap_us=1950., end_deg=196.,
                         trig_head_min_deg=-4.0, lost_hold=True, lost_max_s=1.0)

    def _band_init(self):
        self.band = {}
        for k, cc in (self.c.get('curves') or {}).items():
            b = cc.get('band') if isinstance(cc, dict) else None
            if isinstance(b, dict) and b.get('enabled', True) is not False:
                m = dict(self.BAND_DEFAULTS); m.update(b); self.band[k] = m
        self.band_on = bool(self.band); self.pref = {}; self.guard_n = 0; self.lost_t0 = None
        self.a0 = any(isinstance(cc, dict) and cc.get('enabled') and isinstance(cc.get('start'), dict) for cc in (self.c.get('curves') or {}).values())   # c1exit-r1

    def _lp0(self, lp):   # c1exit-r1: lap from the start (lap_pos lap 0, no landmark yet) -> pseudo anchor 'START' at odometer 0
        if not lp or lp.get('anchor') is not None or lp.get('lap') != 0 or lp.get('lost') or not _num(lp.get('odo_m')): return lp
        return dict(lp, anchor='START', anchor_d=0., since_anchor_m=float(lp['odo_m']))

    def _band_acc(self, lp):   # straight-before-the-curve heading: IMU yaw over anchor + ref_win of this anchor instance
        if not lp or lp.get('lost') or not _num(lp.get('since_anchor_m')) or not _num(lp.get('yaw_imu')): return
        for k, b in self.band.items():
            s0 = self.c['curves'][k].get('start') if lp.get('anchor') == 'START' else None   # c1exit-r1
            if lp.get('anchor') != ('START' if isinstance(s0, dict) else self.c['curves'][k]['anchor']): continue
            if isinstance(s0, dict) and s0.get('ref_win'): b = dict(b, ref_win=s0['ref_win'])
            key = (lp.get('lap'), None if lp.get('anchor_d') is None else round(float(lp['anchor_d']), 3))
            R = self.pref.get(k)
            if R is None or R[0] != key: R = self.pref[k] = [key, []]
            if float(b['ref_win'][0]) <= float(lp['since_anchor_m']) <= float(b['ref_win'][1]): R[1].append(float(lp['yaw_imu']))

    def _plat(self, k, lp):
        R = self.pref.get(k)
        if R is None or len(R[1]) < int(self.band[k]['ref_min_n']) or (lp or {}).get('lap') != R[0][0]: return None
        return sum(R[1]) / len(R[1])

    def _band_far(self, lp, far):   # trigger guard: no FAR trigger while the car points left of the straight (8d97 lap 2: -4.9 deg)
        a = self.trig.armed(lp)
        if a is None or a[0] not in self.band or self.band[a[0]].get('trig_head_min_deg') is None: return far
        p = self._plat(a[0], lp); y = (lp or {}).get('yaw_imu')
        if p is None or not _num(y): return far
        if p - float(y) < float(self.band[a[0]]['trig_head_min_deg']): self.guard_n += 1; return None
        return far

    def _band_arm(self, lp):
        A = self.act
        if A is None or A['curve'] not in self.band: return
        A['plat'] = self._plat(A['curve'], lp); A['yaw0i'] = lp.get('yaw_imu') if _num(lp.get('yaw_imu')) else None; A['guard_n'] = self.guard_n; self.guard_n = 0

    def _band_turned(self, A, lp):
        if A is None or A['curve'] not in self.band: return None
        y = (lp or {}).get('yaw_imu')
        if not _num(y): return None
        if A.get('plat') is not None: return A['plat'] - float(y)
        if A.get('yaw0i') is not None: return A['yaw0i'] - float(y) + float(self.band[A['curve']]['turn0_deg'])
        return None

    def _dr(self, cc):   # c1-band-r1: curves.<cN>.delay_m / ramp_m override the global values (absent / None = global, r1)
        d = cc.get('delay_m'); r = cc.get('ramp_m')
        return float(self.c['delay_m'] if d is None else d), float(self.c['ramp_m'] if r is None else r)

    def _band_prof(self, b, tb, v, g):
        kap = interp(b['kappa_turn'], tb); kmax = max(float(q[1]) for q in b['kappa_turn']) or 1.
        if b.get('out_cm') and b.get('radius_m'):   # c1-band-r1: path moved out by out_cm -> radius larger -> curvature x R/(R+d)
            sc = float(b['radius_m']) / (float(b['radius_m']) + float(b['out_cm']) / 100.); kap *= sc; kmax *= sc
        prof = float(self.c['c_eff_us']) + float(self.c['frac']) * kap / g
        return kap, kmax, prof

    def _band_tgt(self, A, lp, near, valid, v, g):
        tb = self._band_turned(A, lp)
        if tb is None: return None
        b = self.band[A['curve']]
        if _num(b.get('v_ref')) and float(b['v_ref']) > 0.2: v = float(b['v_ref']); g = gain(self.c['gain_table'], v)   # c1exit-r1: fixed speed for the profile / gyro term
        kap, kmax, prof = self._band_prof(b, tb, v, g)
        ec = (float(b['e_c']) + float(b.get('out_cm') or 0.) * float(b.get('e_per_cm', 0.025))) * kap / kmax if b.get('out_cm') else float(b['e_c']) * kap / kmax   # c1-band-r1: outward offset (cm) -> error target
        ect = ec_turn(b, tb)   # band-ec-turn-r1
        if ect is not None: ec = ect
        et = max(-float(b['e_lo_us']), min(float(b['e_hi_us']), float(b['k_e_us']) * (float(near) - ec))) if (valid and _num(near)) else 0.
        gz = (lp or {}).get('gz_imu'); gt = 0.
        if _num(gz) and b.get('k_gz_us'): gt = max(-float(b['gz_lim_us']), min(float(b['gz_lim_us']), float(b['k_gz_us']) * (float(gz) + float(v) * kap)))
        up = None if b.get('m_hi_us') is None else prof + float(b['m_hi_us'])
        own = (prof + et + gt + float(b.get('own_us') or 0.)) if (b.get('own_after_deg') is not None and tb >= float(b['own_after_deg'])) else None   # c1-band-r1: own mode (absent = r1)
        r_ = dict(tgt=prof + et + gt, upper=up, cap_us=float(b['cap_us']), tb=tb, kap=kap, prof=prof, et=et, gt=gt, ec=ec, own_up=own)
        if own is not None and _num(b.get('own_lo_us')) and float(b['own_lo_us']) < 0.: r_['own_lo'] = float(b['own_lo_us'])   # c1exit-r1
        return r_

    def lost(self, lp, v, steer, centre, lo, hi, now):
        """c2-band-r1 lost frame (live_pid hook, only while band_on): (steer, info) or None (= the lost policy's own command)."""
        A = self.act
        if A is None or A['curve'] not in self.band or not self.band[A['curve']].get('lost_hold'): return None
        if lp is None or lp.get('lost') or now is None: return None
        b = self.band[A['curve']]
        if _num(b.get('v_ref')) and float(b['v_ref']) > 0.2: v = float(b['v_ref'])   # c1exit-r1
        if self.lost_t0 is None: self.lost_t0 = float(now)
        if float(now) - self.lost_t0 > float(b['lost_max_s']): return None
        tb = self._band_turned(A, lp)
        if tb is None or tb >= float(b['end_deg']) or A.get('cancel') is not None or A.get('end_d') is not None: return None
        c = self.c; vv = float(v) if _num(v) and float(v) > 0.2 else float(c['v_default']); g = gain(c['gain_table'], vv)
        kap, kmax, prof = self._band_prof(b, tb, vv, g)
        gz = lp.get('gz_imu'); gt = 0.
        if _num(gz) and b.get('k_gz_us'): gt = max(-float(b['gz_lim_us']), min(float(b['gz_lim_us']), float(b['k_gz_us']) * (float(gz) + vv * kap)))
        tgt = max(float(centre) + (float(b['own_lo_us']) if (b.get('own_after_deg') is not None and _num(b.get('own_lo_us')) and float(b['own_lo_us']) < 0. and tb >= float(b['own_after_deg'])) else 0.), min(float(b['cap_us']), float(hi), prof + gt))   # c1exit-r1
        a = 1.
        dl_, rm_ = self._dr(c['curves'][A['curve']])   # c1-band-r1
        if _num(lp.get('odo_m')): xi = float(lp['odo_m']) - A['d0'] - dl_; a = 0. if xi <= 0. else min(1., xi / max(1e-3, rm_))
        out = max(float(lo), min(float(hi), float(centre) + a * (tgt - float(centre))))
        if a <= 0.: return None
        self.last = out
        return out, dict(revision='c2-band-r1', curve=A['curve'], state='LOST_HOLD', turned_deg=round(tb, 1), kappa=round(kap, 1), prof_us=round(prof, 1), gz_us=round(gt, 1),
                         a=round(a, 3), lost_s=round(float(now) - self.lost_t0, 3), held_us=round(float(steer), 1), steer_us=round(out, 1))

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


def own_share(O, x, near, valid):
    """band-own-r1: share 0..1 of 'output = band target' at distance x, or None (O absent / not a dict / enabled not true).
    ramp over ramp_m from x0_m; falls to 0 as the near error goes e_max - e_band -> e_max (car far outside); 0 on a frame without a usable error."""
    if not isinstance(O, dict) or O.get('enabled') is not True: return None
    try:
        wx = max(0., min(1., (float(x) - float(O.get('x0_m', 0.))) / max(1e-3, float(O.get('ramp_m', 0.3)))))
        if not (valid and _num(near)): return 0.
        return wx * max(0., min(1., (float(O.get('e_max', 0.9)) - float(near)) / max(1e-3, float(O.get('e_band', 0.1)))))
    except (TypeError, ValueError): return None


def ec_turn(b, tb):
    """band-ec-turn-r1: error centre from b['e_c_turn'] at the turned angle tb, or None (key absent / unusable -> the old formula)."""
    t = b.get('e_c_turn') if isinstance(b, dict) else None
    if not isinstance(t, (list, tuple)) or len(t) < 2 or not _num(tb): return None
    try:
        if not all(len(q) == 2 and _num(q[0]) and _num(q[1]) and -1. <= float(q[1]) <= 1. for q in t): return None
        return float(interp(t, float(tb)))
    except (TypeError, ValueError): return None


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

    def __init__(self, c4, name='c4'):
        self.c = c4; self.act = None; self.last = None; self.last_d0 = None; self.n = 0; self.name = name; self.ay = 1.   # arc2-r1: name / yield state
        self._b4_init()   # c4-band-r1: band_on False when the block has no enabled 'band' dict (r1 code path)
        self._xb_init()   # x-band-r1 (band-own-r1): xb None when the block has no enabled 'xband' dict

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
        md = md if md is None else _RI.step(md)   # rate-indep-r1
        if md is None or self.last is None: self.last = None; return st, None
        if not eligible or self.last - float(md) <= st: self.last = None; return st, None
        out = max(float(lo), min(float(hi), self.last - float(md))); self.last = out
        return out, dict(revision=self.REV, curve=self.name, state='TAIL', pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=True)

    # ---- c3-tail-r1 (c3-tail-20261119, offline candidate, NOT deployed). Key arc2.json c3.top (dict; absent / enabled false = r1).
    # Curve-3 tail 'straight' piece E2 (between the last flat bend B4 and the ramp-foot corner B5, C3s + ~5.2..5.9 m): the camera
    # look-ahead, the PID integral (+40) and the curve-map yaw-rate floor (-20 dps) keep the car turning ~20 deg/s there and it
    # reaches the corner with the right rear wheel 9-13 cm over the inner line. top: final command of this layer <= top(x),
    # x = lap_pos metres since this lap's C3s crossing, top(x) = interp(table) inside [table[0].x, table[-1].x] (never below
    # centre + min_rel_us). Lowered by at most max_down_us per frame from the uncapped value, raised back by at most max_up_us per
    # frame after the window / on release. Applied only on eligible frames with a fresh lap_pos state anchored at C3s (otherwise no
    # top on that frame). Released (latched for this pass) when the camera near error >= e_release (car far out / corner early) or
    # when the car has turned ahead_min_deg less than the median pass at this x (prof_turn of the c3 block; heading at the C3s
    # crossing from the IMU + lap_pos history, as the c3 floor does). Never raises the command.
    TOP_DEFAULTS = dict(anchor='C3s', table=[], e_release=0.6, ahead_min_deg=-12., max_down_us=25., max_up_us=150., min_rel_us=0., max_age_s=0.25)

    def step(self, lp, near, valid, v, yaw, steer, centre, lo, hi, eligible, now=None):   # c3-tail-r1 wrapper ('top' absent = _step0 unchanged)
        out, info = self._step0(lp, near, valid, v, yaw, steer, centre, lo, hi, eligible, now)
        T = self.c.get('top') if isinstance(self.c, dict) else None
        if not isinstance(T, dict) or T.get('enabled', True) is False: return out, info
        return self._top(T, lp, near, valid, yaw, steer, out, info, centre, lo, hi, eligible, now)

    def _top(self, T, lp, near, valid, yaw, steer, out, info, centre, lo, hi, eligible, now):
        c = dict(self.TOP_DEFAULTS); c.update(T); S = getattr(self, '_ts', None); lp = lp or {}
        tb = sorted((float(a), float(b)) for a, b in (c.get('table') or []))
        if not tb: return out, info
        ok = (not lp.get('lost') and lp.get('anchor') == c['anchor'] and _num(lp.get('since_anchor_m')) and _num(lp.get('odo_m')))
        fresh = now is None or not _num(lp.get('t')) or float(now) - float(lp['t']) <= float(c['max_age_s'])
        x = None
        if ok:
            x = float(lp['since_anchor_m']); d0 = float(lp['odo_m']) - x
            if S is None or abs(S['d0'] - d0) > 2.0: S = self._ts = dict(d0=d0, yaw0=None, rel=None, last=None, n=0)
        if S is None: return out, info
        st = float(out); win = x is not None and tb[0][0] <= x <= tb[-1][0]
        turned = ahead = None
        if win and S['yaw0'] is None and _num(lp.get('yaw_imu')) and _num(yaw):   # heading at the C3s crossing (IMU now + lap_pos turn since it)
            S['yaw0'] = float(lp['yaw_imu']) + self._yaw_land(lp, float(lp['odo_m']) - x, float(lp['odo_m']), yaw) - float(yaw)
        if win and S['yaw0'] is not None and _num(lp.get('yaw_imu')):
            turned = S['yaw0'] - float(lp['yaw_imu']); ahead = turned - interp(self.c.get('prof_turn') or [[0., 0.]], x)
        if win and S['rel'] is None and eligible and fresh:
            if valid and _num(near) and float(near) >= float(c['e_release']): S['rel'] = 'CAMERA'
            elif ahead is not None and ahead < float(c['ahead_min_deg']): S['rel'] = 'UNDER_TURNED'
        state = None
        if win and S['rel'] is None and eligible and fresh:
            tgt = max(float(centre) + float(c['min_rel_us']), interp(tb, x))
            base = S['last'] if S['last'] is not None else st
            new = min(st, max(tgt, base - _RI.step(float(c['max_down_us']))))
            S['last'] = new if new < st else None; S['n'] += 1
            state = 'CAP' if new < st else 'BELOW'; res = new
        elif S['last'] is not None and (eligible or not win):   # after the window / released / not fresh: rate-limited return to the chain value
            new = min(st, S['last'] + _RI.step(float(c['max_up_us'])))
            S['last'] = new if new < st else None
            state = 'RETURN'; res = new
        else:
            if not win: return out, info
            if not eligible: S['last'] = None
            state = 'RELEASED' if S['rel'] is not None else ('NOT_ELIGIBLE' if not eligible else 'STALE'); res = st
        res = max(float(lo), min(float(hi), res))
        if info is None: info = dict(revision=self.REV, curve=self.name, state='TOP', pid_steer_us=round(float(steer), 1))
        info['steer_us'] = round(res, 1); info['applied'] = bool(res != float(steer))
        info['top'] = dict(revision='c3-tail-r1', state=state, x_m=None if x is None else round(x, 3), top_us=round(interp(tb, x), 1) if win else None, in_us=round(st, 1),
                           out_us=round(res, 1), turned_deg=None if turned is None else round(turned, 1), ahead_deg=None if ahead is None else round(ahead, 1), rel=S['rel'])
        return res, info

    def _step0(self, lp, near, valid, v, yaw, steer, centre, lo, hi, eligible, now=None):
        """lp: lap_pos state of its last update (lap, anchor, anchor_d, since_anchor_m, lost, odo_m); yaw: lap_pos gyro heading (deg, left +)."""
        c = self.c; st = float(steer)
        if getattr(self, 'xb_dr', False): self._xb_tick(lp, now)   # x-band-r2: when did the odometer last step
        b4 = getattr(self, 'band_on', False)   # c4-band-r1
        if b4: self._b4_acc(lp); self.lost_t0 = None
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
                        if c.get('yaw_ref') == 'anchor':   # arc2-r1 (c3): heading at the anchor crossing = IMU now + lap_pos turn since the crossing
                            yl = self._yaw_land(lp, D - x, D, yaw)
                            tr = dict(anchor=lp['anchor'], x_trig=round(x, 3), d_land=D - x, yaw0=(float(lp['yaw_imu']) + yl - float(yaw)) if _num(lp.get('yaw_imu')) else yl, yaw_ok=True, ysrc='imu' if _num(lp.get('yaw_imu')) else 'lap_pos')
                        elif _num(lp.get('yaw_imu')) and _num(lp.get('yaw_land_imu')): tr = dict(anchor=lp['anchor'], x_trig=round(x, 3), d_land=D - x, yaw0=float(lp['yaw_land_imu']), yaw_ok=True, ysrc='imu')
                        else: tr = dict(anchor=lp['anchor'], x_trig=round(x, 3), d_land=D - x, yaw0=self._yaw_land(lp, D - x, D, yaw), yaw_ok=x >= 0., ysrc='lap_pos')
            if tr is None: return self._tail(st, eligible, lo, hi)
            self.act = dict(tr, end_d=None, cancel=None, cancel_d=None); self.last_d0 = float(lp['odo_m']); self.ay = None   # arc2-r1: first frame takes the yield target
            if b4: self._b4_arm(lp)   # c4-band-r1: slope reference heading of this pass
        A = self.act
        if lp is None or lp.get('lost') or not _num(lp.get('odo_m')) or not _num(yaw):
            self.act = None
            out, _ = self._tail(st, eligible, lo, hi)
            return out, dict(revision=self.REV, curve=self.name, state='END', reason='LAP_POS_LOST' if (lp or {}).get('lost') else 'NO_INPUT', applied=bool(out != st), steer_us=round(out, 1))
        D = float(lp['odo_m']); x = D - A['d_land']; xt = A['x_trig']
        if not A['yaw_ok'] and D >= A['d_land']: A['yaw0'] = self._yaw_land(lp, A['d_land'], D, yaw); A['yaw_ok'] = True   # trigger before the LAND8 point (DOWN anchor)
        ycur = lp.get('yaw_imu') if A['ysrc'] == 'imu' else yaw
        if not _num(ycur):   # chosen heading source missing on this frame: no floor, state kept
            return self._tail(st, eligible, lo, hi)[0], dict(revision=self.REV, curve=self.name, state='NO_YAW', applied=False)
        turned = A['yaw0'] - float(ycur)
        ahead = turned - interp(c['prof_turn'], x)
        xi = x - xt - float(c['delay_m']); a_in = 0. if xi <= 0. else min(1., xi / max(1e-3, float(c['ramp_m'])))
        if A['cancel'] is None:
            if c.get('yield_e') is None and x >= float(c['cancel_after_m']) and valid and _num(near) and float(near) <= float(c['e_cancel']): A['cancel'] = 'CAMERA_LEFT'   # arc2-r1: yield_e set -> yield, no camera cancel
            elif x >= float(c['check_m']) and turned < float(c['min_turn_deg']): A['cancel'] = 'NOT_TURNING'
            elif c.get('pitch_max') is not None and _num(lp.get('pitch')) and abs(float(lp['pitch'])) > float(c['pitch_max']): A['cancel'] = 'PITCH'   # arc2-r1 (c3: ends before the ramp)
            if A['cancel'] is not None: A['cancel_d'] = D
        tb = self._b4_tb(A, lp) if b4 else None   # c4-band-r1: IMU turn since the slope heading (None = r1 end rule)
        ea = (turned >= float(c['end_deg'])) if tb is None else (tb >= float(self.band['end_deg']))
        mx = float(c['max_m']) if tb is None else float(self.band['max_m'])
        if A['end_d'] is None and (ea or x >= mx): A['end_d'] = D; A['end_why'] = 'ANGLE' if ea else 'MAX_M'
        a_out = 1.
        if A['end_d'] is not None: a_out = min(a_out, max(0., 1. - (D - A['end_d']) / max(1e-3, float(c['fade_m']))))
        if A['cancel_d'] is not None: a_out = min(a_out, max(0., 1. - (D - A['cancel_d']) / max(1e-3, float(c['cancel_fade_m']))))
        lo_h, hi_h = float(c['ahead_lo']), float(c['ahead_hi'])
        a_head = max(0., min(1., 1. - (ahead - lo_h) / max(1e-3, hi_h - lo_h)))
        a = a_in * a_out * a_head
        ye = c.get('yield_e')
        if ye is not None:   # arc2-r1 (B): inside-yield 1 at near >= ye[1] .. 0 at near <= ye[0], rate limited both directions; returns when the error comes back
            yt = None if not (valid and _num(near)) else max(0., min(1., (float(near) - float(ye[0])) / max(1e-3, float(ye[1]) - float(ye[0]))))
            yr = _RI.step(float(c.get('yield_rate') or 1.)); self.ay = (yt if yt is not None else 1.) if self.ay is None else self.ay + max(-yr, min(yr, (self.ay if yt is None else yt) - self.ay)); a *= self.ay
        vv = float(v) if _num(v) and float(v) > 0.2 else float(c['_v_default'])
        g = gain(c['_gain_table'], vv); kap = interp(c['kappa_table'], max(0., x))
        tgt = max(float(centre), min(float(c['cap_us']), float(hi), float(c['_c_eff_us']) + float(c['frac']) * kap / g))
        floor = st + a * max(0., tgt - st)   # blend toward the target only where the PID is below it: a = 0 leaves the PID untouched (no clamp at the centre)
        out = floor if eligible else st
        bi = self._b4_band(A, lp, x, tb, near, valid, vv, g, st, float(centre), float(hi), a_out) if tb is not None else None   # c4-band-r1
        if bi is not None and eligible: out = out + bi['w'] * (bi['out'] - out)
        xb = self._xb(A, lp, x, near, valid, vv, g, out, float(centre), float(hi), now, turned) if (eligible and getattr(self, 'xb', None) is not None) else None   # x-band-r1 / r2
        if xb is not None: out = xb['out']
        md = c.get('max_drop_us')
        md = md if md is None else _RI.step(md)   # rate-indep-r1
        if md is not None and eligible and self.last is not None and out < self.last - float(md): out = self.last - float(md)
        out = max(float(lo), min(float(hi), out))
        self.last = out if (eligible and out > st) else None
        state = 'RAMP' if a_in < 1. and A['end_d'] is None and A['cancel'] is None else ('FADE' if a_out < 1. else 'HOLD')
        info = dict(revision=self.REV, curve=self.name, anchor=A['anchor'], ysrc=A['ysrc'], state=state, x_m=round(x, 3), x_trig=xt, turned_deg=round(turned, 1), ahead_deg=round(ahead, 1),
                    a=round(a, 3), a_head=round(a_head, 3), v=round(vv, 3), g=round(g, 4), kappa=round(kap, 1), target_us=round(tgt, 1), floor_us=round(floor, 1),
                    pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=bool(out != st), eligible=bool(eligible))
        if A['cancel'] is not None: info['cancel'] = A['cancel']
        if A['end_d'] is not None: info['end'] = A.get('end_why')
        if ye is not None: info['a_yield'] = round(self.ay, 3)
        if b4: info['band'] = self._b4_info(A, tb, bi)   # c4-band-r1
        if xb is not None: info['xband'] = xb['info']   # x-band-r1
        if a_out <= 0. and (A['end_d'] is not None or A['cancel_d'] is not None):
            info['state'] = 'END'; self.act = None
        self.n += 1
        return out, info

    # ---- c4-band-r1 (c4-band-20261117, offline candidate, NOT deployed). Key lap_ff.json c4.band (dict; absent / enabled false = r1).
    # Curve 4 from x0_m (default 2.3 m after LAND8 = where the arc2 landing band starts its hand-over) to the main straight:
    #   target = c_eff + kappa(tb)/g(v) + clip(k_e*(near - e_c*kappa/kappa_ref), -e_lo, +e_hi) + clip(k_gz*(gz + v*kappa), +-gz_lim)
    #   tb = slope reference heading - IMU heading now (deg, right +); slope reference = mean IMU heading of the valid frames while the
    #   lap_pos anchor is ref_anchor ('DOWN') and its distance is within ref_win (the straight downhill; 59 passes: reference -> main
    #   straight 199.4 deg, sd 1.7, the landing heading _bm_yaw0 had sd 6.4 with 24-deg outliers). kappa(tb) = kappa_turn (deg/m of the
    #   logged odometer; median of the 16 not-bad passes 11-16/11-17, tapered to 0 at the straight heading).
    #   band = min(max(PID chain, target), top), top = max(target, min(cap_us, prof + max(m_hi_min, m_hi*min(1, kappa/kappa_ref))))
    #   (the camera / curve_ff / yaw_inner chain may add at most the margin above the profile: no slam to the servo end).
    #   Blend: out = r1 floor + w*(band - r1 floor), w = ramp over ramp_m from x0_m (no old floor after it; the arc2 band owns x < x0_m).
    #   End: tb >= end_deg (or x >= max_m) -> r1 fade_m (only the top applies while fading) and max_drop_us TAIL. No reference (too few slope frames / IMU) -> r1 unchanged.
    #   Lost frames (live_pid hook, lost_hold): clip(profile + gyro term, centre, cap) for at most lost_max_s per lost stretch, x >= x0_m.
    B4_DEFAULTS = dict(enabled=True, ref_anchor='DOWN', ref_win=[0.1, 1.9], ref_min_n=5, ref_max_m=5.0,
                       kappa_turn=[[0., 5.], [10., 30.], [15., 46.], [30., 45.], [40., 30.], [45., 20.], [55., 22.], [65., 24.], [72., 28.], [80., 26.], [90., 21.],
                                   [112., 21.], [122., 26.], [140., 29.], [150., 25.], [163., 20.], [176., 19.], [184., 16.], [192., 8.], [199., 0.]],
                       x0_m=2.3, ramp_m=0.3, e_c=0.30, kappa_ref=22., k_e_us=200., e_lo_us=250., e_hi_us=150., k_gz_us=1.5, gz_lim_us=60.,
                       m_hi_us=120., m_hi_min_us=40., cap_us=1780., floor_below_us=0., end_deg=199., max_m=10.5, lost_hold=True, lost_max_s=1.0)

    # ---- x-band-r1 (band-own-r1, c3c4-20261122). Key 'xband' of a LapFF4 block (arc2.json c3.xband; dict, enabled true; absent = nothing runs).
    XB_DEFAULTS = dict(enabled=False, kappa_x=None, e_c_x=None, w_x=None, k_e_us=200., e_lo_us=250., e_hi_us=200., k_gz_us=1.5, gz_lim_us=60., cap_rel_us=330.,
                       floor_below_us=0., e_max=0.9, e_band=0.1)

    def _xb_init(self):
        self.xb = None; self.xb_on = False
        X = self.c.get('xband') if isinstance(self.c, dict) else None
        if not isinstance(X, dict) or X.get('enabled') is not True: return
        m = dict(self.XB_DEFAULTS); m.update(X)
        try:
            for k in ('kappa_x', 'e_c_x', 'w_x'):
                t = m[k]
                if not isinstance(t, (list, tuple)) or len(t) < 2 or not all(len(q) == 2 and _num(q[0]) and _num(q[1]) for q in t): return
            if not all(0. <= float(q[1]) <= 1. for q in m['w_x']) or not all(-1. <= float(q[1]) <= 1. for q in m['e_c_x']): return
            for k in ('k_e_us', 'e_lo_us', 'e_hi_us', 'k_gz_us', 'gz_lim_us', 'cap_rel_us', 'floor_below_us', 'e_max', 'e_band'):
                if not _num(m[k]): return
            m['_r2'] = any(k in X for k in ('x_dr', 'u_x', 'h_x', 'y_x', 'rate_us_s'))   # x-band-r2 (c3refit): none of these keys = the r1 code below
            for k in ('u_x', 'h_x', 'y_x'):
                t = m.get(k)
                if t is not None and (not isinstance(t, (list, tuple)) or len(t) < 2 or not all(len(q) == 2 and _num(q[0]) and _num(q[1]) for q in t)): return
            for k, dflt in (('k_h_us', 0.), ('h_lim_us', 80.), ('h_dev_max', 25.), ('k_y_us', 0.), ('y_lim_us', 80.), ('y_hold_s', 0.6), ('y_age_s', 0.2), ('rate_us_s', 0.), ('dr_max_s', 0.3)):
                m.setdefault(k, dflt)
                if not _num(m[k]) or float(m[k]) < 0.: return
        except (TypeError, ValueError): return
        self.xb = m; self.xb_on = True; self.xb_dr = bool(m['_r2'] and m.get('x_dr') is True); self._xd = None; self._xy = None; self._xl = None

    def _xb_tick(self, lp, now):   # x-band-r2: [odometer value, clock when it last stepped (None until a step has been seen)]
        D = (lp or {}).get('odo_m')
        if not _num(D): return
        D = float(D); S = self._xd
        if S is None or D < S[0] - 1e-9 or D - S[0] > 2.0: self._xd = [D, None]
        elif D > S[0] + 1e-9: S[0] = D; S[1] = float(now) if _num(now) else None

    def _xb2(self, A, lp, xs, near, valid, v, g, out, centre, hi, now, turned):
        X = self.xb; lp = lp or {}; x = xs; tn = float(now) if _num(now) else None
        S = self._xd
        if self.xb_dr and S is not None and S[1] is not None and tn is not None and _num(v):
            x = xs + max(0., min(float(X['dr_max_s']), tn - S[1])) * float(v)
        w = interp(X['w_x'], x)
        if w <= 0. or not (valid and _num(near)):
            self._xl = None; return None
        kap = interp(X['kappa_x'], x); ec = interp(X['e_c_x'], x)
        prof = float(self.c['_c_eff_us']) + (interp(X['u_x'], x) if X.get('u_x') is not None else kap / g)
        et = max(-float(X['e_lo_us']), min(float(X['e_hi_us']), float(X['k_e_us']) * (float(near) - ec)))
        gz = lp.get('gz_imu'); gt = 0.
        if _num(gz) and X['k_gz_us']: gt = max(-float(X['gz_lim_us']), min(float(X['gz_lim_us']), float(X['k_gz_us']) * (float(gz) + float(v) * kap)))
        ht = 0.; hd = None; href = None
        if X.get('h_x') is not None and float(X['k_h_us']) > 0. and _num(turned):
            if 'h_off' not in A:   # heading at the START of the encoder step that carries the anchor crossing (turned_deg refers to its end)
                f = lp.get('yaw_at'); A['h_off'] = None
                try:
                    if callable(f): A['h_off'] = float(f(float(A['d_land']) - 1e-6)) - float(f(float(A['d_land'])))
                except Exception: A['h_off'] = None
            if A['h_off'] is not None and abs(A['h_off']) <= 25.:
                hd = float(turned) + A['h_off']; href = interp(X['h_x'], x)
                if abs(href - hd) <= float(X['h_dev_max']): ht = max(-float(X['h_lim_us']), min(float(X['h_lim_us']), float(X['k_h_us']) * (href - hd)))   # further off the plan than h_dev_max: heading not trusted, no term
        yt = 0.; ym = None
        if X.get('y_x') is not None and float(X['k_y_us']) > 0. and tn is not None:
            al = lp.get('al')
            if isinstance(al, (list, tuple)) and len(al) >= 2 and _num(al[0]) and _num(al[1]) and 0. <= tn - float(al[0]) <= float(X['y_age_s']): self._xy = (float(al[1]), tn)
            if self._xy is not None and 0. <= tn - self._xy[1] <= float(X['y_hold_s']):
                ym = self._xy[0]; yt = max(-float(X['y_lim_us']), min(float(X['y_lim_us']), float(X['k_y_us']) * (ym - interp(X['y_x'], x))))
        raw = max(centre - float(X['floor_below_us']), min(centre + float(X['cap_rel_us']), hi, prof + et + gt + ht + yt)); tgt = raw
        L = self._xl
        if float(X['rate_us_s']) > 0. and L is not None and tn is not None and 0. < tn - L[1] <= 0.25:
            r = float(X['rate_us_s']) * (tn - L[1]); tgt = max(L[0] - r, min(L[0] + r, raw))
        self._xl = (tgt, tn) if tn is not None else None
        we = max(0., min(1., (float(X['e_max']) - float(near)) / max(1e-3, float(X['e_band']))))
        o = out if X.get('shadow') else out + w * we * (tgt - out)
        r3 = lambda q: None if q is None else round(q, 1)
        return dict(out=o, info=dict(revision='x-band-r2', x_m=round(x, 3), xs_m=round(xs, 3), w=round(w, 3), own=round(w * we, 3), kappa=round(kap, 1), prof_us=round(prof, 1), e_c=round(ec, 3),
                                     e_us=round(et, 1), gz_us=round(gt, 1), h_deg=r3(hd), h_ref=r3(href), h_us=round(ht, 1), y_cm=r3(ym), y_us=round(yt, 1), raw_us=round(raw, 1),
                                     target_us=round(tgt, 1), in_us=round(out, 1), out_us=round(o, 1)))

    def _xb(self, A, lp, x, near, valid, v, g, out, centre, hi, now=None, turned=None):
        X = self.xb
        if X.get('_r2'):   # x-band-r2 (c3refit)
            if A.get('cancel') is not None or A.get('end_d') is not None: return None
            return self._xb2(A, lp, x, near, valid, v, g, out, centre, hi, now, turned)
        if A.get('cancel') is not None or A.get('end_d') is not None: return None
        w = interp(X['w_x'], x)
        if w <= 0. or not (valid and _num(near)): return None
        kap = interp(X['kappa_x'], x); prof = float(self.c['_c_eff_us']) + kap / g; ec = interp(X['e_c_x'], x)
        et = max(-float(X['e_lo_us']), min(float(X['e_hi_us']), float(X['k_e_us']) * (float(near) - ec)))
        gz = (lp or {}).get('gz_imu'); gt = 0.
        if _num(gz) and X['k_gz_us']: gt = max(-float(X['gz_lim_us']), min(float(X['gz_lim_us']), float(X['k_gz_us']) * (float(gz) + float(v) * kap)))
        tgt = max(centre - float(X['floor_below_us']), min(centre + float(X['cap_rel_us']), hi, prof + et + gt))
        we = max(0., min(1., (float(X['e_max']) - float(near)) / max(1e-3, float(X['e_band']))))
        o = out if X.get('shadow') else out + w * we * (tgt - out)   # shadow true = computed and logged only
        return dict(out=o, info=dict(revision='x-band-r1', x_m=round(x, 3), w=round(w, 3), own=round(w * we, 3), kappa=round(kap, 1), prof_us=round(prof, 1), e_c=round(ec, 3),
                                     e_us=round(et, 1), gz_us=round(gt, 1), target_us=round(tgt, 1), in_us=round(out, 1), out_us=round(o, 1)))

    def _b4_init(self):
        b = self.c.get('band') if isinstance(self.c, dict) else None
        self.band_on = isinstance(b, dict) and b.get('enabled', True) is not False
        self.band = None; self.bref = None; self.lost_t0 = None
        if self.band_on: m = dict(self.B4_DEFAULTS); m.update(b); self.band = m

    def _b4_acc(self, lp):   # slope reference: IMU heading over ref_anchor + ref_win of this anchor instance (valid frames, any eligibility)
        B = self.band
        if not lp or lp.get('lost') or lp.get('anchor') != B['ref_anchor'] or not _num(lp.get('since_anchor_m')) or not _num(lp.get('yaw_imu')) or not _num(lp.get('odo_m')): return
        if not float(B['ref_win'][0]) <= float(lp['since_anchor_m']) <= float(B['ref_win'][1]): return
        key = (lp.get('lap'), None if lp.get('anchor_d') is None else round(float(lp['anchor_d']), 3))
        if self.bref is None or self.bref[0] != key: self.bref = [key, [], None]
        self.bref[1].append(float(lp['yaw_imu'])); self.bref[2] = float(lp['odo_m'])

    def _b4_arm(self, lp):
        A = self.act; R = self.bref; A['bref'] = None
        if R is None or len(R[1]) < int(self.band['ref_min_n']) or not _num(lp.get('odo_m')) or float(lp['odo_m']) - R[2] > float(self.band['ref_max_m']): return
        A['bref'] = sum(R[1]) / len(R[1]); A['bref_n'] = len(R[1])

    def _b4_tb(self, A, lp):
        y = (lp or {}).get('yaw_imu')
        if A is None or A.get('bref') is None or not _num(y): return None
        return A['bref'] - float(y)

    def _b4_terms(self, tb, v, g, gz):
        B = self.band; kap = interp(B['kappa_turn'], tb)
        prof = float(self.c['_c_eff_us']) + kap / g
        gt = 0.
        if _num(gz) and B.get('k_gz_us'): gt = max(-float(B['gz_lim_us']), min(float(B['gz_lim_us']), float(B['k_gz_us']) * (float(gz) + float(v) * kap)))
        return kap, prof, gt

    def _b4_band(self, A, lp, x, tb, near, valid, v, g, st, centre, hi, a_out):
        B = self.band
        w = max(0., min(1., (x - float(B['x0_m'])) / max(1e-3, float(B['ramp_m']))))
        if w <= 0.: return None
        kap, prof, gt = self._b4_terms(tb, v, g, (lp or {}).get('gz_imu'))
        kr = max(1e-3, float(B['kappa_ref'])); ec = (float(B['e_c']) + float(B.get('out_cm') or 0.) * float(B.get('e_per_cm', 0.025))) * kap / kr if B.get('out_cm') else float(B['e_c']) * kap / kr   # c1-band-r1: c4 outward dial
        ect = ec_turn(B, tb)   # band-ec-turn-r1 (c4 band)
        if ect is not None: ec = ect
        et = max(-float(B['e_lo_us']), min(float(B['e_hi_us']), float(B['k_e_us']) * (float(near) - ec))) if (valid and _num(near)) else 0.
        cap = min(float(B['cap_us']), hi)
        tgt = max(centre - float(B.get('floor_below_us') or 0.), min(cap, prof + et + gt))   # floor_below_us 0 = no band command left of the centre (c2-band convention)
        top = max(min(cap, prof + et + gt), min(cap, prof + max(float(B['m_hi_min_us']), float(B['m_hi_us']) * min(1., kap / kr))))
        ended = A.get('end_d') is not None
        band = min(st, top) if ended else min(max(st, tgt), top)   # after the end angle only the top stays (no floor at the centre while fading out)
        ow = None if ended else own_share(B.get('own'), x, near, valid)   # band-own-r1: None = key absent / off (old rule)
        if ow is not None: band = band + ow * (tgt - band)
        r = dict(w=w, out=st + a_out * (band - st), kap=kap, prof=prof, et=et, gt=gt, ec=ec, tgt=tgt, top=top, ended=ended)
        if ow is not None: r['own'] = ow
        return r

    def _b4_info(self, A, tb, bi):
        d = dict(revision='c4-band-r1', ref=None if A.get('bref') is None else round(A['bref'], 2), ref_n=A.get('bref_n'), turned_deg=None if tb is None else round(tb, 1))
        if tb is None: d['state'] = 'NO_REF' if A.get('bref') is None else 'NO_YAW'
        elif bi is None: d['state'] = 'BEFORE_X0'
        else: d.update(state='FADE' if bi['ended'] else 'ON', w=round(bi['w'], 3), kappa=round(bi['kap'], 1), prof_us=round(bi['prof'], 1), e_c=round(bi['ec'], 3), e_us=round(bi['et'], 1),
                       gz_us=round(bi['gt'], 1), target_us=round(bi['tgt'], 1), top_us=round(bi['top'], 1), band_us=round(bi['out'], 1))
        if bi is not None and bi.get('own') is not None: d['own'] = round(bi['own'], 3)   # band-own-r1
        return d

    def lost(self, lp, v, steer, centre, lo, hi, now):
        """c4-band-r1 lost frame (live_pid hook, only while band_on): (steer, info) or None (= the lost policy's own command)."""
        A = self.act; B = self.band
        if not getattr(self, 'band_on', False) or A is None or not B.get('lost_hold') or A.get('bref') is None: return None
        if lp is None or lp.get('lost') or now is None or not _num(lp.get('odo_m')) or A.get('end_d') is not None or A.get('cancel') is not None: return None
        x = float(lp['odo_m']) - A['d_land']; tb = self._b4_tb(A, lp)
        if tb is None or x < float(B['x0_m']) or tb >= float(B['end_deg']): return None
        if self.lost_t0 is None: self.lost_t0 = float(now)
        if float(now) - self.lost_t0 > float(B['lost_max_s']): return None
        c = self.c; vv = float(v) if _num(v) and float(v) > 0.2 else float(c['_v_default']); g = gain(c['_gain_table'], vv)
        kap, prof, gt = self._b4_terms(tb, vv, g, lp.get('gz_imu'))
        tgt = max(float(centre), min(float(B['cap_us']), float(hi), prof + gt))
        w = max(0., min(1., (x - float(B['x0_m'])) / max(1e-3, float(B['ramp_m']))))
        out = max(float(lo), min(float(hi), float(steer) + w * (tgt - float(steer))))
        self.last = out
        return out, dict(revision='c4-band-r1', curve=self.name, state='LOST_HOLD', turned_deg=round(tb, 1), kappa=round(kap, 1), prof_us=round(prof, 1), gz_us=round(gt, 1),
                         w=round(w, 3), lost_s=round(float(now) - self.lost_t0, 3), held_us=round(float(steer), 1), steer_us=round(out, 1))


def load_block(raw, top=None):   # arc2-r1: any LapFF4 block (c3 from arc2.json) -> merged config, None when absent / disabled
    if not isinstance(raw, dict) or not raw.get('enabled'): return None
    t = top if isinstance(top, dict) else DEFAULTS
    c = dict(C4_DEFAULTS); c.update(raw)
    c['_c_eff_us'] = float(raw.get('c_eff_us', t['c_eff_us'])); c['_gain_table'] = raw.get('gain_table', t['gain_table']); c['_v_default'] = float(raw.get('v_default', t['v_default']))
    return c
