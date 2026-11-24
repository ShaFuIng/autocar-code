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
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'arc2-r1'   # + blind-v-miss (win_to_x0 / log_miss keys) + blind-r4 (blind4-20261117) under blind_c2.r4
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
        c = _sub(raw, 'land', LAND_DEFAULTS); o = cls(c) if c else None
        _ext_attach(o, raw)   # landing-20261119: land.approach / pre.r2.heading 'imu' absent -> nothing attached (o unchanged)
        return o

    def _tail(self, st, lo, hi):
        if self.last is None: return st, None
        md = _RI.step(float(self.c['max_step_us']))
        if abs(st - self.last) <= md: self.last = None; return st, None
        out = self.last + max(-md, min(md, st - self.last)); out = max(float(lo), min(float(hi), out)); self.last = out
        return out, dict(revision=self.REV, state='TAIL', pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=True)

    def step(self, lp, near, v, yaw_imu, yaw_land, steer, lo, hi, eligible, now=None):   # landing-20261119: ext None -> the r1 body below, unchanged
        ex = getattr(self, 'ext', None)
        if ex is None: return self._step_r1(lp, near, v, yaw_imu, yaw_land, steer, lo, hi, eligible, now)
        return ex.step(self, lp, near, v, yaw_imu, yaw_land, steer, lo, hi, eligible, now)

    def _step_r1(self, lp, near, v, yaw_imu, yaw_land, steer, lo, hi, eligible, now=None):
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
            md = _RI.step(float(c['max_step_us']))
            if a < 1.: out = self.last + max(-md, min(md, out - self.last))   # hand-over: rate-limited both directions
        out = max(float(lo), min(float(hi), out))
        self.last = out if out != st else (self.last if a < 1. and self.last is not None else None)
        return out, dict(revision=self.REV, anchor=A['anchor'], state='OWN' if a >= 1. else 'HAND', x_m=round(x, 3), v=round(vv, 3), kappa=round(kap, 1),
                         base_us=round(base, 1), turned_deg=None if turned is None else round(turned, 1), head_us=round(head, 1), y=round(self.y, 3),
                         lower_us=round(lower_eff, 1), upper_us=round(upper, 1), a=round(a, 3), pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=out != st)


# ---- blind-v2 (blind2-20261117, offline candidate, NOT deployed): circle-crossing window law, only with an enabled blind_c2.v2 block.
# MEASURED (ef9a lap 3, 11-17): the r1 law held dpsi +3.2..+4.8 deg for 2 m with the raw command ~1550 (sent ~1605 incl. the +55 play
# compensation) and gz ~0, i.e. the straight point seen through the linkage play was ~35-40 us right of straight_us 1513: a pure
# P law (k_psi 10) has a steady heading error = straight-point error / k_psi (~3.7 deg) and nothing integrates it; the camera trim was
# frozen. v2 = one owner from the end of the curve-2 band to the crossing's end:
#   cmd = straight_us + I + k_psi * (dpsi - tgt) + k_gz * gz          (dpsi = IMU heading - psi_ref, + = left; gz + = turning left)
#   tgt = clip(-k_y * y, +-tgt_lim)                                   (y = lateral estimate, cm, + = car left of the lane centre)
#   I  += ki * (dpsi - tgt) * dt  only while |dpsi - tgt| < i_gate_deg and |gz| < i_gate_gz, |I| <= i_lim (straight-point error)
#   y  : dead reckoning y += 100 * dodo * sin(dpsi) every frame (valid or lost), corrected toward the camera lateral
#        y_cam = 40 * e - 64 * sin(dpsi) on trustworthy frames only: valid, two-line share >= dbl_frac, width ratio in wr_ok and within
#        wr_jump of the previous frame, |de| < jump_e; inside cam_zones always, elsewhere only when |y_cam - y| < y_gate_cm.
#        y starts from the mean of the trustworthy camera laterals seen since the band end (or 0).
#   b  : reference-bias observer: inside cam_zones each trusted frame moves b by kb * (y_cam - y) * metres travelled (|b| <= b_lim);
#        the law and the dead reckoning use dpsi + b (heading relative to the lane as the camera sees it). The 204.7 deg turn constant
#        has p10..p90 203.8..205.7: a 1-2 deg reference error alone drifts 6-12 cm over the window (model).
# Window: from x0 (blend in_m); lost frames keep the law (LivePID._b2_lost); at x1 it hands back only when |dpsi - tgt| <= hand_dpsi and
# |y| <= hand_y_cm, else it extends up to x_ext (before the left bend at C2e+6.8 m) and then fades out over out_m regardless.
# |dpsi| > max_dev_deg no longer releases (v2): the lateral target is frozen (heading recovery only) and the frame is flagged DEV.
V2_DEFAULTS = dict(x0=0.3, in_m=0.4, x1=5.8, x_ext=6.5, out_m=0.4, hand_dpsi=2.0, hand_y_cm=4.0, k_psi=25., k_gz=3.0, ki=20., i_lim=80.,
                   i_gate_deg=4.0, i_gate_gz=15., k_y=0.4, tgt_lim=4.0, cam_zones=[[0.8, 2.7]], dbl_frac=0.6, wr_ok=[0.65, 0.85], wr_jump=0.05,
                   jump_e=0.1, y_gate_cm=5.0, tau_cam=0.4, lost_age_s=1.5, kb=1.0, b_lim=3.0, y_lim_cm=30., lim_us=250., max_step_us=100., max_dev_deg=12., dev_lat_freeze=True)


def _v2_trust(vis, near, valid, prev):
    """(trustworthy two-line frame?, width ratio)"""
    if not valid or not _num(near) or not isinstance(vis, dict): return False, None
    src = vis.get('sources') or {}; n = sum(v for v in src.values() if _num(v)) or 0
    wr = vis.get('width_ratio'); wr = float(wr) if _num(wr) else None
    return n > 0 and src.get('double', 0) / n >= prev['dbl_frac'] and wr is not None, wr


# ---- blind-r4 (blind4-20261117, offline candidate, NOT deployed): camera-first circle crossing, only with an enabled blind_c2.r4 block
# (takes precedence over v2; key absent / enabled false -> r1 / v2 exactly as before).
# MEASURED (34 passes 11-17, out/zone_profile.txt, out/ref_z3.3.txt): C2e+0.3..2.5 m two-line frames 75-98 %, camera lateral bias vs dead
# reckoning |p90| <= 4.4 cm up to 3.25 m; 3.5..5.25 m single-line 87-100 %, fallback corridor up to 63 %, bias |p90| 7-18 cm; <= 5 cm again
# from 5.5 m; the left bend starts at ~C2e+6.8 m. Hence:
#   x < x0 (3.3)          : untouched, the camera PID owns it; the frames feed the reference / trim estimators only
#   x0 .. x0+in_m         : distance blend in; x0+in_m .. x1 (5.3) heading hold; x1 .. x1+out_m (5.6) distance blend out; then released
#   cmd = straight_us + I + k_psi * (yaw - ref) + k_gz * gz + trim     (yaw + = left, gz + = turning left; straight_us = shared centre via
#         single_centre when that is on), limited to straight_us +- lim_us, per-frame change <= max_step_us (time based), PWM untouched
#   ref : distance-weighted mean IMU heading over [x0 - ref_m, x0) (the heading the camera carried the car in with). If that window swings
#         (heading range > swing_deg or |mean gz| > swing_gz) -> the later [x0 - ref_late_m, x0) window if its range <= late_deg; else the
#         r1 map reference (ref_anchor C1e mean heading + ref_off_deg) when available; if the chosen value differs from the map reference by
#         more than agree_deg -> map. Nothing usable -> no hold (camera keeps the frame).
#   cam_ref_m (null = off): PRE reference replaced by the camera-corrected lane direction over [x0 - cam_ref_m, x0): distance-weighted mean
#         heading minus the slope of the camera lateral y_cam = 40 e - 64 sin(yaw - L) (cm per m, iterated), within +-cam_lim_deg of PRE.
#   trim: k_cam * e_lp, |trim| <= trim_us. e_lp = low-pass (tau_s, time) of the near error, initialised on the camera-owned frames from
#         trim_from to x0; inside the window a frame is fed only when it belongs to a consistent run >= cons_s (valid, |de| < jump_e,
#         dominant line source unchanged); its step e - e_lp is clipped to +-cons_e (a slow consistent drift is followed at a bounded rate,
#         a single offset frame moves e_lp by at most cons_e * dt / (tau + dt)). Jumps / switches / lost frames: not fed (trim held).
#   I   : optional (ki 0 = off); |I| <= i_lim, only in HOLD while |yaw - ref| < i_gate_deg and |gz| < i_gate_gz; reset every window.
#   |yaw - ref| > max_dev_deg -> released for the rest of the window (camera).
#   lost (invalid) frames inside the active window (x >= x0): same law, trim held, lap_pos extrapolated with the window speed (<= lost_age_s);
#   before x0 lost frames are not touched (existing lost policy).
R4_DEFAULTS = dict(x0=3.3, in_m=0.2, x1=5.3, out_m=0.3, ref_m=0.8, ref_late_m=0.4, swing_deg=4.0, swing_gz=12.0, late_deg=2.5, agree_deg=4.0,
                   k_psi=12., k_gz=1.2, ki=0., i_lim=20., lim_us=150., max_step_us=60., k_cam=100., trim_us=36., tau_s=0.5, trim_from=2.3,
                   jump_e=0.08, cons_s=0.15, cons_e=0.2, max_dev_deg=12., lost_age_s=1.0, cam_ref_m=None, cam_lim_deg=2.5, cam_min_n=6,
                   i_gate_deg=3., i_gate_gz=15.)


def _r4_dom(vis):
    if not isinstance(vis, dict): return None
    src = vis.get('sources')
    if not isinstance(src, dict) or not src: return None
    return max(sorted(src.items()), key=lambda kv: kv[1] if _num(kv[1]) else -1)[0]


class BlindR4Mixin:
    r4 = None
    R4_KEY = 'r4'; R4_REV = 'arc2-blind-r4'   # blind-v-r2 (blind-v-20261119): the V-crossing instance sets 'r2' / 'arc2-blind-v-r2'

    def _r4_init(self):
        c = dict(R4_DEFAULTS); c.update(self.c[self.R4_KEY]); self.r4 = c; self.lost_on = True   # lost_on: LivePID._b2_lost hook (w / psi / done)
        self.w = None

    def _r4_reset(self, key):
        self.w = dict(key=key, smp=[], esmp=[], how=None, elp=None, ep=None, dp=None, run_t=None, tp=None, I=0., v=None, odo=None, t=None, done=False, dev=False)
        self.psi = None

    def _r4_ref(self, lap):
        """reference heading at x0 (latched once). returns (ref or None, how)."""
        c = self.r4; s = self.w['smp']; x0 = float(c['x0'])
        R = getattr(self, 'cref', None); mapref = None
        if R is not None and len(R[1]) >= int(self.c.get('ref_min_n', 3)) and R[0][0] == lap: mapref = sum(R[1]) / len(R[1]) + float(self.c.get('ref_off_deg', 0.))

        def win(L):
            q = [a for a in s if x0 - L <= a[0] < x0]
            if len(q) < 3 or (x0 if c.get('win_to_x0') else q[-1][0]) - q[0][0] < 0.5 * L: return None   # blind-v-miss: win_to_x0 (absent -> unchanged)
            xs = [a[0] for a in q] + [x0]; ws = [max(1e-3, xs[i + 1] - xs[i]) for i in range(len(q))]
            ys = [a[1] for a in q]; gs = [a[2] for a in q]
            return sum(y * w for y, w in zip(ys, ws)) / sum(ws), max(ys) - min(ys), abs(sum(gs) / len(gs))
        a = win(float(c['ref_m']))
        if a is not None and a[1] <= float(c['swing_deg']) and a[2] <= float(c['swing_gz']):
            ref, how = a[0], 'PRE'
            if c.get('cam_ref_m') is not None:
                Lc = self._r4_camref(win)
                if Lc is not None: ref, how = max(ref - float(c['cam_lim_deg']), min(ref + float(c['cam_lim_deg']), Lc)), 'PRE+CAM'
        else:
            b = win(float(c['ref_late_m']))
            if b is not None and b[1] <= float(c['late_deg']) and b[2] <= float(c['swing_gz']): ref, how = b[0], 'LATE'
            elif mapref is not None: return mapref, 'MAP'
            else: return None, 'NONE'
        if mapref is not None and abs(ref - mapref) > float(c['agree_deg']): return mapref, how + '>MAP'
        return ref, how

    def _r4_camref(self, win):
        """lane direction over [x0 - cam_ref_m, x0) implied by the heading samples and the camera lateral (None if too few frames)."""
        c = self.r4; x0 = float(c['x0']); Lm = float(c['cam_ref_m'])
        q = [a for a in self.w['esmp'] if x0 - Lm <= a[0] < x0]
        b = win(Lm)
        if b is None or len(q) < int(c['cam_min_n']) or (x0 if c.get('win_to_x0') else q[-1][0]) - q[0][0] < 0.6 * Lm: return None   # blind-v-miss: win_to_x0
        xs = [a[0] for a in q]; mx = sum(xs) / len(xs); sxx = sum((u - mx) ** 2 for u in xs)
        if sxx <= 1e-9: return None
        L = b[0]
        for _ in range(4):
            yc = [40. * a[2] - 64. * math.sin(math.radians(a[1] - L)) for a in q]; my = sum(yc) / len(yc)
            k = sum((u - mx) * (v - my) for u, v in zip(xs, yc)) / sxx   # cm per m
            L = b[0] - math.degrees(math.atan(k / 100.))
        return L

    def _r4_trim(self, near, valid, vis, now, gate):
        """camera trim estimator. gate False: pre-window initialisation (jump rule only); True: in-window consistency rule."""
        c = self.r4; W = self.w
        e = float(near) if valid and _num(near) else None; d = _r4_dom(vis) if e is not None else None
        ok = e is not None and W['ep'] is not None and abs(e - W['ep']) < float(c['jump_e']) and d == W['dp']
        if not ok: W['run_t'] = None
        elif W['run_t'] is None: W['run_t'] = W['tp'] if W['tp'] is not None else now
        fed = False
        if e is not None:
            dt = 1. / 30. if W['tp'] is None or now is None else max(0.01, min(0.3, float(now) - W['tp']))
            if not gate:
                if W['elp'] is None or ok: W['elp'] = e if W['elp'] is None else W['elp'] + dt / (float(c['tau_s']) + dt) * (e - W['elp']); fed = True
            elif ok and W['elp'] is not None and now is not None and W['run_t'] is not None and float(now) - W['run_t'] >= float(c['cons_s']):
                W['elp'] += dt / (float(c['tau_s']) + dt) * max(-float(c['cons_e']), min(float(c['cons_e']), e - W['elp'])); fed = True   # step clipped
        W['ep'] = e; W['dp'] = d
        if now is not None: W['tp'] = float(now)
        return fed

    def _r4_law(self, x, dpsi, gz, st, lo, hi, dt, lost):
        c = self.r4; W = self.w
        g = float(gz) if _num(gz) else 0.
        trim = 0. if W['elp'] is None else max(-float(c['trim_us']), min(float(c['trim_us']), float(c['k_cam']) * W['elp']))
        if float(c['ki']) > 0. and not lost and x >= float(c['x0']) + float(c['in_m']) and x <= float(c['x1']) and abs(dpsi) < float(c['i_gate_deg']) and abs(g) < float(c['i_gate_gz']):
            W['I'] = max(-float(c['i_lim']), min(float(c['i_lim']), W['I'] + float(c['ki']) * dpsi * dt))
        s0 = float(self.c['straight_us'])
        cmd = s0 + W['I'] + float(c['k_psi']) * dpsi + float(c['k_gz']) * g + trim
        cmd = max(s0 - float(c['lim_us']), min(s0 + float(c['lim_us']), cmd))
        x0, x1 = float(c['x0']), float(c['x1'])
        a = min(1., max(0., (x - x0) / max(1e-3, float(c['in_m']))))
        if x > x1: a = min(a, max(0., 1. - (x - x1) / max(1e-3, float(c['out_m']))))
        out = st + a * (cmd - st)
        md = _RI.step(float(c['max_step_us']))
        if self.last is not None: out = self.last + max(-md, min(md, out - self.last))
        out = max(float(lo), min(float(hi), out)); self.last = out
        if x > x1 and a <= 0.: W['done'] = True
        return out, dict(revision=self.R4_REV, state='HAND' if x > x1 else 'HOLD' if a >= 1. else 'BLEND', x_m=round(x, 3), how=W['how'],
                         psi_ref=round(self.psi, 2), dpsi=round(dpsi, 2), gz=round(g, 1), e_lp=None if W['elp'] is None else round(W['elp'], 3),
                         trim_us=round(trim, 1), I_us=round(W['I'], 1), cmd_us=round(cmd, 1), a=round(a, 3), pid_steer_us=round(st, 1),
                         steer_us=round(out, 1), applied=out != st, lost=bool(lost))

    def _r4_step(self, lp, near, valid, yaw, gz, steer, lo, hi, eligible, now, vis, lost=False):
        c = self.r4; st = float(steer)
        if self.c.get('ref_anchor') is not None and not lost: self._carry(lp, yaw)
        if not lp or lp.get('lost') or lp.get('anchor') != self.c['anchor'] or not _num(lp.get('since_anchor_m')): return self._release(st, lo, hi)
        if now is not None and _num(lp.get('t')) and float(now) - float(lp['t']) > float(self.c['max_age_s']): return self._release(st, lo, hi)
        x = float(lp['since_anchor_m'])
        key = (lp.get('lap'), None if not _num(lp.get('anchor_d')) else round(float(lp['anchor_d']), 2))
        if self.w is None or self.w.get('key') != key: self._r4_reset(key)
        W = self.w
        odo = float(lp['odo_m']) if _num(lp.get('odo_m')) else None
        dt = 1. / 30. if W['t'] is None or now is None else max(0.005, min(0.2, float(now) - W['t']))
        if odo is not None and W['odo'] is not None and odo > W['odo'] and dt > 0.:
            vv = min(2.5, (odo - W['odo']) / dt); W['v'] = vv if W['v'] is None else W['v'] + 0.3 * (vv - W['v'])
        if odo is not None: W['odo'] = odo
        if now is not None: W['t'] = float(now)
        x0 = float(c['x0'])
        if x < x0:   # camera-owned: record heading samples and the camera trim, do not touch the frame
            if not lost and _num(yaw) and x >= x0 - max(float(c['ref_m']), float(c['ref_late_m']), float(c.get('cam_ref_m') or 0.)) - 0.2:   # blind-v-r2: heading samples also cover cam_ref_m (cam_ref_m null -> unchanged)
                W['smp'].append((x, float(yaw), float(gz) if _num(gz) else 0.))
            if not lost and eligible and x >= float(c['trim_from']): self._r4_trim(near, valid, vis, now, False)
            if c.get('cam_ref_m') is not None and not lost and eligible and valid and _num(near) and _num(yaw) and x >= x0 - float(c['cam_ref_m']) - 0.2:
                W['esmp'].append((x, float(yaw), float(near)))
            return self._release(st, lo, hi)
        if W['done'] or W['dev'] or x > float(c['x1']) + float(c['out_m']) + 0.5: return self._release(st, lo, hi)
        if W['how'] is None:
            ref, how = self._r4_ref(lp.get('lap')); W['how'] = how
            if ref is None:
                W['done'] = True; o, inf = self._release(st, lo, hi)
                if inf is None and c.get('log_miss'): inf = self._r4_miss(x, how, st, o)   # blind-v-miss: log_miss (absent -> unchanged: no record)
                return o, inf
            self.psi = ref
        if self.psi is None or not _num(yaw): return self._release(st, lo, hi)
        if not eligible and not lost: return st, None
        dpsi = float(yaw) - self.psi
        if abs(dpsi) > float(c['max_dev_deg']):
            W['dev'] = True; o, inf = self._release(st, lo, hi)
            return o, dict(revision=self.R4_REV, state='DEV', x_m=round(x, 3), how=W['how'], dpsi=round(dpsi, 2), pid_steer_us=round(st, 1), steer_us=round(o, 1), applied=o != st)
        fed = False if lost else self._r4_trim(near, valid, vis, now, True)
        out, inf = self._r4_law(x, dpsi, gz, st, lo, hi, dt, lost)
        inf['cam_fed'] = fed
        return out, inf

    def _r4_miss(self, x, how, st, o):   # blind-v-miss: one record per window when no reference could be formed
        c = self.r4; x0 = float(c['x0']); s = self.w['smp']
        lv = sorted(set(round(a[0], 2) for a in s if x0 - max(float(c['ref_m']), float(c['ref_late_m'])) <= a[0] < x0))
        return dict(revision=self.R4_REV, state='NOREF', x_m=round(x, 3), how=how, n_smp=len(s), levels=lv, n_esmp=len(self.w['esmp']),
                    pid_steer_us=round(st, 1), steer_us=round(o, 1), applied=False)

    def _r4_lost(self, lp, yaw, gz, steer, lo, hi, now):
        """lost frame inside an active r4 hold (x >= x0, reference latched): same law, trim held. None otherwise (existing lost policy)."""
        W = self.w
        if W is None or self.psi is None or W.get('done') or W.get('dev') or W.get('how') is None: return None
        if lp and now is not None and _num(lp.get('t')) and _num(lp.get('since_anchor_m')) and W.get('v'):
            age = float(now) - float(lp['t'])
            if float(self.c['max_age_s']) < age <= float(self.r4['lost_age_s']):
                ex = W['v'] * age; lp = dict(lp, since_anchor_m=float(lp['since_anchor_m']) + ex, t=float(now))
                if _num(lp.get('odo_m')): lp['odo_m'] = float(lp['odo_m']) + ex
        if not lp or lp.get('anchor') != self.c['anchor'] or not _num(lp.get('since_anchor_m')) or float(lp['since_anchor_m']) < float(self.r4['x0']): return None
        if (lp.get('lap'), None if not _num(lp.get('anchor_d')) else round(float(lp['anchor_d']), 2)) != W['key']: return None
        out, inf = self._r4_step(lp, None, False, yaw, gz, steer, lo, hi, True, now, None, lost=True)
        return None if inf is None else (out, inf)


class BlindV2Mixin:
    lost_on = False

    def _v2_init(self):
        c = dict(V2_DEFAULTS); c.update(self.c['v2']); self.v2 = c; self.lost_on = True   # v2 parameters live in blind_c2.v2 (shared keys anchor / ref_* / straight_us / max_age_s stay top level)
        self.w = None

    def _v2_reset(self, key):
        self.w = dict(key=key, y=None, ycam=[], I=0., b=0., v=None, odo=None, t=None, last_e=None, last_wr=None, act=False, ext=False, done=False, hand=None, dev=False)

    def _v2_window(self, lp, now):
        c = self.c
        if not lp or lp.get('lost') or lp.get('anchor') != c['anchor'] or not _num(lp.get('since_anchor_m')): return None
        if now is not None and _num(lp.get('t')) and float(now) - float(lp['t']) > float(c['max_age_s']): return None
        return float(lp['since_anchor_m'])

    def _v2_track(self, lp, x, dpsi, near, valid, vis, dt):
        """lateral estimate update (dead reckoning + trustworthy camera)."""
        c = self.v2; W = self.w
        odo = lp.get('odo_m'); odo = float(odo) if _num(odo) else None
        ds = 0. if W['odo'] is None or odo is None else max(0., odo - W['odo'])
        if W['y'] is not None and dpsi is not None:
            W['y'] += 100. * ds * math.sin(math.radians(dpsi + W['b']))
        if ds > 0. and dt > 0.: W['v'] = ds / max(dt, 1e-3) if W['v'] is None else W['v'] + 0.3 * (min(2.5, ds / max(dt, 1e-3)) - W['v'])
        if odo is not None: W['odo'] = odo
        ok, wr = _v2_trust(vis, near, valid, c)
        ycam = None
        if ok and dpsi is not None:
            if not (float(c['wr_ok'][0]) <= wr <= float(c['wr_ok'][1])): ok = False
            if W['last_wr'] is not None and abs(wr - W['last_wr']) > float(c['wr_jump']): ok = False
            if W['last_e'] is not None and abs(float(near) - W['last_e']) >= float(c['jump_e']): ok = False
            if ok: ycam = 40. * float(near) - 64. * math.sin(math.radians(dpsi + W['b']))
        W['last_wr'] = wr if valid else None; W['last_e'] = float(near) if valid and _num(near) else None
        used = False
        if ycam is not None:
            inz = any(float(a) <= x <= float(b) for a, b in c['cam_zones'])
            if W['y'] is None:
                if inz: W['ycam'].append(ycam); used = True
            elif inz or abs(ycam - W['y']) < float(c['y_gate_cm']):
                r = ycam - W['y']; w = dt / (float(c['tau_cam']) + dt); W['y'] += w * r; used = True
                if inz and c.get('kb'): W['b'] = max(-float(c['b_lim']), min(float(c['b_lim']), W['b'] + float(c['kb']) * r * ds))   # reference-bias observer (clean zone only)
        if W['y'] is not None: W['y'] = max(-float(c['y_lim_cm']), min(float(c['y_lim_cm']), W['y']))
        return ycam, used

    def _v2_law(self, x, dpsi, gz, st, lo, hi, dt):
        c = self.v2; W = self.w
        y = W['y'] if W['y'] is not None else 0.
        dev = abs(dpsi) > float(c['max_dev_deg'])
        if dev: W['dev'] = True
        tgt = 0. if (dev and c.get('dev_lat_freeze')) else max(-float(c['tgt_lim']), min(float(c['tgt_lim']), -float(c['k_y']) * y))
        kp, kg, ki, lim = float(c['k_psi']), float(c['k_gz']), float(c['ki']), float(c['lim_us'])
        gd = c.get('guard') if isinstance(c.get('guard'), dict) else None   # blind3: oscillation guard (key absent -> unchanged)
        if gd is not None and W.get('guard'):
            if gd.get('lat_off', True): tgt = 0.
            kp, kg, ki, lim = float(gd.get('k_psi', 8.)), float(gd.get('k_gz', 0.8)), 0., float(gd.get('lim_us', 120.))
        err = dpsi - tgt; g = float(gz) if _num(gz) else 0.
        if gd is not None and not W.get('guard'): self._v2_guard(err, x, gd)
        if abs(err) < float(c['i_gate_deg']) and abs(g) < float(c['i_gate_gz']):
            W['I'] = max(-float(c['i_lim']), min(float(c['i_lim']), W['I'] + ki * err * dt))
        s0 = float(self.c['straight_us'])
        cmd = s0 + W['I'] + kp * err + kg * g
        cmd = max(s0 - lim, min(s0 + lim, cmd))
        # blend: in over in_m from x0; hand-back decided at x1 (latched), extension to x_ext, fade over out_m
        a = min(1., max(0., (x - float(c['x0'])) / max(1e-3, float(c['in_m']))))
        if x >= float(c['x1']) and W['hand'] is None:
            if abs(err) <= float(c['hand_dpsi']) and abs(y) <= float(c['hand_y_cm']): W['hand'] = x
            elif x >= float(c['x_ext']): W['hand'] = x; W['ext'] = True
            else: W['ext'] = True
        if W['hand'] is not None: a = min(a, max(0., 1. - (x - W['hand']) / max(1e-3, float(c['out_m']))))
        out = st + a * (cmd - st)
        md = _RI.step(float(c['max_step_us']))
        if self.last is not None: out = self.last + max(-md, min(md, out - self.last))
        out = max(float(lo), min(float(hi), out)); self.last = out
        if a <= 0. and W['hand'] is not None: W['done'] = True
        inf = dict(revision='arc2-blind-v2', state=('HAND' if W['hand'] is not None else 'EXT' if W['ext'] else 'HOLD' if a >= 1. else 'BLEND'),
                   x_m=round(x, 3), psi_ref=round(self.psi, 2), dpsi=round(dpsi, 2), tgt=round(tgt, 2), y_cm=None if W['y'] is None else round(W['y'], 1),
                   I_us=round(W['I'], 1), gz=round(g, 1), cmd_us=round(cmd, 1), a=round(a, 3), dev=dev, pid_steer_us=round(st, 1), steer_us=round(out, 1),
                   applied=out != st)
        if gd is not None:   # blind3: guard log (only when configured)
            inf['guard'] = bool(W.get('guard')); inf['guard_ext'] = len(W.get('g_ext') or [])
            if W.get('guard') and inf['state'] in ('HOLD', 'EXT'): inf['state'] = 'GUARD'
        return out, inf

    def _v2_guard(self, err, x, gd):   # blind3: extrema of the heading error in HOLD -> latch W['guard'] on growth / persistence / size
        W = self.w; c = self.v2
        if x < float(gd.get('from_m', float(c['x0']) + float(c['in_m']) + 0.6)): return
        if abs(err) >= float(gd.get('a_max', 9.)): W['guard'] = x; return
        hy = float(gd.get('hyst_deg', 0.5)); s = W.get('g_sgn', 0)
        if s == 0:
            if abs(err) > hy: W['g_sgn'] = 1 if err > 0 else -1; W['g_max'] = abs(err)
            return
        if err * s > 0: W['g_max'] = max(W['g_max'], abs(err)); return
        if abs(err) <= hy: return
        ext = W.setdefault('g_ext', []); ext.append(W['g_max']); W['g_sgn'] = -s; W['g_max'] = abs(err)
        a = float(gd.get('a_min', 4.)); r = float(gd.get('ratio', 1.3)); n = int(gd.get('grow_n', 1))
        if len(ext) >= n + 1 and all(ext[-k] >= r * ext[-k - 1] and ext[-k - 1] >= float(gd.get('a_prev', 1.5)) for k in range(1, n + 1)) and ext[-1] >= a: W['guard'] = x
        elif sum(1 for e in ext if e >= a) >= int(gd.get('n_max', 4)): W['guard'] = x

    def _v2_step(self, lp, near, valid, yaw, gz, steer, lo, hi, eligible, now, vis, lost=False):
        c = self.c; st = float(steer)
        if c.get('ref_anchor') is not None and not lost: self._carry(lp, yaw)
        x = self._v2_window(lp, now)
        if x is None: return self._release(st, lo, hi)
        key = (lp.get('lap'), None if not _num(lp.get('anchor_d')) else round(float(lp['anchor_d']), 2))
        if self.w is None or self.w['key'] != key:
            self._v2_reset(key); self.key = key; self.psi = None
        W = self.w
        if self.psi is None:
            R = getattr(self, 'cref', None)
            if R is None or len(R[1]) < int(c.get('ref_min_n', 3)) or R[0][0] != lp.get('lap'): return self._release(st, lo, hi)
            self.psi = sum(R[1]) / len(R[1]) + float(c.get('ref_off_deg', 0.))
        if not _num(yaw): return self._release(st, lo, hi)
        dpsi = float(yaw) - self.psi
        dt = 1. / 30. if W['t'] is None or now is None else max(0.005, min(0.2, float(now) - W['t']))
        if now is not None: W['t'] = float(now)
        ycam, used = self._v2_track(lp, x, dpsi, near, valid and not lost, vis, dt)
        if W['done'] or x > float(self.v2['x_ext']) + float(self.v2['out_m']) + 0.5: return self._release(st, lo, hi)
        if x < float(self.v2['x0']):
            return self._release(st, lo, hi)
        if W['y'] is None: W['y'] = sum(W['ycam']) / len(W['ycam']) if W['ycam'] else 0.
        if not eligible and not lost: return st, None
        out, inf = self._v2_law(x, dpsi + W['b'], gz, st, lo, hi, dt)
        inf['b_deg'] = round(W['b'], 2)
        inf['ycam'] = None if ycam is None else round(ycam, 1); inf['cam_used'] = used
        if lost: inf['lost'] = True
        return out, inf

    def lost(self, lp, yaw, gz, steer, lo, hi, now):
        """lost (invalid) frame inside an active v2 window: same law, no camera. None when the window is not active."""
        if getattr(self, 'r4', None) is not None: return self._r4_lost(lp, yaw, gz, steer, lo, hi, now)   # blind-r4 (blind4-20261117)
        if not self.lost_on or self.w is None or self.psi is None or self.w.get('done'): return None
        if lp and now is not None and _num(lp.get('t')) and _num(lp.get('since_anchor_m')) and self.w.get('v'):   # lap_pos is not advanced on lost frames:
            age = float(now) - float(lp['t'])                                                                    # extrapolate with the window's speed
            if float(self.c['max_age_s']) < age <= float(self.v2['lost_age_s']):
                ex = self.w['v'] * age; lp = dict(lp, since_anchor_m=float(lp['since_anchor_m']) + ex, t=float(now))
                if _num(lp.get('odo_m')): lp['odo_m'] = float(lp['odo_m']) + ex
        x = self._v2_window(lp, now)
        if x is None or x < float(self.v2['x0']) or (lp.get('lap'), None if not _num(lp.get('anchor_d')) else round(float(lp['anchor_d']), 2)) != self.w['key']: return None
        out, inf = self._v2_step(lp, None, False, yaw, gz, steer, lo, hi, True, now, None, lost=True)
        return None if inf is None else (out, inf)


class Blind(BlindR4Mixin, BlindV2Mixin):
    REV = 'arc2-blind-r1'

    def __init__(self, c):
        self.c = c; self.ref = []; self.psi = None; self.elp = None; self.ep = None; self.last = None; self.key = None; self.tp = None
        if isinstance(c.get('r4'), dict) and c['r4'].get('enabled'): self._r4_init()   # blind-r4 (blind4-20261117): key 'r4' absent / disabled -> v2 / r1 unchanged
        elif isinstance(c.get('r2'), dict) and c['r2'].get('enabled') and c.get('ref_anchor') is None:   # blind-v-r2 (blind-v-20261119): V crossing,
            self.R4_KEY = 'r2'; self.R4_REV = self.REV = 'arc2-blind-v-r2'; self._r4_init()                   # key 'r2' absent / disabled -> r1 unchanged
        elif isinstance(c.get('v2'), dict) and c['v2'].get('enabled'): self._v2_init()   # blind-v2 (blind2-20261117): key 'v2' absent / disabled -> r1 unchanged

    @classmethod
    def from_config(cls, raw, key='blind'):   # c2-blind-r1: key 'blind_c2' builds the circle-crossing instance (absent -> None)
        c = _sub(raw, key, BLIND_DEFAULTS); return cls(c) if c else None

    def step(self, lp, near, valid, yaw, gz, steer, lo, hi, eligible, now=None, vis=None):
        if self.r4 is not None: return self._r4_step(lp, near, valid, yaw, gz, steer, lo, hi, eligible, now, vis)   # blind-r4 (blind4-20261117)
        if self.lost_on: return self._v2_step(lp, near, valid, yaw, gz, steer, lo, hi, eligible, now, vis)   # blind-v2
        c = self.c; st = float(steer)
        if c.get('ref_anchor') is not None: self._carry(lp, yaw)   # c2-blind-r1: reference heading carried from an earlier straight
        if not lp or lp.get('lost') or lp.get('anchor') != c['anchor'] or not _num(lp.get('since_anchor_m')):
            return self._release(st, lo, hi)
        if now is not None and _num(lp.get('t')) and float(now) - float(lp['t']) > float(c['max_age_s']): return self._release(st, lo, hi)
        key = (lp.get('lap'), None if not _num(lp.get('anchor_d')) else round(float(lp['anchor_d']), 2))
        if key != self.key: self.key = key; self.ref = []; self.psi = None; self.elp = None; self.ep = None; self.dev = False
        x = float(lp['since_anchor_m'])
        x0, x1 = float(c['x0']), float(c['x1'])
        carry = c.get('ref_anchor') is not None
        if x < x0:
            if not carry and x >= float(c['ref_from']) and _num(yaw): self.ref.append(float(yaw))
            if carry and c.get('trim_from') is not None and x >= float(c['trim_from']) and eligible: self._trim(near, valid, now)   # c2-blind-r1: trim measured before the window
            return self._release(st, lo, hi)
        if x > x1 + float(c['out_m']) or not _num(yaw): return self._release(st, lo, hi)
        if self.psi is None:
            if carry:   # c2-blind-r1: psi = mean heading of the earlier straight + the measured turn to this straight
                R = getattr(self, 'cref', None)
                if R is None or len(R[1]) < int(c.get('ref_min_n', 3)) or R[0][0] != lp.get('lap'): return self._release(st, lo, hi)
                self.psi = sum(R[1]) / len(R[1]) + float(c.get('ref_off_deg', 0.))
            else:
                if not self.ref: return self._release(st, lo, hi)
                self.psi = sum(self.ref) / len(self.ref)
        if c.get('max_dev_deg') is not None and (getattr(self, 'dev', False) or abs(float(yaw) - self.psi) > float(c['max_dev_deg'])): self.dev = True; return self._release(st, lo, hi)   # c2-blind-r1
        if not eligible: return st, None
        # camera trim: slow low-pass of the near error, jumps ignored
        if valid and _num(near) and not (carry and c.get('trim_from') is not None):
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
        md = _RI.step(float(c['max_step_us']))
        if self.last is not None: out = self.last + max(-md, min(md, out - self.last))
        out = max(float(lo), min(float(hi), out)); self.last = out
        return out, dict(revision=self.REV, state='HOLD' if a >= 1. else 'BLEND', x_m=round(x, 3), psi_ref=round(self.psi, 2), dpsi=round(float(yaw) - self.psi, 2),
                         gz=round(g, 1), e_lp=None if self.elp is None else round(self.elp, 3), trim_us=round(trim, 1), cmd_us=round(cmd, 1), a=round(a, 3),
                         pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=out != st)

    def _carry(self, lp, yaw):   # c2-blind-r1: mean IMU heading over ref_anchor + ref_win (this lap's instance of that anchor)
        c = self.c
        if not lp or lp.get('lost') or lp.get('anchor') != c['ref_anchor'] or not _num(lp.get('since_anchor_m')) or not _num(yaw): return
        k = (lp.get('lap'), None if not _num(lp.get('anchor_d')) else round(float(lp['anchor_d']), 2))
        R = getattr(self, 'cref', None)
        if R is None or R[0] != k: R = self.cref = [k, []]
        w = c.get('ref_win') or [4.0, 9.0]
        if float(w[0]) <= float(lp['since_anchor_m']) <= float(w[1]): R[1].append(float(yaw))

    def _trim(self, near, valid, now):   # same low-pass / jump rule as the in-window trim
        c = self.c
        if valid and _num(near):
            e = float(near)
            if self.ep is None or abs(e - self.ep) < float(c['jump_e']):
                dt = 0.1 if self.tp is None or now is None else max(0.02, min(0.3, float(now) - self.tp))
                w = dt / (float(c['tau_s']) + dt); self.elp = e if self.elp is None else self.elp + w * (e - self.elp)
            self.ep = e
        if now is not None: self.tp = float(now)

    def _release(self, st, lo, hi):
        if self.last is None: return st, None
        md = _RI.step(float(self.c['max_step_us']))
        if abs(st - self.last) <= md: self.last = None; return st, None
        out = self.last + max(-md, min(md, st - self.last)); out = max(float(lo), min(float(hi), out)); self.last = out
        return out, dict(revision=self.REV, state='TAIL', pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=True)


# ---- slope-arc-r1 (slope-arc-20261116, offline candidate): item A, slope pre-rotation (arc2.json key 'pre')
PRE_DEFAULTS = dict(enabled=False, anchor='DOWN', x0=1.0, x1=2.55, in_m=0.15, ref_win=[-1.85, -0.3], ref_min_n=3,
                    prof=[[1.0, -1.2], [1.16, -2.0], [1.26, -4.1], [1.37, -5.9], [1.47, -7.2], [1.57, -8.4], [1.68, -9.0], [1.78, -10.1],
                          [1.88, -11.0], [2.6, -11.0]],
                    k_us_deg=8., db_deg=1.0, lim_right_us=80., lim_left_us=30., max_step_us=30.)


def _median(v):
    v = sorted(v); n = len(v)
    return None if n == 0 else (v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2]))


class SlopePre:
    """Bounded heading-reference bias on the last metre of the slope (before yaw_inner, so the inner loop treats it as requested).
    Position x = lap_pos metres since the DOWN crossing (lap odometer; atlas seam at x ~ 2.40). Reference = median lap_pos yaw over
    [DOWN + ref_win[0], DOWN + ref_win[1]] (the upper straight before the slope, from lap_pos's own (D, yaw) history), latched at the
    first DOWN-anchored frame. Target heading = ref + prof(x) (deg, right negative; median of the clean passes, tools/ana2.py).
    bias = clip(k * deadband(psi - target), -lim_left, +lim_right) * fade-in, rate limited max_step_us per eligible frame both ways
    (also the release after landing / end of window). Lost frames: untouched (bias kept)."""
    REV = 'arc2-pre-r1'

    def __init__(self, c): self.c = c; self.key = None; self.ref = None; self.bias = 0.

    @classmethod
    def from_config(cls, raw):
        c = _sub(raw, 'pre', PRE_DEFAULTS); o = cls(c) if c else None
        if o is not None: _pre_r2_attach(o, raw)   # landing-20261119: pre.r2 absent -> o.r2 None (r1)
        return o

    def step(self, anchor, anchor_d, D, lost, yaw, hist, steer, lo, hi, eligible, landed):
        if getattr(self, 'r2', None) is not None: return _pre_r2_step(self, anchor, anchor_d, D, lost, yaw, hist, steer, lo, hi, eligible, landed)
        c = self.c; st = float(steer); tgt = 0.; info = dict(revision=self.REV, state='OFF')
        if not lost and anchor == c['anchor'] and _num(anchor_d) and _num(D) and _num(yaw) and not landed:
            key = round(float(anchor_d), 3)
            if key != self.key:
                a, b = float(c['ref_win'][0]), float(c['ref_win'][1])
                ys = [float(y) for d, y in (hist or []) if float(anchor_d) + a <= float(d) <= float(anchor_d) + b]
                self.key = key; self.ref = _median(ys) if len(ys) >= int(c['ref_min_n']) else None
            x = float(D) - float(anchor_d)
            if self.ref is not None and float(c['x0']) <= x <= float(c['x1']):
                rel = float(yaw) - self.ref; t_ = interp(c['prof'], x); e = rel - t_; db = float(c['db_deg'])
                e2 = 0. if abs(e) <= db else (e - db if e > 0 else e + db)
                tgt = max(-float(c['lim_left_us']), min(float(c['lim_right_us']), float(c['k_us_deg']) * e2))
                tgt *= min(1., (x - float(c['x0'])) / max(1e-3, float(c['in_m'])))
                info = dict(revision=self.REV, state='ON', x_m=round(x, 3), ref=round(self.ref, 2), psi_rel=round(rel, 2), target=round(t_, 2), e=round(e, 2))
            else: info = dict(revision=self.REV, state='WAIT' if self.ref is not None and x < float(c['x0']) else ('NO_REF' if self.ref is None else 'END'), x_m=round(x, 3))
        elif landed: info = dict(revision=self.REV, state='LANDED')
        if not eligible: return st, None
        if tgt == 0. and self.bias == 0.: return st, None
        md = _RI.step(float(c['max_step_us'])); self.bias = self.bias + max(-md, min(md, tgt - self.bias))
        if abs(self.bias) < 1e-9: self.bias = 0.
        out = max(float(lo), min(float(hi), st + self.bias))
        info.update(bias_us=round(self.bias, 1), target_bias_us=round(tgt, 1), pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=out != st)
        return out, info


# ---- landing-r2 (landing-20261119, offline candidate, NOT deployed). Two switchable parts, both only add / raise, never push left:
# (a) pre.r2  : slope pre-rotation, revised. MEASURED (d2af lap 2, 973e lap 1, 0f91 lap 3): the r1 heading came from lap_pos, which
#     integrates one gz per camera frame and holds the last gz across a frame gap; a 0.47-0.54 s gap at the slope top added -4.8..-7.7 deg,
#     so the car looked 'ahead' and r1 pushed LEFT (-17..-30 us). r2: heading = the IMU heading (heading._yaw) that the land stage reads on
#     every valid frame anyway (shared trace, previous frame's value; no new reads); without that trace lap_pos is used only when no
#     frame gap > gap_m happened since the reference window (else state GAP, no action). Bias = clip(k * deadband(psi - target), 0, lim_right)
#     (right only, released at max_step_us), no action if |psi - target| > max_err_deg (reference not trusted).
# (b) land.approach : pre-landing floor of the SENT command. MEASURED (42 landings 11-18): in the last ~0.5 s before LAND8 downhill_dlimit
#     floors the pre-compensation command at the centre when the PID falls, play_comp (+-55, reversal threshold 15 us, output stage)
#     sees a fall and switches LEFT -> the servo gets centre-55 and the car turns left into the landing (sent -45..-15 s before LAND8:
#     <= centre-40 in 16 landings, 12 pressed the outside line, median gz at LAND8 +13.7 deg/s; >= centre+40 in 8, 1 pressed, gz -9.7).
#     approach emulates play_comp from the commands this stage leaves (params duplicated in arc2.json, must equal play_comp.json) and
#     raises the pre-compensation command to the smallest value whose emulated sent command is >= centre + target_us, without forcing
#     a left reversal, never above centre + max_rel_us. Window: lap_pos anchor DOWN, x in [x0, x1] (ends when lap_pos switches to LAND8).
#     After LAND8 the last floor is kept for hold_m (0.3 m) unless the camera says left (a fall from it at the landing would reverse
#     play_comp to the left again); it only raises, so the land band's output is kept when it is higher. The r1 land body is unchanged.
PRE_R2_DEFAULTS = dict(heading='imu', gap_m=0.3, max_err_deg=12., lim_right_us=40., k_us_deg=8., db_deg=1.0, max_step_us=20., min_n=3,
                       x0=1.0, x1=2.55, prof=None)
APPROACH_DEFAULTS = dict(enabled=False, anchor='DOWN', x0=1.5, x1=3.0, target_us=40., max_rel_us=130., e_min=-0.2, max_age_s=0.25, hold_m=0.3,
                         comp=dict(right_us=55., left_us=55., threshold_us=15.))
TRACE_M = 8.
# landing2-20261120 (arc2-land-pulse-r1, offline candidate, NOT deployed): land.approach.pulse -> camera-gated pre-landing floor.
PULSE_DEFAULTS = dict(enabled=False, anchor='DOWN', x_on=1.95, x_off=3.0, sent_us=295., pre_cap_us=240., e0=0.0, e1=0.25, max_age_s=0.25, v_default=1.0)


class _CompEmu:
    """play_comp.PlayComp direction / extreme logic (play-comp-r1), fed with the commands the land stage leaves."""
    def __init__(self, centre, right, left, th): self.dir = 0; self.ext = float(centre); self.r = float(right); self.l = float(left); self.th = float(th)

    def feed(self, raw):
        raw = float(raw); th = self.th
        if self.dir == 0:
            if raw - self.ext >= th: self.dir = 1; self.ext = raw
            elif self.ext - raw >= th: self.dir = -1; self.ext = raw
        elif self.dir > 0:
            if raw >= self.ext: self.ext = raw
            elif self.ext - raw >= th: self.dir = -1; self.ext = raw
        else:
            if raw <= self.ext: self.ext = raw
            elif raw - self.ext >= th: self.dir = 1; self.ext = raw

    def sent(self, raw):   # what play_comp would send for raw (state not changed)
        e = _CompEmu(0., self.r, self.l, self.th); e.dir = self.dir; e.ext = self.ext; e.feed(raw)
        return float(raw) + (self.r if e.dir > 0 else (-self.l if e.dir < 0 else 0.))

    def floor(self, s_min):
        """raw whose emulated sent is >= s_min and closest to it (candidates: keep the direction, or reverse)."""
        th = self.th; c = []
        if self.dir > 0:
            c.append(max(s_min - self.r, self.ext - th + 0.5))                        # stay right: no fall >= th below the extreme
            if s_min + self.l <= self.ext - th: c.append(s_min + self.l)              # reverse to left when the left offset still gives s_min
        elif self.dir < 0:
            c.append(s_min + self.l)                                                  # stay left
            c.append(max(s_min - self.r, self.ext + th))                              # reverse to right
        else: c.append(max(s_min, self.ext + th) if s_min > self.ext else s_min)
        c = [(self.sent(x), x) for x in c]; c = [z for z in c if z[0] >= s_min - 1e-6]
        return min(c)[1] if c else None                                               # the candidate whose SENT command is closest to s_min


class LandExt:
    """landing-r2 extension of LandArc: IMU heading trace for pre.r2 and the pre-landing approach floor.
    land-smooth-20261120 (arc2-land-lost-r1, offline candidate, NOT deployed): land.approach.lost {enabled, max_s} -> lost frames inside the
    approach window (lap_pos DOWN + [x0, x1] of the last update) or within hold_m after LAND8 get the same floor (emulated sent >= centre +
    target_us, <= centre + max_rel_us, only raises), for at most max_s after the last valid approach frame. MEASURED (f675/a5c0/709c/e81e,
    15 landings): 9 had 5-12 lost seam frames in the last 0.2-0.47 s before LAND8; down_straight's GYRO hold (yaw locked at the start of
    the loss) sent centre -78..-204 us on them (the valid-frame floor never sees lost frames); those landings turned only -2 deg in the
    first 0.5 m (-9 without blackout) and -28 deg in the next 0.5 m (-21). The emulator is also fed with these lost commands (play_comp
    sees every frame). Key absent / enabled false -> LandArc.lost_on is not set and live_pid never calls lost()."""
    REV = 'arc2-land-ext-r2'

    def __init__(self, approach, trace):
        self.ap = approach; self.trace = trace; self.emu = None; self.hold = None; self.ap_live = False; self.t_valid = None
        pu = (approach or {}).get('pulse')   # landing2-20261120: land.approach.pulse absent / enabled false -> None (never used)
        self.pulse = dict(PULSE_DEFAULTS, **pu) if isinstance(pu, dict) and pu.get('enabled') else None

    def step(self, land, lp, near, v, yaw_imu, yaw_land, steer, lo, hi, eligible, now=None):
        if self.trace is not None and lp and _num(lp.get('odo_m')) and _num(yaw_imu):
            self.trace.append((float(lp['odo_m']), float(yaw_imu)))
            while self.trace and self.trace[-1][0] - self.trace[0][0] > TRACE_M: self.trace.pop(0)
        out, info = land._step_r1(lp, near, v, yaw_imu, yaw_land, steer, lo, hi, eligible, now)
        a = self.ap
        if a is None: return out, info
        cen = float(land.c['straight_us']); self.c_straight = cen
        if self.emu is None: self.emu = _CompEmu(cen, a['comp']['right_us'], a['comp']['left_us'], a['comp']['threshold_us'])
        why = None; x = None
        if land.act is not None: why = 'LAND_BAND'
        elif not lp or lp.get('lost'): why = 'NO_LAP_POS'
        elif lp.get('anchor') != a['anchor'] or not _num(lp.get('since_anchor_m')): why = 'NOT_IN_WINDOW'
        elif now is not None and _num(lp.get('t')) and float(now) - float(lp['t']) > float(a['max_age_s']): why = 'STALE'
        else:
            x = float(lp['since_anchor_m'])
            if not (float(a['x0']) <= x <= float(a['x1'])): why = 'NOT_IN_WINDOW'
            elif _num(near) and float(near) < float(a['e_min']): why = 'CAMERA_LEFT'
        ai = None; cap = min(float(hi), cen + float(a['max_rel_us']))
        if why is None:
            s_min = cen + float(a['target_us']); f = self.emu.floor(s_min)
            if f is not None: f = min(f, cap)
            self.hold = f
            o2 = out if f is None or out >= f else max(float(lo), f)
            ai = dict(revision=self.REV, state='APPROACH', x_m=round(x, 3), floor_us=None if f is None else round(f, 1), emu_dir=self.emu.dir,
                      emu_ext=round(self.emu.ext, 1), emu_sent_us=round(self.emu.sent(o2), 1), in_steer_us=round(out, 1), steer_us=round(o2, 1), applied=o2 != out)
            out = o2
        elif self.hold is not None and why in ('NOT_IN_WINDOW', 'LAND_BAND') and lp and not lp.get('lost') and lp.get('anchor') == 'LAND8'                 and _num(lp.get('since_anchor_m')) and float(lp['since_anchor_m']) <= float(a['hold_m']) and not (_num(near) and float(near) < float(a['e_min'])):
            # just after LAND8: keep the last floor for hold_m (a fall from it would reverse play_comp to the left at the landing itself)
            f = min(self.hold, cap); o2 = out if out >= f else max(float(lo), f)
            ai = dict(revision=self.REV, state='HOLD', x_land_m=round(float(lp['since_anchor_m']), 3), floor_us=round(f, 1), emu_dir=self.emu.dir,
                      emu_sent_us=round(self.emu.sent(o2), 1), in_steer_us=round(out, 1), steer_us=round(o2, 1), applied=o2 != out)
            out = o2
        elif lp and lp.get('anchor') != a['anchor']: self.hold = None
        ap_ai = ai
        if self.pulse is not None:   # landing2-20261120: only with land.approach.pulse enabled (absent -> this block never runs)
            pr = self._pulse(lp, near, v, now, out, lo, hi, eligible)
            if pr is not None:
                out = pr[0]
                if ai is None: ai = pr[1]
                else: ai['pulse'] = pr[1]
        self.emu.feed(out)
        self.ap_live = ap_ai is not None; self.t_valid = float(now) if _num(now) else None   # land-smooth-20261120: state for lost()
        if ai is None: return out, info
        if info is not None: ai['land'] = info
        return out, ai

    def _pulse(self, lp, near, v, now, st, lo, hi, eligible):
        """landing2-20261120 (arc2-land-pulse-r1): valid frame on the slope, lap_pos anchor DOWN, distance extrapolated with the wheel speed
        since the last lap_pos update xe = since_anchor + v * min(age, max_age_s) in [x_on, x_off], and still on the slope (the land stage's
        eligible flag is False until live_pid detects the landing = LAND8, pitch back above -8 deg) -> raise the pre-compensation command
        so that the emulated sent command is >= centre + g * sent_us, g = clip((near - e0) / (e1 - e0), 0, 1) (camera says the lane ahead is
        to the right; ~0 when the car comes down on the right side). Only raises, never above centre + pre_cap_us (= bottom_owner cap_us,
        so the capped command after the landing is not a fall for play_comp). MEASURED (11-20, 18 landings):
        the post-seam straight is ~22 deg right of the landing heading (sd 2.7); the 11 landings with camera error >= 0.09 at 0.3-0.4 s before
        LAND8 all ran 9-21 cm outward before the heading caught up; the 7 with error <= 0.07 (all with lost seam frames) did not."""
        P = self.pulse
        if eligible or not lp or lp.get('lost') or lp.get('anchor') != P['anchor'] or not _num(lp.get('since_anchor_m')) or not _num(near): return None
        age = 0.
        if _num(now) and _num(lp.get('t')): age = max(0., min(float(P['max_age_s']), float(now) - float(lp['t'])))
        vv = float(v) if _num(v) and float(v) > 0.2 else float(P['v_default'])
        xe = float(lp['since_anchor_m']) + vv * age
        if not (float(P['x_on']) <= xe <= float(P['x_off'])): return None
        g = max(0., min(1., (float(near) - float(P['e0'])) / max(1e-3, float(P['e1']) - float(P['e0']))))
        cen = float(self.c_straight); f = None; out = float(st)
        if g > 0.:
            f = self.emu.floor(cen + g * float(P['sent_us']))
            if f is not None:
                f = min(f, float(hi), cen + float(P['pre_cap_us'])); out = st if st >= f else max(float(lo), f)
        return out, dict(revision='arc2-land-pulse-r1', state='PULSE', x_e=round(xe, 3), g=round(g, 3), floor_us=None if f is None else round(f, 1),
                         emu_dir=self.emu.dir, emu_sent_us=round(self.emu.sent(out), 1), in_steer_us=round(float(st), 1), steer_us=round(out, 1), applied=out != st)

    def lost(self, lp, steer, lo, hi, now=None):
        """land-smooth-20261120: lost frame -> (steer, info) with the approach floor, or None (not in the window / off / too long)."""
        a = self.ap; L = (a or {}).get('lost')
        if not isinstance(L, dict) or not L.get('enabled') or not self.ap_live or self.emu is None: return None
        if _num(now) and self.t_valid is not None and float(now) - self.t_valid > float(L.get('max_s', 0.6)): return None
        if not lp or lp.get('lost') or not _num(lp.get('since_anchor_m')): return None
        an = lp.get('anchor'); x = float(lp['since_anchor_m'])
        if not ((an == a['anchor'] and float(a['x0']) <= x <= float(a['x1'])) or (an == 'LAND8' and x <= float(a['hold_m']))): return None
        cen = float(self.c_straight); cap = min(float(hi), cen + float(a['max_rel_us']))
        st = float(steer); f = self.emu.floor(cen + float(a['target_us']))
        if f is None: self.emu.feed(st); return None
        f = min(f, cap); out = st if st >= f else max(float(lo), f)
        self.emu.feed(out)
        return out, dict(revision='arc2-land-lost-r1', state='LOST_FLOOR', anchor=an, x_m=round(x, 3), floor_us=round(f, 1), emu_dir=self.emu.dir,
                         held_us=round(st, 1), steer_us=round(out, 1), applied=out != st)


def _ext_attach(land, raw):
    """attach LandExt when land.approach is enabled or pre.r2 uses the IMU heading; otherwise land.ext stays absent (r1)."""
    global _TRACE
    _TRACE = None
    if land is None: return
    ap = (raw.get('land') or {}).get('approach') if isinstance(raw, dict) else None
    ap = dict(APPROACH_DEFAULTS, **ap) if isinstance(ap, dict) and ap.get('enabled') else None
    if ap is not None: ap['comp'] = dict(APPROACH_DEFAULTS['comp'], **(ap.get('comp') or {}))
    pr = raw.get('pre') if isinstance(raw, dict) else None
    want = isinstance(pr, dict) and pr.get('enabled') and isinstance(pr.get('r2'), dict) and pr['r2'].get('enabled') and pr['r2'].get('heading', 'imu') == 'imu'
    tr = [] if want else None
    if ap is None and tr is None: return
    land.ext = LandExt(ap, tr); _TRACE = tr
    if ap is not None and isinstance(ap.get('lost'), dict) and ap['lost'].get('enabled'): land.lost_on = True   # land-smooth-20261120 (absent -> attribute not set)


_TRACE = None


def _pre_r2_attach(pre, raw):
    r = (raw.get('pre') or {}).get('r2') if isinstance(raw, dict) else None
    if not isinstance(r, dict) or not r.get('enabled'): pre.r2 = None; return
    c = dict(PRE_R2_DEFAULTS, **r)
    if c['prof'] is None: c['prof'] = pre.c['prof']
    pre.r2 = c; pre.trace = _TRACE if c['heading'] == 'imu' else None
    pre.rkey = None; pre.rref = None; pre.rsrc = None


def _pre_r2_step(self, anchor, anchor_d, D, lost, yaw, hist, steer, lo, hi, eligible, landed):
    c = self.c; r = self.r2; st = float(steer); tgt = 0.; info = dict(revision='arc2-pre-r2', state='OFF')
    if not lost and anchor == c['anchor'] and _num(anchor_d) and _num(D) and not landed:
        a, b = float(c['ref_win'][0]), float(c['ref_win'][1]); ad = float(anchor_d); x = float(D) - ad
        tr = self.trace if (self.trace is not None and r['heading'] == 'imu') else None
        src = 'imu' if tr is not None else 'lap_pos'
        key = round(ad, 3)
        if key != self.rkey or self.rref is None:   # reference: retried until it has min_n samples (the IMU trace starts with the land stage)
            pts = tr if tr is not None else [(float(d), float(y)) for d, y in (hist or [])]
            ys = [y for d, y in pts if ad + a <= d <= ad + b]
            self.rkey = key; self.rref = _median(ys) if len(ys) >= int(r['min_n']) else None; self.rsrc = src
        cur = (tr[-1][1] if tr else None) if tr is not None else (float(yaw) if _num(yaw) else None)
        gap = bool(tr) and float(D) - tr[-1][0] > float(r['gap_m'])   # IMU trace not fed for gap_m (lost frames): its last heading is stale
        if tr is None and hist:
            pts = [(float(d), float(y)) for d, y in hist if float(d) >= ad + a]
            gap = any(d1 - d0 > float(r['gap_m']) for (d0, _), (d1, _) in zip(pts, pts[1:]))
        if self.rref is None or cur is None: info = dict(revision='arc2-pre-r2', state='NO_REF', src=src, x_m=round(x, 3))
        elif gap: info = dict(revision='arc2-pre-r2', state='GAP', src=src, x_m=round(x, 3))
        elif float(r['x0']) <= x <= float(r['x1']):
            rel = cur - self.rref; t_ = interp(r['prof'], x); e = rel - t_; db = float(r['db_deg'])
            if abs(e) > float(r['max_err_deg']): info = dict(revision='arc2-pre-r2', state='DISTRUST', src=src, x_m=round(x, 3), psi_rel=round(rel, 2), target=round(t_, 2))
            else:
                e2 = 0. if abs(e) <= db else (e - db if e > 0 else e + db)
                tgt = max(0., min(float(r['lim_right_us']), float(r['k_us_deg']) * e2))
                tgt *= min(1., (x - float(r['x0'])) / max(1e-3, float(c['in_m'])))
                info = dict(revision='arc2-pre-r2', state='ON', src=src, x_m=round(x, 3), ref=round(self.rref, 2), psi_rel=round(rel, 2), target=round(t_, 2), e=round(e, 2))
        else: info = dict(revision='arc2-pre-r2', state='WAIT' if x < float(r['x0']) else 'END', src=src, x_m=round(x, 3))
    elif landed: info = dict(revision='arc2-pre-r2', state='LANDED')
    if not eligible: return st, None
    if tgt == 0. and self.bias == 0.: return st, None
    md = _RI.step(float(r['max_step_us'])); self.bias = max(0., self.bias + max(-md, min(md, tgt - self.bias)))
    if abs(self.bias) < 1e-9: self.bias = 0.
    out = max(float(lo), min(float(hi), st + self.bias))
    info.update(bias_us=round(self.bias, 1), target_bias_us=round(tgt, 1), pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=out != st)
    return out, info
