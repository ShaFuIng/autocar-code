"""bottom-guide-r1 (offline candidate 2026-11-14, bottom-20261114; NOT deployed): two additions inside the bottom-owner scope
(landing = live_pid down_level_t0 .. curve-4 FLOOR / t_max_s, see bottom_owner.py). Each has its own switch in bottom_guide.json;
enabled false (default) or a missing / bad file = live_pid never calls this module (behaviour identical to without it).

(a) hold  - lost frames: distance-based heading-profile hold instead of bottom-owner-r2's fixed-rate RATE_HOLD.
    Why: r2 holds gz = -20 deg/s whatever the speed and the heading. 40e9 lap 3 (11-14): at the landing the car was yawing left
    (+22 deg/s); r2 commanded 1730 -> 1636 -> 1498 (it backed off when gz overshot to -53) while the car was still 15-20 deg
    behind the turn the successful passes make; 0bc4 lap 2 (11-13, 1.6 m/s): 2 s of RATE_HOLD at -20 deg/s = radius 4.6 m on a
    2.3 m curve. On the 32 IMU passes the heading after the landing follows the distance, not the time (median, deg rel. the yaw
    at the first steep frame: s 0/0.5/1/1.5/2/3/4/4.5 m -> -9/-13/-37/-48/-55/-79/-102/-112, sd 4-8 deg; from 0.5 to 4.5 m
    -24.8 deg/m = the 2.275 m centre-line radius).
    Command = centre + ff + k_psi*(psi - psi_ref(s)) + k_gz*(gz - omega_ref), limited to [-lim_left_us, +lim_right_us]:
      s         = odometer since the landing (rear encoder, m); psi = yaw now - yaw reference (deg, left +);
      psi_ref   = profile (profile_s / profile_deg) at s; reference yaw = the yaw latched at the first steep frame of this slope
                  (ref 'steep'), or the landing yaw + profile_deg[0] when that latch is missing;
      kappa     = profile slope over [s + kappa_ahead_m, + kappa_window_m] (look-ahead for the steering delay), 0..kappa_max_deg_m;
                  omega_ref = -kappa * v (deg/s, right negative);
      ff        = straight_offset_us + kappa / gain(v) (gain = deg/m per us vs speed, dynamics REPORT 11-13: 0.21/0.18/0.16/0.145 at
                  0.6/0.9/1.2/1.4 m/s; straight_offset_us = effective straight point 1513 - centre 1480).
    It acts only where bottom-owner-r2's rate hold acts (scope active, lost streak <= lost_max_s, gz readable) and only for
    s <= s_max_m; outside that the r2 result is kept. hold_pwm (number) caps the PWM of those frames; null = unchanged.
(b) curl  - valid frames: a single-LEFT-boundary estimate whose boundary bends LEFT at the top (image curl > curl_max) for
    curl_frames consecutive frames inside the scope is treated as a lost frame (curve 4 is a right curve; its own boundaries
    always bend right in the image). On the passes 0.4 % of single-left frames in the scope have curl > 1.0 and never two in a
    row (47 passes, 1 frame with curl_frames 2); in 40e9 lap 3 the foreign line had curl 1.15-2.21 on 8 consecutive valid frames
    (0.93-1.48 s after the landing), in 9ba3 1.28-1.59 on 4. Counted on every frame of the scope (valid or not).
(c) cap_lag    (bottom-guide-r2, bottom-20261115) - valid frames where bottom_owner capped the command (RIGHT_CAP / RIGHT_RAMP): the cap
    opens by k_us_per_deg * (lag - deg0), at most max_us, where lag = psi - psi_ref(s) (car behind the distance profile); the command
    becomes min(uncapped PID, cap + extra). A car on or ahead of the profile keeps the bottom-owner cap.
(d) corr_right (r2) - worker result at the level bottom: the junction switch's corridor is kept by bottom-left-r1 (corridor error >= 0)
    but the baseline is more right by > gap and the heading lags the profile by > lag_deg -> the baseline path is used.
(e) lag_lost   (r2) - valid frame inside the scope with heading lag > lag_deg and image error < e_max (camera asks for little right
    while the car is far behind the turn) -> lost frame (the profile hold drives).
Keys absent / null = that part off; with all r2 keys absent the module and live_pid behave exactly as bottom-guide-r1.
Pure arithmetic: no clock, IMU, MCU or file access except load_config(); live_pid passes every input.
Conventions: servo us larger = right; yaw / gz left positive (deg, deg/s). Experiment values, not verified on the car.
"""
import json
import math
from pathlib import Path
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'bottom-guide-r1'
R2_REVISION = 'bottom-guide-r2'   # new log entries only (r1 entries keep REVISION, so a config without r2 keys logs exactly as r1)
R2_KEYS = dict(cap_lag=('k_us_per_deg', 'deg0', 'max_us'), corr_right=('gap', 'lag_deg'), lag_lost=('lag_deg', 'e_max'))
CONFIG_FILE = Path(__file__).with_name('bottom_guide.json')
PROFILE_S = [0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5, 5.75, 6.0]
PROFILE_DEG = [-9.1, -8.2, -12.8, -23.5, -36.5, -45.5, -48.0, -51.3, -55.3, -59.2, -65.6, -72.3, -78.9, -84.2, -91.6, -97.0, -101.9, -107.3, -111.8, -118.0, -124.2, -130.4, -136.6, -142.8, -149.0]   # 4.75-6.0 m: continued at -24.8 deg/m (the 0.5-4.5 m slope)
DEFAULTS = dict(enabled=False, hold=False, curl=False,
                profile_s=PROFILE_S, profile_deg=PROFILE_DEG, kappa_ahead_m=0.25, kappa_window_m=0.5, kappa_max_deg_m=50.,
                k_psi_us_per_deg=8., k_gz_us_per_dps=2., straight_offset_us=33.,
                gain_v=[0.6, 0.9, 1.2, 1.4], gain_deg_m_us=[0.21, 0.18, 0.16, 0.145],
                lim_right_us=350., lim_left_us=100., s_max_m=4.5, hold_pwm=None,
                curl_max=1.0, curl_frames=2, curl_min_points=12, cap_lag=None, corr_right=None, lag_lost=None)
_NUM = ('kappa_ahead_m', 'kappa_window_m', 'kappa_max_deg_m', 'k_psi_us_per_deg', 'k_gz_us_per_dps', 'straight_offset_us', 'lim_right_us', 'lim_left_us', 's_max_m',
        'curl_max', 'curl_frames', 'curl_min_points')


def load_config(path=None):
    """DEFAULTS updated by bottom_guide.json; any error -> DEFAULTS (= disabled)."""
    c = dict(DEFAULTS)
    try:
        f = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        u = dict(c)
        for k in _NUM:
            if k in f and f[k] is not None:
                v = float(f[k])
                if not math.isfinite(v): raise ValueError(k)
                u[k] = v
        for k in ('profile_s', 'profile_deg', 'gain_v', 'gain_deg_m_us'):
            if k in f and f[k] is not None:
                v = [float(x) for x in f[k]]
                if len(v) < 2 or not all(math.isfinite(x) for x in v): raise ValueError(k)
                u[k] = v
        if len(u['profile_s']) != len(u['profile_deg']) or len(u['gain_v']) != len(u['gain_deg_m_us']): raise ValueError('table length')
        if any(b <= a for a, b in zip(u['profile_s'], u['profile_s'][1:])) or any(b <= a for a, b in zip(u['gain_v'], u['gain_v'][1:])): raise ValueError('table order')
        if 'hold_pwm' in f: u['hold_pwm'] = None if f['hold_pwm'] is None else float(f['hold_pwm'])
        u['hold'] = f.get('hold') is True; u['curl'] = f.get('curl') is True
        for k, need in R2_KEYS.items():   # r2: dict with numeric fields, or null / absent = off
            if f.get(k) is not None:
                d = {n: float(f[k][n]) for n in need}
                if not all(math.isfinite(x) for x in d.values()): raise ValueError(k)
                u[k] = d
        u['enabled'] = f.get('enabled') is True and (u['hold'] or u['curl'] or any(u[k] is not None for k in R2_KEYS))
        c = u
    except Exception:
        c = dict(DEFAULTS)
    return c


def interp(x, xs, ys):
    if x <= xs[0]: return float(ys[0])
    if x >= xs[-1]: return float(ys[-1])
    for i in range(1, len(xs)):
        if x <= xs[i]:
            f = (x - xs[i - 1]) / (xs[i] - xs[i - 1]); return float(ys[i - 1] + f * (ys[i] - ys[i - 1]))
    return float(ys[-1])


def curl(points, min_points=12):
    """Image bend of one boundary (list of [x, row], 1280x720): slope dx/drow of the top quarter minus that of the lower half.
    > 0 = the boundary bends LEFT going up the image. None if fewer than min_points points or a part spans < 8 rows."""
    try:
        P = sorted(((float(p[0]), float(p[1])) for p in points), key=lambda q: q[1])
    except Exception:
        return None
    n = len(P)
    if n < max(4, int(min_points)): return None
    def slope(Q):
        r = [q[1] for q in Q]; x = [q[0] for q in Q]
        if max(r) - min(r) < 8: return None
        mr = sum(r) / len(r); mx = sum(x) / len(x)
        d = sum((a - mr) ** 2 for a in r)
        return sum((a - mr) * (b - mx) for a, b in zip(r, x)) / d if d > 0 else None
    a = slope(P[:max(4, n // 4)]); b = slope(P[n // 2:])
    return None if a is None or b is None else a - b


class BottomGuide(object):
    def __init__(self, config=None):
        self.c = dict(DEFAULTS) if config is None else dict(DEFAULTS, **config)
        self.on = bool(self.c.get('enabled'))
        self.t_land = None; self.enc0 = None; self.yaw_land = None; self.yaw_steep = None; self.curl_n = 0; self.s = None

    @classmethod
    def from_file(cls, path=None):
        try: return cls(load_config(path))
        except Exception: return cls(dict(DEFAULTS))

    # ---- landing latch (called by live_pid on every scope frame) ------------------------------------------------------------
    def steep(self, yaw):
        """first steep frame of a slope: latch the reference yaw (None allowed)."""
        self.yaw_steep = None if yaw is None else float(yaw)

    def landing(self, t_land, enc, yaw):
        """scope frame: a new t_land latches the odometer (counts) and the landing yaw; returns True on a new landing."""
        if t_land is None or t_land == self.t_land: return False
        self.t_land = t_land; self.enc0 = enc; self.yaw_land = None if yaw is None else float(yaw); self.curl_n = 0; self.s = None
        return True

    def dist(self, enc, m_per_count=2.22e-4):
        if enc is None or self.enc0 is None: return None
        self.s = max(0., (float(enc) - float(self.enc0)) * m_per_count)
        return self.s

    # ---- (a) lost-frame heading-profile hold --------------------------------------------------------------------------------
    def gain(self, v):
        return interp(max(0., float(v)), self.c['gain_v'], self.c['gain_deg_m_us'])

    def kappa(self, s):
        """reference curvature (deg/m, right +) = profile slope over [s + kappa_ahead_m, s + kappa_ahead_m + kappa_window_m]
        (look-ahead for the ~0.1 s steering delay + play), limited to 0 .. kappa_max_deg_m."""
        c = self.c; a = s + float(c['kappa_ahead_m']); w = max(1e-3, float(c['kappa_window_m']))
        k = -(interp(a + w, c['profile_s'], c['profile_deg']) - interp(a, c['profile_s'], c['profile_deg'])) / w
        return max(0., min(float(c['kappa_max_deg_m']), k))

    def hold(self, s, yaw, gz, v, c0, lo, hi):
        """-> (command, info) or None (switch off, no distance / yaw / gz, or s > s_max_m)."""
        c = self.c
        if not (self.on and c['hold']) or s is None or yaw is None or gz is None: return None
        if s > float(c['s_max_m']): return None
        if self.yaw_steep is not None: ref = self.yaw_steep; ref_src = 'steep'
        elif self.yaw_land is not None: ref = self.yaw_land - float(c['profile_deg'][0]); ref_src = 'land'
        else: return None
        v = 0. if v is None or not math.isfinite(float(v)) else max(0., float(v))
        psi = float(yaw) - ref; pref = interp(s, c['profile_s'], c['profile_deg'])
        kap = self.kappa(s)
        wref = -kap * v
        ff = float(c['straight_offset_us']) + kap / max(1e-3, self.gain(v))
        u = ff + float(c['k_psi_us_per_deg']) * (psi - pref) + float(c['k_gz_us_per_dps']) * (float(gz) - wref)
        u = max(-float(c['lim_left_us']), min(float(c['lim_right_us']), u))
        out = max(float(lo), min(float(hi), float(c0) + u))
        return out, dict(revision=REVISION, mode='PROFILE_HOLD', s_m=round(s, 3), psi_deg=round(psi, 1), psi_ref_deg=round(pref, 1), lag_deg=round(psi - pref, 1), kappa=round(kap, 1),
                         gz=round(float(gz), 1), omega_ref=round(wref, 1), v=round(v, 2), ff_us=round(ff, 1), offset_us=round(u, 1), steer_us=round(out, 1), ref=ref_src)

    def hold_pwm(self, pwm):
        hp = self.c.get('hold_pwm')
        if hp is None or pwm is None: return pwm
        return min(float(pwm), float(hp))

    # ---- (b) curl guard -----------------------------------------------------------------------------------------------------
    def curl_check(self, sources, left_points, valid):
        """every scope frame: -> (reject, info). reject only for a valid frame after curl_frames consecutive left-curling frames."""
        c = self.c
        if not (self.on and c['curl']): return False, None
        side = max(sources, key=sources.get) if sources else None
        k = curl(left_points, c['curl_min_points']) if side == 'left+width' else None
        hit = k is not None and k > float(c['curl_max'])
        self.curl_n = self.curl_n + _RI.w() if hit else 0
        rej = bool(valid) and self.curl_n >= _RI.thr(int(c['curl_frames']))
        return rej, dict(revision=REVISION, mode='CURL', side=side, curl=None if k is None else round(k, 3), run=self.curl_n, reject=rej)

    # ---- r2 (bottom-20261115) -------------------------------------------------------------------------------------------------
    def lag(self, s, yaw):
        """heading lag behind the distance profile (deg, + = car turned less right than the median pass) or None."""
        if s is None or yaw is None: return None
        if self.yaw_steep is not None: ref = self.yaw_steep
        elif self.yaw_land is not None: ref = self.yaw_land - float(self.c['profile_deg'][0])
        else: return None
        return float(yaw) - ref - interp(s, self.c['profile_s'], self.c['profile_deg'])

    def cap_open(self, lag, from_us, steer_us, right_cap_us, s):
        """(c) -> (command, info) or None (off, no lag, s > s_max_m, lag <= deg0, or no change)."""
        c = self.c.get('cap_lag')
        if not self.on or c is None or lag is None or s is None or s > float(self.c['s_max_m']): return None
        extra = max(0., min(float(c['max_us']), float(c['k_us_per_deg']) * (float(lag) - float(c['deg0']))))
        out = min(float(from_us), float(right_cap_us) + extra)
        if out <= float(steer_us): return None
        return out, dict(revision=R2_REVISION, mode='CAP_LAG', lag_deg=round(float(lag), 1), s_m=round(float(s), 3), extra_us=round(extra, 1),
                         capped_us=round(float(steer_us), 1), steer_us=round(out, 1))

    def corr_check(self, ce, be, lag, s):
        """(d) -> True when the kept corridor should give way to the baseline."""
        c = self.c.get('corr_right')
        if not self.on or c is None or ce is None or be is None or lag is None or s is None or s > float(self.c['s_max_m']): return False
        return float(ce) >= 0. and float(be) - float(ce) > float(c['gap']) and float(lag) > float(c['lag_deg'])

    def lag_lost_check(self, e, lag, s):
        """(e) -> True when this valid frame is to be treated as lost."""
        c = self.c.get('lag_lost')
        if not self.on or c is None or e is None or lag is None or s is None or s > float(self.c['s_max_m']): return False
        return float(lag) > float(c['lag_deg']) and float(e) < float(c['e_max'])
