"""xp-corridor-r2 (integration v2, 2026-11-13; r1 in r1_archive/): heading corridor after curve 2 (circle crossing with the V marking).

Problem (ffc2 2026-11-12, b44b / ed96 2026-11-13): after right curve 2 the camera path switched onto the V's left arm and the car
followed it onto another lane, always turning LEFT (about 33 deg in 2-2.5 s). Normal passes change the heading by -6.4..+0.8 deg.

The camera keeps steering; the gyro only bounds how far the heading may leave the entry direction:
  scope   from the crossing window's first active frame (curve-2 end distance known) until dist_after >= end_m;
  psi0    median gyro yaw over the first ref_s of the scope;
  dev     = yaw - psi0, wrapped to (-180, 180], LEFT positive (gyro convention);
  sides   left (dev > 0): soft_deg / hard_deg; right (dev < 0): soft_right_deg / hard_right_deg, null = right side off (r2 default:
          psi0 is captured while curve 2 is still finishing, so normal passes reach -10..-18 deg);
  IN      |dev| <= soft of that side (or that side off): command unchanged;
  SOFT    soft < |dev| < hard: command + k_us_per_deg * (|dev| - soft) toward psi0 (dev > 0 = car points left -> + us = right), then
          at least centre + k_soft_floor_us_per_deg * (|dev| - soft) toward psi0 (r2 floor; a camera command already further toward
          psi0 is kept);
  HARD    |dev| >= hard: camera ignored, command = centre + clamp(k_hold * dev + k_gz * gz, +-lim_us) (gz left positive: rotating
          further left -> more right), never on the far side of centre; stays HARD until |dev| < exit_deg (r2; exit_right_deg for the
          right side, null = exit_deg) or dev changes sign;
  lost    invalid frames inside the scope while |dev| > soft (or HARD latched): the HARD law instead of the existing hold.
Servo: larger us = right. Pure logic, standard library only; live_pid supplies time, yaw, gz, distances and the command.
"""
import json, math
from pathlib import Path

REVISION = 'xp-corridor-r2'
CONFIG_FILE = Path(__file__).parent / 'xp_corridor.json'
DEFAULTS = dict(enabled=False, end_m=3.6, ref_s=0.3, soft_deg=10.0, hard_deg=13.0, soft_right_deg=None, hard_right_deg=None,
                exit_deg=6.0, exit_right_deg=None, k_us_per_deg=25.0, k_soft_floor_us_per_deg=20.0, k_hold_us_per_deg=20.0,
                k_gz_us_per_dps=4.0, lim_us=350.0, lost_hold=True)
NULLABLE = ('soft_right_deg', 'hard_right_deg', 'exit_right_deg')


def load_config(path=None):
    """DEFAULTS updated by xp_corridor.json (known keys only). Missing / unreadable file or bad value -> disabled (never raises)."""
    c = dict(DEFAULTS)
    try:
        f = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        for k in DEFAULTS:
            if k in f and f[k] is not None:
                c[k] = bool(f[k]) if isinstance(DEFAULTS[k], bool) else float(f[k])
            elif k in NULLABLE and k in f:
                c[k] = None
        rs, rh = c['soft_right_deg'], c['hard_right_deg']
        ok = (0. < c['soft_deg'] < c['hard_deg'] and 0. <= c['exit_deg'] <= c['soft_deg'] and c['ref_s'] >= 0. and c['end_m'] > 0.
              and c['lim_us'] >= 0. and c['k_soft_floor_us_per_deg'] >= 0. and (rs is None) == (rh is None)
              and (rs is None or 0. < rs < rh) and (c['exit_right_deg'] is None or rs is None or 0. <= c['exit_right_deg'] <= rs))
        if not ok: raise ValueError('xp_corridor.json values')
    except FileNotFoundError:
        c = dict(DEFAULTS)
    except Exception:
        c = dict(DEFAULTS); c['enabled'] = False
    return c


def wrap(d):
    """angle difference wrapped to (-180, 180]."""
    d = math.fmod(float(d), 360.)
    if d > 180.: d -= 360.
    elif d <= -180.: d += 360.
    return d


def _median(v):
    s = sorted(v); n = len(s)
    return s[n // 2] if n % 2 else .5 * (s[n // 2 - 1] + s[n // 2])


class XpCorridor(object):
    def __init__(self, cfg=None):
        self.c = dict(DEFAULTS) if cfg is None else dict(cfg)
        self.enabled = bool(self.c.get('enabled'))
        self.reset()

    def reset(self):
        self.active = False; self.end_ref = None; self.started_ref = None; self.t0 = None; self.ref = []; self.psi0 = None
        self.mode = None; self.dist_after = None; self.hard_sign = 0

    # ---- scope -------------------------------------------------------------------------------------------------------------
    def scope(self, t, window_curve_end, dist):
        """update the scope; window_curve_end = curve-2 end odometry (m) on a frame where the crossing window is active, else None;
        dist = current odometry (m) or None (lost frame: last value kept). Returns True while inside the scope."""
        if window_curve_end is not None and window_curve_end != self.started_ref:
            self.reset(); self.active = True; self.end_ref = float(window_curve_end); self.started_ref = window_curve_end; self.t0 = t
            self.mode = 'REF'
        if not self.active: return False
        if dist is not None:
            self.dist_after = float(dist) - self.end_ref
            if self.dist_after >= float(self.c['end_m']):
                self.active = False; self.mode = None; return False
        return True

    def _ref(self, t, yaw):
        if self.psi0 is not None: return True
        if yaw is not None and t - self.t0 <= float(self.c['ref_s']) + 1e-9: self.ref.append(float(yaw))
        if t - self.t0 >= float(self.c['ref_s']) - 1e-9 and self.ref:
            r0 = self.ref[0]; self.psi0 = wrap(r0 + _median([wrap(y - r0) for y in self.ref])); return True
        return False

    def side(self, dev):
        """(soft, hard, exit) of the side dev points to; None = that side off."""
        c = self.c
        if dev > 0: return float(c['soft_deg']), float(c['hard_deg']), float(c['exit_deg'])
        if dev < 0 and c.get('soft_right_deg') is not None and c.get('hard_right_deg') is not None:
            ex = c.get('exit_right_deg'); ex = float(c['exit_deg']) if ex is None else float(ex)
            return float(c['soft_right_deg']), float(c['hard_right_deg']), min(ex, float(c['soft_right_deg']))
        return None

    def _mode(self, dev):
        sd = self.side(dev)
        if sd is None: self.hard_sign = 0; return 'IN'
        soft, hard, ex = sd; a = abs(dev); sg = 1 if dev > 0 else -1
        if self.mode == 'HARD' and getattr(self, 'hard_sign', 0) == sg and a >= ex: return 'HARD'
        if a >= hard: self.hard_sign = sg; return 'HARD'
        self.hard_sign = 0
        if a > soft: return 'SOFT'
        return 'IN'

    def _side(self, dev, out, c0):
        return max(out, c0) if dev > 0 else (min(out, c0) if dev < 0 else out)

    def hard_cmd(self, dev, gz, c0, lo, hi):
        lim = float(self.c['lim_us'])
        term = float(self.c['k_hold_us_per_deg']) * dev + (float(self.c['k_gz_us_per_dps']) * float(gz) if gz is not None else 0.)
        out = c0 + max(-lim, min(lim, term))
        return max(lo, min(hi, self._side(dev, out, c0)))

    # ---- per frame ---------------------------------------------------------------------------------------------------------
    def step(self, t, yaw, gz, steer_us, c0, lo, hi, valid=True):
        """(steer_us, info) for one frame inside the scope (call scope() first). info None = nothing to log."""
        if not self.active: return steer_us, None
        if yaw is None:
            return steer_us, dict(revision=REVISION, mode=self.mode, reason='NO_YAW', from_us=round(float(steer_us), 1))
        if not self._ref(t, yaw):
            return steer_us, dict(revision=REVISION, mode='REF', from_us=round(float(steer_us), 1), steer_us=round(float(steer_us), 1),
                                  dist_after_m=None if self.dist_after is None else round(self.dist_after, 2))
        dev = wrap(float(yaw) - self.psi0); self.mode = m = self._mode(dev)
        s = float(steer_us); out = s
        if m == 'SOFT' and valid:
            ex = abs(dev) - self.side(dev)[0]; sg = 1. if dev > 0 else -1.
            out = s + float(self.c['k_us_per_deg']) * ex * sg
            fl = c0 + float(self.c['k_soft_floor_us_per_deg']) * ex * sg   # r2 floor toward psi0
            out = max(lo, min(hi, max(out, fl) if dev > 0 else min(out, fl)))
        elif m == 'HARD' or (m == 'SOFT' and not valid and self.c.get('lost_hold', True)):
            out = self.hard_cmd(dev, gz, c0, lo, hi)
        info = dict(revision=REVISION, mode=m, dev=round(dev, 2), psi0=round(self.psi0, 2), from_us=round(s, 1), steer_us=round(out, 1),
                    gz=None if gz is None else round(float(gz), 1), dist_after_m=None if self.dist_after is None else round(self.dist_after, 2))
        if not valid: info['lost'] = True
        return out, info
