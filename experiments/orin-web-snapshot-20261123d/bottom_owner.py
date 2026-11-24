"""bottom-owner-r2 (integration v2, 2026-11-13; r1 kept in r1_archive/): ONE steering owner for the slope bottom, from the landing to curve 4's FLOOR.

Problem (2026-11-13, ~20 bottom passes, 5 failures f7be / 9ba3 / f3a0 lap 2 / 8e39 lap 5 / b3be): at the landing the camera view
changes, the image error jumps to +0.6..+1.0 in 2-3 frames, the PID alone commands 1707-1961 us and the car yaws right at -43..-74
deg/s; then downhill_bottom (JUMP / HOLD) keeps the hard-right command 0.3-0.4 s and, on lost frames, bottom_map (MAP_TURN) turns
right by gyro at 1800-1900 us. In the failures the car has turned 40-57 deg (b3be ~80) right before curve 4 starts.

Scope (live_pid decides the inputs, this module only does arithmetic; no clock, IMU or file access except load_config()):
  starts at the landing   = live_pid `down_level_t0` (first frame whose dash-route pitch (after flat_dip_guard) is >= DOWN_STEEP -8
                            after the slope was below it; the same moment bottom-map-r1 latches its landing yaw `_bm_yaw0`);
  ends at                 curve_map seg after_down idx 0 reason FLOOR (curve owner takes over), or t_max_s after the landing,
                          or the pitch steep again (live_pid resets down_level_t0 to None -> a new landing = a new scope).
Inside the scope:
  valid frames : right cap  centre + cap_us for limit_s after the landing, then the cap opens linearly to the servo limit over ramp_s.
                 r2 re-arm: after >= rearm_lost_frames consecutive lost frames inside the scope, the cap timer restarts at the first
                 valid frame that is NOT a WARMUP frame (9ba3: lost 1.25 s after the landing, spike at 1.33-1.43 s). WARMUP frames are
                 limited by the running timer but never start the timer, never consume a pending re-arm and do not end a lost streak.
                 Left guard centre - left_cap_us for the whole scope (r2: default null = off; the counter-swing after the landing spike
                 is normal in 10 of 14 ok passes). r1's limit_from key is removed (ignored if present): the re-arm replaces it.
  lost frames  : gyro rate hold (r2 defaults: turn_limit_deg 55 - ok passes turn 37-62 deg right before FLOOR; k 4)  target = target_dps while turned_left > -turn_limit_deg, else 0 (hold heading);
                 command = centre + ff + k_us_per_dps * (gz - target), offset limited to +-lim_us (ff = ff_us for target_dps,
                 0 for target 0); then the left guard (if set). For at most lost_max_s of one lost streak; after that None (= the
                 existing behaviour of live_pid).
  live_pid skips downhill_bottom JUMP / HOLD and the bottom_map MAP_TURN modifier while the scope is active.
Conventions: servo us, larger = right; gyro yaw / gz left positive (deg, deg/s); turned_left = yaw now - yaw at the landing.
bottom_owner.json is read once per run (LivePID.start); enabled false (default) -> live_pid never calls this module after the
construction. Experiment values, not verified on the car. Python 3 standard library only.
"""
import json
import math
from pathlib import Path
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'bottom-owner-r2'
CONFIG_FILE = Path(__file__).with_name('bottom_owner.json')
DEFAULTS = dict(enabled=False, cap_us=300., limit_s=.6, ramp_s=.6, left_cap_us=None, rearm_lost_frames=3.,
                target_dps=-20., turn_limit_deg=55., ff_us=150., k_us_per_dps=4., lim_us=250., lost_max_s=2., t_max_s=3.8)
_NUM = ('cap_us', 'limit_s', 'ramp_s', 'rearm_lost_frames', 'target_dps', 'turn_limit_deg', 'ff_us', 'k_us_per_dps', 'lim_us', 'lost_max_s', 't_max_s')
_OPT = ('left_cap_us',)   # number or null (null = feature off)


def load_config(path=None):
    """DEFAULTS updated by bottom_owner.json; any error (missing file, bad JSON, non-numeric value) -> DEFAULTS (= disabled)."""
    c = dict(DEFAULTS)
    try:
        f = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        u = dict(c)
        for k in _NUM:
            if k in f and f[k] is not None:
                v = float(f[k])
                if not math.isfinite(v): raise ValueError(k)
                u[k] = v
        for k in _OPT:
            if k in f:
                if f[k] is None: u[k] = None
                else:
                    v = float(f[k])
                    if not math.isfinite(v): raise ValueError(k)
                    u[k] = v
        u['enabled'] = f.get('enabled') is True
        c = u
    except Exception:
        c = dict(DEFAULTS)
    return c


class BottomOwner(object):
    def __init__(self, config=None):
        self.c = dict(DEFAULTS) if config is None else dict(DEFAULTS, **config)
        self.on = bool(self.c.get('enabled'))
        self.t_land = None; self.ended = None; self.t_cap0 = None; self.lost_t0 = None; self.lost_n = 0; self._lost_now = None; self.rearms = 0; self._act = False

    @classmethod
    def from_file(cls, path=None):
        try: return cls(load_config(path))
        except Exception: return cls(dict(DEFAULTS))

    # ---- scope -------------------------------------------------------------------------------------------------------------
    def scope(self, now, t_land, floor=False):
        """Active flag for this moment. t_land = live_pid down_level_t0 (None = no landing / steep again); floor = this frame's
        curve_map says seg after_down idx 0 reason FLOOR. A new t_land starts a new scope; FLOOR / t_max end it for that landing."""
        if not self.on: return False
        if t_land is None:
            if self.t_land is not None and self.ended is None: self.ended = 'STEEP'
            self.t_land = None; self._act = False; return False
        if t_land != self.t_land:
            self.t_land = t_land; self.ended = None; self.t_cap0 = t_land; self.lost_t0 = None; self.lost_n = 0; self._lost_now = None; self.rearms = 0
        if self.ended is None:
            if floor: self.ended = 'FLOOR'
            elif now - t_land > float(self.c['t_max_s']): self.ended = 'T_MAX'
        self._act = self.ended is None
        return self._act

    @property
    def active(self):
        return self._act

    # ---- valid frames ------------------------------------------------------------------------------------------------------
    def left_floor(self, c0, lo):
        lc = self.c.get('left_cap_us')
        return None if lc is None else max(float(lo), float(c0) - float(lc))

    def right_cap(self, now, c0, hi):
        t0 = self.t_cap0 if self.t_cap0 is not None else self.t_land
        if t0 is None: return float(hi), 'OPEN'
        dt = now - t0; cap = float(c0) + float(self.c['cap_us']); ls = float(self.c['limit_s']); rs = float(self.c['ramp_s'])
        if dt <= ls: return min(float(hi), cap), 'CAP'
        if rs > 0. and dt < ls + rs: return min(float(hi), cap + (float(hi) - cap) * (dt - ls) / rs), 'RAMP'
        return float(hi), 'OPEN'

    def limit(self, now, steer, c0, lo, hi, warmup=False):
        """valid frame inside the scope: (command, info). info['cap'] names the bound that acted (None = unchanged).
        warmup=True (controller WARMUP frame): limited by the running timer, but it neither re-arms nor ends the lost streak."""
        rearmed = False
        if not warmup:
            if self.lost_n >= _RI.thr(float(self.c['rearm_lost_frames'])): self.t_cap0 = now; self.rearms += 1; rearmed = True
            self.lost_t0 = None; self.lost_n = 0; self._lost_now = None
        s = float(steer); rc, phase = self.right_cap(now, c0, hi); lc = self.left_floor(c0, lo)
        out = s; which = None
        if s > rc: out = rc; which = 'RIGHT_' + phase
        elif lc is not None and s < lc: out = lc; which = 'LEFT_GUARD'
        return out, dict(revision=REVISION, mode='LIMIT', from_us=round(s, 1), steer_us=round(out, 1), cap=which, phase=phase,
                         right_cap_us=round(rc, 1), left_cap_us=None if lc is None else round(lc, 1), warmup=bool(warmup), rearmed=rearmed,
                         rearms=self.rearms, timer_s=round(now - self.t_cap0, 3) if self.t_cap0 is not None else None,
                         since_s=round(now - self.t_land, 3) if self.t_land is not None else None)

    # ---- lost frames -------------------------------------------------------------------------------------------------------
    def lost_due(self, now):
        """True while the current lost streak (first lost frame inside the scope; a valid frame inside the scope ends it) is at most
        lost_max_s old. Called by live_pid before MAP_TURN (skip) and by lost()."""
        if self.lost_t0 is None: self.lost_t0 = now
        if self._lost_now != now: self._lost_now = now; self.lost_n += _RI.w()   # r2: one count per lost frame (live_pid may call it twice per frame)
        return now - self.lost_t0 <= float(self.c['lost_max_s'])

    def lost(self, now, gz, turned_left, c0, lo, hi):
        """lost frame inside the scope: (command, info) of the gyro rate hold, or None (lost streak longer than lost_max_s, or no gz)
        -> the caller keeps its existing behaviour."""
        if not self.lost_due(now) or gz is None: return None
        ls = now - self.lost_t0
        gz = float(gz)
        if not math.isfinite(gz): return None
        turning = turned_left is None or float(turned_left) > -float(self.c['turn_limit_deg'])
        tgt = float(self.c['target_dps']) if turning else 0.
        ff = float(self.c['ff_us']) if turning else 0.
        lim = float(self.c['lim_us'])
        u = max(-lim, min(lim, ff + float(self.c['k_us_per_dps']) * (gz - tgt)))
        out = float(c0) + u; lc = self.left_floor(c0, lo); which = None
        if lc is not None and out < lc: out = lc; which = 'LEFT_GUARD'
        out = max(float(lo), min(float(hi), out))
        return out, dict(revision=REVISION, mode='RATE_HOLD', target_dps=tgt, gz=round(gz, 1),
                         turned_left_deg=None if turned_left is None else round(float(turned_left), 1), ff_us=ff, offset_us=round(u, 1),
                         steer_us=round(out, 1), cap=which, lost_s=round(ls, 3), lost_n=self.lost_n, since_s=round(now - self.t_land, 3) if self.t_land is not None else None)
