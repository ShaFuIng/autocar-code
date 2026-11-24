"""lap-speed-r1 (lap65-20261115 offline candidate, NOT deployed): position-based PWM floor from lap_pos.

Problem (lap65-20261115 analysis/lap_budget.txt, wait_need.txt): on the flat straights the PWM is below 50 for ~11.7 s per lap;
about half of that time (WAIT_STABLE 0.5 s + RAMP_UP 30/s, MID ERROR / ERROR_RATE) is followed by a calm second (|e| < 0.30,
|gz| < 15 deg/s), and the straight |image error| at PWM 50 does not grow with speed between 0.9 and 1.3 m/s.

What it does: on a valid flat-lap frame whose lap_pos position lies inside a configured zone (metres after the last lap_pos
crossing of an anchor landmark of the current lap), the PWM chosen by speed_adapt is raised towards the zone's PWM, at most
rate_pwm_s per second, and speed_adapt's internal PWM follows (so gain_schedule and the ramp after the zone see the raised value).
It never lowers a PWM. The caps that come after it in speed_policy.caps (upper-section caps, brake zones, turn PWM, speed guard)
still apply unchanged, so a brake zone or the crossing cap 45 still wins.

It does NOT raise (frame left exactly as speed_adapt made it) when any of these holds:
  lap_pos missing / disabled / lost; the anchor landmark of the zone not crossed in this lap or farther than to_m; reason not
  PROPOSAL_ONLY (warm-up, hold, lost frames never come here anyway); speed_adapt reason SPECIAL_CONTROL / DOWNHILL / AFTER_CREST /
  NO_ERROR / DISABLED / ZERO_BASE / error; speed_adapt info with upper_pwm (upper section), landing, crest_drop or downhill_entry;
  speed_adapt level LOW (any reason: ERROR_LARGE / CURVE_AHEAD / STEERING_LARGE / ERROR_RATE_LARGE keep their reduction), except in a
  zone with allow_low true, where a path-caused LOW (not weave_slow) may be raised; route not WAIT_CLIMB; |pitch| > level_deg;
  |image error| > err_guard (zone or global); |steer - centre| > steer_guard_us. What it removes: WAIT_STABLE, the up_rate ramp,
  the MID levels (ERROR / STEERING / ERROR_RATE), STABLE_PENDING - i.e. the waiting, not the reductions.
Config lap_speed.json, read once per run (first call of a run). File absent / enabled false / no zones = no effect, nothing logged.

lap-speed-r3 (straightfast-20261121, offline candidate, NOT deployed; both keys per zone, absent = r1 / r2 exactly):
  rate_pwm_s   per-zone ramp rate (PWM/s) instead of the global rate_pwm_s.
  two_level    {"e_trouble": 0.40, "e_ok": 0.30, "steer_trouble_us": 300, "gz_trouble_dps": 25, "gz_tau_s": 0.25,
                "mid_pwm": 70, "rate_up_pwm_s": 200, "rate_down_pwm_s": 120}
               Why (11-21, runs e1b4 / b93e / 9934, 30 zone passes): 36 of 41 slow-downs inside the PWM-100 zones came from one or
               two frames of speed_adapt LOW / ERROR_RATE_LARGE (camera error rate; |e| 0.13 and IMU |gz| ~3 deg/s at that moment),
               each one dropped the PWM to speed_adapt's <= 50 and restarted the 60 PWM/s ramp (0.8 s back to 100).
               In a two_level zone the camera error RATE and speed_adapt's LOW / MID levels are not used. Two states only:
                 NORMAL  -> the zone PWM;   TROUBLE -> min(zone PWM, mid_pwm)
               TROUBLE when |image error| > e_trouble (left again below e_ok), or |steer - centre| > steer_trouble_us, or the
               IMU yaw rate low-passed over gz_tau_s is above gz_trouble_dps (null = not used; no IMU value = not used).
               The PWM moves at rate_up_pwm_s / rate_down_pwm_s from the last frame of the same zone, without any wait, and is
               never below the PWM speed_adapt chose. The hard conditions of r1 still leave the frame untouched: reason not
               PROPOSAL_ONLY, speed_adapt SPECIAL_CONTROL / DOWNHILL / AFTER_CREST / NO_ERROR / DISABLED / ZERO_BASE, upper / landing /
               crest / downhill-entry frames, special control, route not WAIT_CLIMB, |pitch| > level_deg, no image error.
               Log: lap_speed.two_level = {state, why, gz_lp}; revision lap-speed-r3 on these frames.
Standard library only."""
import json
from pathlib import Path

REVISION = 'lap-speed-r1'
try:import throttle_cap   # throttle-cap-r1 (speed-next-20261117): web/throttle_cap.json max_pct (absent = 50 = old fixed limits)
except Exception:throttle_cap=None
CONFIG_FILE = Path(__file__).with_name('lap_speed.json')
BLOCK_REASONS = ('SPECIAL_CONTROL', 'DOWNHILL', 'AFTER_CREST', 'NO_ERROR', 'DISABLED', 'ZERO_BASE', 'SPEED_ERROR')
BLOCK_KEYS = ('upper_pwm', 'landing', 'crest_drop', 'downhill_entry')
UPPER_KEYS = ('upper_pwm', 'crest_drop')   # lap-speed-r2: the keys an allow_upper zone may lift
REVISION_R3 = 'lap-speed-r3'   # logged only for frames inside a two_level zone (straightfast-20261121)
REVISION_R2 = 'lap-speed-r2'   # logged only for frames inside a zone with allow_climb / allow_upper
PATH_LOW = ('ERROR_LARGE', 'CURVE_AHEAD', 'STEERING_LARGE', 'ERROR_RATE_LARGE')   # = speed_adapt.PATH_LOW
DEFAULTS = dict(enabled=False, rate_pwm_s=60., err_guard=0.30, steer_guard_us=250., level_deg=5., carry_s=0.5, zones=[])


def load_config(path=None):
    c = dict(DEFAULTS)
    try:
        f = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        if isinstance(f, dict): c.update(f)
    except (OSError, ValueError): return None
    return c


class LapSpeed(object):
    def __init__(self, cfg, run_id=None):
        self.c = cfg or {}; self.run_id = run_id
        z = [x for x in (self.c.get('zones') or []) if isinstance(x, dict) and x.get('anchor') and x.get('pwm') is not None]
        self.zones = z
        self.on = bool(self.c.get('enabled')) and bool(z)
        self.last = None   # (t, pwm) of the last raised frame

    @staticmethod
    def crossing(lap, name):
        """odometer (m) of the last crossing of landmark `name` in the current lap, None if none (skipped / other lap)."""
        for e in reversed(getattr(lap, 'events', None) or []):
            if e.get('name') == name and 'd_cross' in e:
                return float(e['d_cross']) if e.get('lap') == getattr(lap, 'lap', None) else None
        return None

    def zone_at(self, lap):
        """(zone, rel_m) of the first zone that contains the current position, else (None, None)."""
        D = float(getattr(lap, 'D', 0.))
        for z in self.zones:
            dc = self.crossing(lap, z['anchor'])
            if dc is None: continue
            rel = D - dc
            if float(z.get('from_m', 0.)) <= rel <= float(z.get('to_m', 0.)): return z, rel
        return None, None

    def blocked(self, p, F, center, z=None, soft=False):
        """reason string when this frame must not be raised, else None."""
        c = self.c
        if p.get('reason') != 'PROPOSAL_ONLY': return 'REASON'
        sa = p.get('speed_adapt')
        if not isinstance(sa, dict) or sa.get('reason') in BLOCK_REASONS or sa.get('reason') is None: return 'SPEED_ADAPT'
        zz = z or {}; up_ = bool(zz.get('allow_upper')); cl_ = bool(zz.get('allow_climb'))   # lap-speed-r2 (throttle-free-20261118)
        for k in BLOCK_KEYS:
            if sa.get(k):
                if up_ and k in UPPER_KEYS: continue   # lap-speed-r2 allow_upper: the upper-section cap of speed_adapt may be lifted
                return k.upper()
        if not soft and sa.get('level') == 'LOW' and not ((z or {}).get('allow_low') and sa.get('reason') in PATH_LOW and not sa.get('weave_slow')): return 'LOW'   # r3: soft = two_level zone
        if getattr(F, 'special', False): return 'SPECIAL'
        dr = getattr(F, 'dr', None) or {}
        rt = dr.get('route')
        if rt != 'WAIT_CLIMB' and not ((cl_ and rt == 'CLIMBED') or (up_ and rt in ('CLIMBED', 'OPEN'))): return 'ROUTE'   # lap-speed-r2
        pr = getattr(F, 'pr', None)
        if pr is None: return 'PITCH'
        if abs(float(pr)) > float(c.get('level_deg', 5.)) and not (cl_ and float(pr) > 0.): return 'PITCH'   # lap-speed-r2 allow_climb: uphill only
        return None

    def apply(self, lp, F, p, now):
        """raise p['pwm'] when allowed. Returns the log dict (None outside every zone)."""
        lap = getattr(lp, '_lpos', None)
        if lap is None or lap is False or getattr(lap, 'lost', True): return None
        z, rel = self.zone_at(lap)
        if z is None: return None
        log = dict(revision=REVISION_R2 if (z.get('allow_climb') or z.get('allow_upper')) else REVISION, zone=z.get('name'), rel_m=round(rel, 2), target=float(z['pwm']))
        center = float(lp.run['parameters']['center_us'])
        tl = z.get('two_level')   # lap-speed-r3: absent / not a dict / enabled false -> the r1 / r2 path below, unchanged
        if isinstance(tl, dict) and tl.get('enabled', True) is not False: return self._two_level(lp, F, p, now, z, log, center, tl)
        why = self.blocked(p, F, center, z)
        e = p.get('image_error'); st = p.get('steer_us')
        eg = float(z.get('err_guard', self.c.get('err_guard', .30))); sg = float(z.get('steer_guard_us', self.c.get('steer_guard_us', 250.)))
        if why is None and (e is None or abs(float(e)) > eg): why = 'ERROR'
        if why is None and (st is None or abs(float(st) - center) > sg): why = 'STEER'
        cur = float(p.get('pwm') or 0.)
        if why is not None:
            self.last = None; log.update(blocked=why, pwm=round(cur, 2)); return log
        tgt = float(z['pwm'])
        try: tc = float(throttle_cap.max_pct()) if throttle_cap is not None else 50.   # throttle-cap-r1: a zone never above max_pct
        except Exception: tc = 50.
        if tgt > tc: tgt = tc; log['cap_pct'] = tc
        start = cur
        zi = next((i for i, q in enumerate(self.zones) if q is z), None)   # throttle-cap-r1: the carry only continues inside the same zone
        if self.last is not None and now is not None and 0. <= now - self.last[0] <= float(self.c.get('carry_s', .5)) and (len(self.last) < 3 or self.last[2] == zi):
            start = max(cur, self.last[1]); dt = now - self.last[0]
        else: dt = 0.
        dn = self.c.get('down_rate_pwm_s')   # lap-speed taper (field-20261120): key absent -> every frame unchanged
        if dn is not None and self.last is not None and now is not None and self.last[1] > tgt and 0. <= now - self.last[0] <= float(self.c.get('carry_s', .5)):
            start = max(cur, tgt, self.last[1] - float(dn) * (now - self.last[0])); log['taper'] = True   # last raised value above this zone's target (came from a higher zone): step down at dn PWM/s instead of dropping at once
        new = min(tgt, start + float(z.get('rate_pwm_s', self.c.get('rate_pwm_s', 60.))) * dt) if start < tgt else start   # r3: per-zone rate (absent = global)
        new = max(cur, new)
        if new > cur:
            p['pwm'] = new
            sp = getattr(lp, 'speed', None)
            nsa = new   # throttle-cap-r1: speed_adapt's own PWM is never raised above its hard_cap (50); above 50 only this zone (carry) holds it
            try: nsa = min(new, float(sp.c['hard_cap'])) if sp is not None else new
            except Exception: pass
            if sp is not None and getattr(sp, 'cur', None) is not None and sp.cur < nsa: sp.cur = nsa   # speed_adapt continues from here
            log['pwm_before'] = round(cur, 2)
        log['pwm'] = round(new, 2)
        self.last = (now, new, zi) if now is not None else None   # (a PWM-60 zone followed by a PWM-50 zone must not carry 60 into it)
        return log


def _tl_apply(self, lp, F, p, now, z, log, center, tl):
    """lap-speed-r3 two_level zone (see the module text). Returns the log dict."""
    import math
    log['revision'] = REVISION_R3
    hard = self.blocked(p, F, center, z, soft=True)
    e = p.get('image_error'); st = p.get('steer_us'); cur = float(p.get('pwm') or 0.)
    if hard is None and (e is None or st is None): hard = 'NO_ERROR'
    gz = getattr(F, 'gzy', None); glp = None   # IMU yaw rate (deg/s) of this frame, as yaw_inner / curve_map get it
    if isinstance(gz, (int, float)) and not isinstance(gz, bool) and now is not None:
        s_ = getattr(self, 'tl_gz', None); tau = max(1e-3, float(tl.get('gz_tau_s', .25)))
        y = float(gz) if s_ is None or not (0. <= now - s_[0] <= .5) else s_[1] + (float(gz) - s_[1]) * (1. - math.exp(-(now - s_[0]) / tau))
        self.tl_gz = (now, y); glp = abs(y)
    else: self.tl_gz = None
    if hard is not None:
        self.last = None; self.tl_trouble = False; log.update(blocked=hard, pwm=round(cur, 2)); return log
    et = float(tl.get('e_trouble', .40)); eo = min(et, float(tl.get('e_ok', et))); gt = tl.get('gz_trouble_dps'); sgt = float(tl.get('steer_trouble_us', 300.))
    zi = next((i for i, q in enumerate(self.zones) if q is z), None)
    was = bool(getattr(self, 'tl_trouble', False)) and self.last is not None and len(self.last) >= 3 and self.last[2] == zi
    ae = abs(float(e)); why = None
    if ae > et or (was and ae > eo): why = 'ERROR'
    elif abs(float(st) - center) > sgt: why = 'STEER'
    elif gt is not None and glp is not None and glp > float(gt): why = 'YAW'
    self.tl_trouble = why is not None
    tgt = float(z['pwm'])
    try: tc = float(throttle_cap.max_pct()) if throttle_cap is not None else 50.
    except Exception: tc = 50.
    if tgt > tc: tgt = tc; log['cap_pct'] = tc
    if why is not None: tgt = min(tgt, float(tl.get('mid_pwm', 70.)))
    carry = self.last is not None and now is not None and 0. <= now - self.last[0] <= float(self.c.get('carry_s', .5))
    if carry:
        dt = now - self.last[0]; prev = max(cur, self.last[1]); same = len(self.last) >= 3 and self.last[2] == zi
        if prev < tgt: new = min(tgt, prev + float(tl.get('rate_up_pwm_s', 200.)) * dt)   # also from a lower zone just left (no drop back to speed_adapt's PWM at the hand-over)
        elif same: new = max(tgt, prev - float(tl.get('rate_down_pwm_s', 120.)) * dt)
        elif self.c.get('down_rate_pwm_s') is not None: new = max(cur, tgt, prev - float(self.c['down_rate_pwm_s']) * dt); log['taper'] = True   # came from a higher zone: the existing taper
        else: new = cur
    else: new = cur   # first frame after a gap / a hard block: starts from speed_adapt's PWM (as r1)
    new = max(cur, new)
    if new > cur:
        p['pwm'] = new
        sp = getattr(lp, 'speed', None); nsa = new
        try: nsa = min(new, float(sp.c['hard_cap'])) if sp is not None else new
        except Exception: pass
        if sp is not None and getattr(sp, 'cur', None) is not None and sp.cur < nsa: sp.cur = nsa
        log['pwm_before'] = round(cur, 2)
    log['pwm'] = round(new, 2)
    log['two_level'] = dict(state='TROUBLE' if why else 'NORMAL', why=why, gz_lp=None if glp is None else round(glp, 1))
    self.last = (now, new, zi) if now is not None else None
    return log


LapSpeed._two_level = _tl_apply


def get(lp):
    """this run's LapSpeed, or None when off. Built at the first call of each run (config read once per run)."""
    try:
        rid = (getattr(lp, 'run', None) or {}).get('id')
        g = getattr(lp, '_lap_speed', None)
        if g is None or g.run_id != rid:
            g = LapSpeed(load_config(), rid); lp._lap_speed = g
        return g if g.on else None
    except Exception: return None
