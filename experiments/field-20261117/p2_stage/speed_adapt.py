"""Path-stability speed adaptation for the live PID (speed-adapt-r2).

User 2026-11-01: base speed PWM 20; slow down to 15 where the path is not
stable, speed up to 25 where it is (r1: 15 base, up to 20).

Per valid PID frame:
 - LOW (min_pwm 15): special control (corridor take-over, crest midpoint,
   dashed-line turn / follow, heading hold), downhill (pitch < -level_deg),
   no error, or the path is unsteady: |image error| > ERR_BAD,
   |steering - centre| > STEER_BAD us or |d error / dt| > DERR_BAD per s;
 - HIGH (max_pwm 25): the steady thresholds (|err| <= ERR_MAX,
   |steer dev| <= STEER_DEV_MAX, |derr/dt| <= DERR_MAX, level, normal path)
   held for STABLE_S;
 - otherwise MID (mid_pwm 20). Uphill keeps at least the IMU boost PWM.
The PWM moves to the target at UP_RATE / DOWN_RATE per s. Hold / heading-
blind frames use min_pwm (or the IMU boost PWM when it is higher). Steady
thresholds from live 76a4d787 flat FOLLOW frames (median |err| 0.21,
|derr/dt| 0.16, |steer dev| 166 us); unsteady thresholds near their 90th
percentile (0.51 / 0.41 / 377 us). Experiment values; the PID gains are
unchanged (tuned at PWM 15). max_pwm stays within the visual PID 25 % limit.
"""
# r7 (user 2026-11-04, live 1f5c4da6 / c265c4ca): the downhill ran at 15 PWM at the slope start (route DONE lifted the 13 lane
# cap, then a 0.5 s ramp down to downhill_pwm 10) and again during the 0.4 s PID_HOLD (hold frames used min_pwm 15). The user
# wants 10 on the whole downhill: the downhill target is now taken at once (no ramp), and hold_pwm() gives downhill_pwm to
# hold / blind frames while pitched down or inside live_pid's downhill window.
# r6 (user 2026-11-02, live c6bc3b36): flat back to 15 / 20 / 25 (the 9 cm rear wheels make each PWM ~20 %
# faster; 20 / 25 / 30 swung with a growing amplitude). Uphill keeps 25-30: HIGH on the uphill uses
# uphill_max_pwm (30), the floor is the IMU boost PWM (25).
# r5 (user 2026-11-02): all levels +5 PWM (LOW 20 / MID 25 / HIGH 30, speed_adapt.json); uphill 25-30: on the
# uphill the normal stability levels apply with MID (25) as the floor (HIGH 30 when steady); downhill stays at
# downhill_pwm (12). Experiment values.
# r4 (user 2026-11-02, live 41502243): the downhill is a big right curve; gravity speeds the car up at PWM 15
# and it under-steered. While pitched downhill the target is downhill_pwm (below min_pwm and below the
# saved PWM). Experiment value.
# r3: live 64ecef58 swung in an S at PWM 20 after the crest (|err| stayed <= ERR_BAD, so MID), and the
# dashed-row TURN started at the end of that swing. After the climb (route CLIMBED and level again, or
# route window OPEN) until the downhill the target is LOW.
# r8 curve-ahead (user 2026-11-05: slow down earlier before a curve): the far preview row (turn_preview far_projected_error,
# row 0.45) passes far_bad about 0.3 s (median) before the near error passes err_bad (far_lead.json); with far_bad set in
# speed_adapt.json a far error above it is LOW (reason CURVE_AHEAD). Key absent = off. Experiment value.
# r9 upper-25 (user 2026-11-05: upper section at 25, everything else unchanged): with upper_pwm set in speed_adapt.json, in the
# upper section (route OPEN, or CLIMBED and level = where r3 gave AFTER_CREST) neither AFTER_CREST nor SPECIAL_CONTROL forces LOW;
# the normal stability levels apply (unsteady -> LOW 15, curve ahead 15, ...) and the target is at most upper_pwm. live_pid keeps
# the dashed-turn cap (11) and lock-slow (13); lane_segment.LOCK_PWM (upper cap) is 25. Key absent = r8. Experiment value.
# r10 downhill entry push (user 2026-11-05): see downhill-push-20261105/make_candidate.py. With downhill_entry_pwm in
# speed_adapt.json the first part of each downhill (until over the slope edge) uses downhill_entry_pwm, then downhill_pwm.
import json
from pathlib import Path
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

# r11 (user 2026-11-05): crest drop (crest_drop_deg) and the downhill push ends when the wheels roll (downhill_entry_roll_cps);
# see crest-roll-20261105/make_candidate.py.
# r14 curve-pwm (user 2026-11-06, live 81541065: curve-ff floor brings the car back from the outside at PWM 15): with curve_pwm set in speed_adapt.json, the flat-lap LOW frames caused by the path itself (ERROR_LARGE / CURVE_AHEAD / STEERING_LARGE / ERROR_RATE_LARGE) use max(min_pwm, curve_pwm) instead of min_pwm. Hold / blind frames (safe_pwm), SPECIAL_CONTROL, DOWNHILL, AFTER_CREST, NO_ERROR, the upper section (up_pwm), uphill and the MID/HIGH levels are unchanged. Key absent = off.
# r18 speed-plan-r1 (2026-11-14 offline candidate): curve_pwm_by (per-curve PWM), xing_cap_pwm (cap in the crossing window),
# derr_confirm_n (single-frame error-rate jumps on the flat lap do not end the stable run). Keys absent = r17 behaviour.
# REVISION stays 'speed-adapt-r17' while all three keys are absent (logs identical); with any key set it reads 'speed-adapt-r18'.
REVISION = 'speed-adapt-r17'
REVISION_R18 = 'speed-adapt-r18'
PATH_LOW = ('ERROR_LARGE', 'CURVE_AHEAD', 'STEERING_LARGE', 'ERROR_RATE_LARGE')
CONFIG_FILE = Path(__file__).with_name('speed_adapt.json')
DEFAULTS = dict(enabled=True, min_pwm=15., mid_pwm=20., max_pwm=25., up_rate=2., down_rate=10., stable_s=1.,
                err_max=.25, derr_max=.8, steer_dev_max=200., err_bad=.45, derr_bad=1.2, steer_bad=350.,
                level_deg=3., hard_cap=25., downhill_pwm=12., uphill_max_pwm=None)


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    cap = float(c['hard_cap'])
    for k in ('min_pwm', 'mid_pwm', 'max_pwm'): c[k] = min(float(c[k]), cap)
    return c


class SpeedAdapt:
    def __init__(self, config=None):
        self.c = dict(config) if config is not None else load_config()
        self.cur = None; self.t = None; self.e = None; self.e_t = None; self.since = None; self.base = None
        self.dh = None   # r10 downhill episode: dict(t0, last, steep_t, min_pitch, done)
        self.wheel = None   # r11 left wheel counts/s from live_pid (None = unknown)
        self.down_t0 = None; self.down_last = None   # landing-pwm-r1: current / last downhill episode (monotonic s)
        self.dcnt = 0   # r18: consecutive error-rate-only frames

    def _mark_down(self, now):
        """landing-pwm-r1: a downhill frame (valid or hold); a gap > 1 s starts a new episode."""
        if self.down_last is None or now - self.down_last > 1.: self.down_t0 = now
        self.down_last = now

    def _landing(self, now):
        """landing-pwm-r1 (user 2026-11-14 a747: after the slope the PWM restarted at the saved 15 and ramped 15/s to min_pwm 21
        under ERROR_LARGE, the car fell to 0.26-0.31 m/s before curve 4). Within landing_s after the last downhill frame of an
        episode that lasted >= landing_min_down_s: dict(pwm, since_s), else None. speed_adapt.json landing_s absent / null = off."""
        c = self.c; ls = c.get('landing_s')
        if ls is None or self.down_last is None or self.down_t0 is None: return None
        if not 0. <= now - self.down_last <= float(ls): return None
        if self.down_last - self.down_t0 < float(c.get('landing_min_down_s', 1.5)): return None
        lp = c.get('landing_pwm')
        if lp is None: lp = c.get('curve_pwm')
        if lp is None: lp = c['min_pwm']
        return dict(pwm=min(float(lp), float(c['hard_cap'])), since_s=round(now - self.down_last, 2))

    def _downhill_pwm(self, now, pitch_rel):
        """r10: downhill_pwm, or downhill_entry_pwm while the car is not yet over the slope edge. Returns (pwm, info)."""
        c = self.c; dp = float(c['downhill_pwm']); ep = c.get('downhill_entry_pwm')
        if ep is None: return dp, None
        d = self.dh
        if d is None or now - d['last'] > float(c.get('downhill_entry_reset_s', 1.)):
            d = self.dh = dict(t0=now, last=now, steep_t=None, min_pitch=None, done=False)
        d['last'] = now
        if pitch_rel is not None:
            if d['min_pitch'] is None or pitch_rel < d['min_pitch']: d['min_pitch'] = float(pitch_rel)
            if d['steep_t'] is None and pitch_rel <= float(c.get('downhill_entry_pitch_deg', -14.)): d['steep_t'] = now
        if not d['done']:
            rc = c.get('downhill_entry_roll_cps')   # r11: with a wheel speed the push ends only when rolling (no time fallback)
            if rc is not None and self.wheel is not None:
                over = d['steep_t'] is not None and self.wheel >= float(rc)
                if over: d['why'] = 'ROLLING'
            else:
                over = d['steep_t'] is not None and (now - d['steep_t'] >= float(c.get('downhill_entry_after_s', .6))
                       or (pitch_rel is not None and pitch_rel >= d['min_pitch'] + float(c.get('downhill_entry_rise_deg', 1.5))))
            if over or now - d['t0'] >= float(c.get('downhill_entry_max_s', 3.)): d['done'] = True
        info = dict(entry=not d['done'], since_s=round(now - d['t0'], 2), steep=d['steep_t'] is not None, why=d.get('why'),
                    wheel_cps=None if self.wheel is None else round(self.wheel),
                    min_pitch=None if d['min_pitch'] is None else round(d['min_pitch'], 2))
        return (dp if d['done'] else float(ep)), info

    def ceiling(self, base_ceiling):
        up = min(float(self.c.get('uphill_max_pwm') or 0), float(self.c['hard_cap']))
        return int(max(base_ceiling, round(self.c['max_pwm']), round(self.c['mid_pwm']), round(up))) if self.c.get('enabled') else int(base_ceiling)

    def _ri_push(self, now, err):   # rate-indep-r1: recent (t, error) samples for the error-rate baseline
        h = [x for x in getattr(self, '_ri_eh', []) if now - x[0] <= 1.]; h.append((now, err)); self._ri_eh = h

    def _ri_derr(self, now, err, derr):
        """rate-indep-r1: error rate over the newest earlier sample at least DT0/2 old (at ~10 frames/s that is the previous sample,
        i.e. today's value; at ~20 frames/s it spans two frames, so single-frame noise is not doubled by the shorter interval)."""
        for t_, e_ in reversed(getattr(self, '_ri_eh', [])):
            if now - t_ >= .5 * _RI.dt0(): return abs(err - e_) / (now - t_)
        return derr

    def step(self, now, base, err, steer, center, special, pitch_rel, route=None, far=None, wheel=None, pre_turn=False, post_turn=False, weave_slow=None, curve_key=None, xp=False):
        """PWM for a valid frame; base is the PWM after the IMU boost (saved PWM on level ground)."""
        r18 = any(self.c.get(k_) is not None for k_ in ('curve_pwm_by', 'xing_cap_pwm', 'derr_confirm_n', 'uphill_stable_s', 'uphill_up_rate', 'uphill_low_pwm'))
        info = dict(revision=REVISION_R18 if r18 else REVISION, base_pwm=base)
        self.base = base
        if wheel is not None: self.wheel = float(wheel)   # r11
        if not self.c.get('enabled') or not base:
            self.cur = base; info.update(reason='DISABLED' if not self.c.get('enabled') else 'ZERO_BASE', pwm=base); return base, info
        c = self.c
        derr = None
        if err is not None and self.e is not None and self.e_t is not None and now > self.e_t: derr = abs(err - self.e) / (now - self.e_t)
        if _RI.ON and derr is not None: derr = self._ri_derr(now, err, derr)   # rate-indep-r1
        if err is not None: self.e, self.e_t = err, now
        if _RI.ON and err is not None: self._ri_push(now, err)   # rate-indep-r1
        dev = abs(steer - center)
        upper = route == 'OPEN' or (route == 'CLIMBED' and pitch_rel is not None and pitch_rel < c['level_deg'])
        up_pwm = c.get('upper_pwm') if upper else None   # r9
        if upper and pre_turn and route == 'OPEN' and c.get('upper_pwm_pre_turn') is not None:   # r12 upper-pre-turn: crest -> dashed turn only
            up_pwm = max(float(up_pwm or 0), float(c['upper_pwm_pre_turn'])); info['pre_turn'] = True
        if upper and post_turn and route == 'OPEN' and c.get('upper_pwm_post_turn') is not None:   # r13 upper-post-turn: after the dashed turn
            up_pwm = max(float(up_pwm or 0), float(c['upper_pwm_post_turn'])); info['post_turn'] = True
        if up_pwm is not None: info['upper_pwm'] = float(up_pwm); info['special'] = bool(special)
        if special and up_pwm is None: level, why = 'LOW', 'SPECIAL_CONTROL'
        elif pitch_rel is not None and pitch_rel < -c['level_deg']: level, why = 'LOW', 'DOWNHILL'
        elif upper and up_pwm is None: level, why = 'LOW', 'AFTER_CREST'
        elif err is None: level, why = 'LOW', 'NO_ERROR'
        elif abs(err) > c['err_bad']: level, why = 'LOW', 'ERROR_LARGE'
        elif c.get('far_bad') is not None and far is not None and abs(far) > float(c['far_bad']): level, why = 'LOW', 'CURVE_AHEAD'   # r8
        elif dev > c['steer_bad']: level, why = 'LOW', 'STEERING_LARGE'
        elif derr is not None and derr > c['derr_bad']: level, why = 'LOW', 'ERROR_RATE_LARGE'
        elif abs(err) > c['err_max']: level, why = 'MID', 'ERROR'
        elif dev > c['steer_dev_max']: level, why = 'MID', 'STEERING'
        elif derr is not None and derr > c['derr_max']: level, why = 'MID', 'ERROR_RATE'
        else: level, why = 'STEADY', None
        if pitch_rel is not None and pitch_rel > c['level_deg'] and level == 'LOW' and why != 'SPECIAL_CONTROL': level, why = 'MID', 'UPHILL'   # r5: uphill floor MID
        if c.get('derr_confirm_n') is not None:   # r18: an error-rate-only trigger must repeat derr_confirm_n frames on the flat lap
            d_only = (why in ('ERROR_RATE', 'ERROR_RATE_LARGE') and route == 'WAIT_CLIMB' and not xp and not special and up_pwm is None
                      and pitch_rel is not None and abs(float(pitch_rel)) <= c['level_deg'])
            self.dcnt = self.dcnt + _RI.w() if d_only else 0
            if d_only and self.dcnt < _RI.thr(int(c['derr_confirm_n'])):   # not confirmed yet: the rest of the chain without the rate test
                info['derr_unconfirmed'] = dict(n=self.dcnt, why=why)
                if abs(err) > c['err_max']: level, why = 'MID', 'ERROR'
                elif dev > c['steer_dev_max']: level, why = 'MID', 'STEERING'
                else: level, why = 'STEADY', None
        if level == 'STEADY': self.since = now if self.since is None else self.since
        else: self.since = None
        upc = (pitch_rel is not None and pitch_rel > c['level_deg']) or route == 'CLIMBED'   # r18: the climb keeps its own stable_s / up_rate
        ss_ = float(c['uphill_stable_s']) if upc and c.get('uphill_stable_s') is not None else c['stable_s']
        ur_ = float(c['uphill_up_rate']) if upc and c.get('uphill_up_rate') is not None else c['up_rate']
        if level == 'STEADY': level = 'HIGH' if now - self.since >= ss_ else 'MID'
        target = max({'LOW': c['min_pwm'], 'MID': c['mid_pwm'], 'HIGH': c['max_pwm']}[level], base)
        if (level == 'LOW' and why in ('ERROR_LARGE', 'CURVE_AHEAD', 'STEERING_LARGE', 'ERROR_RATE_LARGE') and not special and up_pwm is None
                and c.get('curve_pwm') is not None and pitch_rel is not None and abs(float(pitch_rel)) <= c['level_deg'] and route == 'WAIT_CLIMB'
                and not weave_slow):   # r14; r17: a straight weave keeps the LOW speed (min_pwm / base) instead of curve_pwm
            target = max(target, float(c['curve_pwm'])); info['curve_pwm'] = float(c['curve_pwm'])
        if weave_slow: info['weave_slow'] = True   # r17
        cb = c.get('curve_pwm_by')   # r18 per-curve PWM: replaces curve_pwm (path-caused LOW) and mid_pwm (MID) inside the listed curves
        if (cb and curve_key is not None and cb.get(curve_key) is not None and not special and up_pwm is None and route == 'WAIT_CLIMB'
                and pitch_rel is not None and abs(float(pitch_rel)) <= c['level_deg']
                and (level == 'MID' or (level == 'LOW' and why in PATH_LOW and not weave_slow))):
            target = max(float(cb[curve_key]), base); info['curve_pwm_by'] = dict(curve=curve_key, pwm=float(cb[curve_key]))
        if level == 'HIGH' and pitch_rel is not None and pitch_rel > c['level_deg'] and c.get('uphill_max_pwm'): target = max(target, min(float(c['uphill_max_pwm']), float(c['hard_cap'])))   # r6
        ulp = c.get('uphill_low_pwm')   # uphill-low-r1: LOW / MID frames on the slope get at least this PWM (HIGH already has max_pwm)
        if (ulp is not None and pitch_rel is not None and pitch_rel > c['level_deg'] and level in ('LOW', 'MID') and why != 'SPECIAL_CONTROL'
                and not special and up_pwm is None):
            target = max(target, min(float(ulp), float(c['hard_cap']))); info['uphill_low_pwm'] = float(ulp)
        if up_pwm is not None: target = min(target, float(up_pwm))   # r9
        xc = c.get('xing_cap_pwm') if xp else None   # r18 crossing-window cap
        if xc is not None: target = min(target, float(xc)); info['xing_cap'] = float(xc)
        down = why == 'DOWNHILL' and c.get('downhill_pwm') is not None
        if down:
            target, dei = self._downhill_pwm(now, pitch_rel)   # r4; r10 entry push
            if c.get('downhill_level_pwm') is not None and pitch_rel is not None and pitch_rel > -float(c['level_deg']): target = max(target, float(c['downhill_level_pwm']))   # r15: level bottom
            if dei is not None: info['downhill_entry'] = dei
            self._mark_down(now)   # landing-pwm-r1
        land = None if down else self._landing(now)   # landing-pwm-r1: path-caused LOW on the level bottom uses landing_pwm, and the PWM jumps (no up_rate ramp)
        if (land is not None and level == 'LOW' and why in ('ERROR_LARGE', 'CURVE_AHEAD', 'STEERING_LARGE', 'ERROR_RATE_LARGE', 'NO_ERROR')
                and not special and up_pwm is None and pitch_rel is not None and abs(float(pitch_rel)) <= c['level_deg']):
            target = max(target, land['pwm'])
        if land is not None: info['landing'] = dict(land, target=target)
        cur = max(c['min_pwm'], base) if self.cur is None else self.cur
        dt = 0. if self.t is None else max(0., min(.5, now - self.t)); self.t = now
        cd = c.get('crest_drop_deg')   # r11: at the crest the upper-section PWM at once, below the IMU boost base
        crest = (not down and cd is not None and c.get('upper_pwm') is not None and route == 'CLIMBED' and pitch_rel is not None
                 and pitch_rel < float(cd))
        if crest: target = min(target, float(c['upper_pwm'])); info['crest_drop'] = True
        if down or crest: cur = target   # r7: downhill PWM at once; r11 crest
        elif land is not None and target > cur: cur = target   # landing-pwm-r1: no ramp from the downhill PWM
        elif target > cur: cur = min(target, cur + ur_ * dt)
        else: cur = max(target, cur - c['down_rate'] * dt)
        if not down and not crest: cur = max(cur, base)
        if xc is not None and cur > float(xc): cur = float(xc)   # r18: the cap applies at once (like a brake zone)
        self.cur = cur
        info.update(reason=why or ('STABLE' if level == 'HIGH' else 'STABLE_PENDING'), level=level,
                    stable_s=None if self.since is None else round(now - self.since, 2),
                    derr=None if derr is None else round(derr, 3), target_pwm=target, pwm=round(cur, 2))
        return cur, info

    def hold_pwm(self, held_pwm, pitch_rel, downhill):
        """r7: hold / blind frames on the downhill get downhill_pwm; elsewhere safe_pwm()."""
        dp = self.c.get('downhill_pwm')
        if self.c.get('enabled') and dp is not None and (downhill or (pitch_rel is not None and pitch_rel < -self.c['level_deg'])):
            import time as _t
            dp, _ = self._downhill_pwm(_t.monotonic(), pitch_rel)   # r10: the entry push also on hold frames
            if self.c.get('downhill_level_pwm') is not None and pitch_rel is not None and pitch_rel > -self.c['level_deg']: dp = max(float(dp), float(self.c['downhill_level_pwm']))   # r15: level bottom hold
            self._mark_down(_t.monotonic())   # landing-pwm-r1
            self.since = None; self.cur = float(dp); return float(dp)
        return self.safe_pwm(held_pwm)

    def safe_pwm(self, held_pwm):
        """PWM for hold / heading-blind frames: min_pwm, or the IMU boost PWM when higher."""
        self.since = None
        lim = self.c['min_pwm'] if self.base is None else max(self.c['min_pwm'], self.base)
        if self.c.get('landing_s') is not None:   # landing-pwm-r1: a lost frame on the level bottom keeps landing_pwm
            import time as _t
            land = self._landing(_t.monotonic())
            if land is not None: lim = max(lim, land['pwm'])
        self.cur = lim if self.cur is None else min(self.cur, lim)
        return lim
