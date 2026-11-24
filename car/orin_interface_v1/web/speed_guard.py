"""speed-guard-r2 (2026-11-13, offline candidate, NOT deployed): three switches, all OFF by default.

Called only from speed_policy.py (speed-policy-r1 + speed-guard hooks). speed_guard.json is read once per run (first call of a new run id).
With all switches off (or the file missing / broken) get(lp) returns None and nothing is read, written or logged.

1. ramp_cap (r1, unchanged; the user decided 2026-11-13 that the uphill is NOT to be changed: keep enabled false, no shipped json enables it).
   speed_adapt reaches HIGH on the ramp and its target is max_pwm 50. With enabled true the PWM of every uphill frame (pitch_rel >
   level_deg, or dash_route CLIMBED when climbed_route is true) is at most uphill_cap_pwm, and speed_adapt.cur is lowered with it.
2. downhill_guard (r1, unchanged): while armed (live_pid's F.down_tail, + arm_after_s) and the faster rear wheel reaches trigger_cps, the
   PWM is held at low_pwm until both measured wheels are below release_cps (or max_hold_s / wheel data stale for max_hold_s; after such a
   timeout no re-trigger until a speed below release_cps was seen or the armed window ended). It can not brake the car.
3. overspeed_watch (r2, NEW, LOG ONLY: it never changes PWM, steering or speed_adapt state). On every new MCU status sample
   (enc_sample_ms) it compares the measured wheel speed with the speed the commanded throttle should give:
     expected = first-order lag (tau_up_s / tau_down_s) of table(u), u = the MCU-reported motor PWM (pwm_command 0..255 -> percent,
     mean of motor_a / motor_b) of the previous sample; table = steady wheel speed per PWM on the flat (fitted on 23 run logs).
     residual = min(right, left) - expected  (BOTH wheels must be fast: turns make the outer wheel fast and the inner slow).
   A sample is judged only inside the validity region (otherwise a reason code; see REASONS). An event starts when the residual is
   >= margin_cps on min_samples consecutive judged samples, and ends when a judged residual < clear_cps or nothing was judged for
   clear_unjudged_s. Logged (proposal['speed_guard']['watch'], a list): one record when the event starts (ev ON), one when it ends
   (ev OFF) and one when the reason code changes (r) - not on every frame. Each record carries n (events so far in this run) and
   j (judged samples so far in this run); live_pid has no end-of-run hook this module can use, so there is no separate run summary:
   the last record of the run carries the counts. Records produced on DOWNHILL_HOLD rows (no proposal to log into) are kept and
   written on the next frame that has one (at most 16 kept). Slopes are outside the validity region; for them one record per descent
   (ev DOWN: duration of live_pid's downhill flag and the fastest wheel speed seen while it was on) is written when the flag goes off.
Wheel speed: both encoders from the MCU status (enc_a = right, enc_b = left; |delta count| / delta enc_sample_ms over >= min_span_s), the
same meter as the downhill guard. Python 3 standard library only."""
import json, math, time
from pathlib import Path

REVISION = 'speed-guard-r2'
CONFIG_FILE = Path(__file__).with_name('speed_guard.json')
RAMP_DEFAULTS = dict(enabled=False, uphill_cap_pwm=28., level_deg=5., climbed_route=True)
DH_DEFAULTS = dict(enabled=False, trigger_cps=6000., release_cps=3500., low_pwm=5., arm_after_s=0., max_hold_s=3.,
                   min_span_s=.15, win_s=.45, stale_s=.6)
WATCH_DEFAULTS = dict(enabled=False, margin_cps=500., clear_cps=250., min_samples=2., clear_unjudged_s=1.,
                      tau_up_s=.405, tau_down_s=.631, u_min=17., u_max=50.5, min_exp_cps=1000.,
                      flat_deg=4., settle_s=2., pitch_gap_s=.5, step_pwm=6., step_settle_s=1., pwm_split=3.,
                      min_span_s=.15, win_s=.45, stale_s=.6, steer_win_s=.5)
# steady wheel speed (counts/s, both-wheel mean) per MCU PWM percent on the flat; 17.6..49.8 measured (23 runs 11-11..11-13), below 17.6 assumed
WATCH_TABLE = ((0., 0.), (5., 350.), (10., 1000.), (15., 1700.), (17.6, 2235.), (18.8, 2370.), (20.8, 2735.), (27.8, 3822.), (29.8, 4159.), (49.8, 5450.))
REASONS = ('OK', 'NO_PWM', 'NO_ENC', 'ONE_ENC', 'NOT_FWD', 'DOWNHILL', 'NO_PITCH', 'SLOPE', 'SETTLE', 'PWM_RANGE', 'STEP', 'SLOW')
MAX_PENDING = 16


def _num(v, d):
    try:
        if isinstance(v, bool): return d
        f = float(v)
        return f if f == f and abs(f) != float('inf') else d
    except (TypeError, ValueError): return d


def _table(v):
    """[[u, cps], ...] with u strictly increasing, cps >= 0, finite -> tuple of pairs; anything else -> None (default table used)."""
    try:
        t = tuple((float(a), float(b)) for a, b in v)
        if len(t) < 2 or any(not (math.isfinite(a) and math.isfinite(b)) or b < 0 for a, b in t): return None
        if any(t[i + 1][0] <= t[i][0] for i in range(len(t) - 1)): return None
        return t
    except Exception: return None


def load_config(path=None):
    """Never raises; a missing / broken file = all off. Only JSON true switches on; a bad number keeps its default."""
    c = dict(ramp_cap=dict(RAMP_DEFAULTS), downhill_guard=dict(DH_DEFAULTS), overspeed_watch=dict(WATCH_DEFAULTS, table=WATCH_TABLE))
    try: raw = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
    except Exception: return c
    if not isinstance(raw, dict): return c
    for sec, dfl in (('ramp_cap', RAMP_DEFAULTS), ('downhill_guard', DH_DEFAULTS), ('overspeed_watch', WATCH_DEFAULTS)):
        r = raw.get(sec)
        if not isinstance(r, dict): continue
        for k, d in dfl.items():
            if k not in r: continue
            c[sec][k] = (r[k] is True) if isinstance(d, bool) else _num(r[k], d)
    w = c['overspeed_watch']; r = raw.get('overspeed_watch')
    if isinstance(r, dict) and 'table' in r: w['table'] = _table(r['table']) or WATCH_TABLE
    # values that would make the watch meaningless fall back to their defaults (it stays log-only whatever the values)
    for k in ('margin_cps', 'tau_up_s', 'tau_down_s', 'min_samples', 'stale_s', 'win_s', 'min_span_s', 'flat_deg', 'u_max'):
        if w[k] <= 0: w[k] = WATCH_DEFAULTS[k]
    for k in ('clear_cps', 'clear_unjudged_s', 'settle_s', 'pitch_gap_s', 'step_settle_s', 'u_min', 'min_exp_cps', 'steer_win_s', 'pwm_split', 'step_pwm'):
        if w[k] < 0: w[k] = WATCH_DEFAULTS[k]
    if w['clear_cps'] > w['margin_cps']: w['clear_cps'] = w['margin_cps']
    if w['u_min'] >= w['u_max']: w['u_min'], w['u_max'] = WATCH_DEFAULTS['u_min'], WATCH_DEFAULTS['u_max']
    return c


class WheelMeter:
    """|counts| per second of one encoder from successive MCU samples (enc_sample_ms, falling back to the status ms)."""
    def __init__(self, key, win_s=.45, min_span_s=.15):
        self.key = key; self.win = float(win_s); self.span = float(min_span_s); self.hist = []; self.t_new = None; self.v = None

    def update(self, now, vehicle):
        try:
            e = (vehicle or {}).get(self.key) or {}
            if 'forward' not in e or 'reverse' not in e: return self.v
            c = int(e['forward']) - int(e['reverse'])
            ms = vehicle.get('enc_sample_ms')
            if ms is None: ms = vehicle.get('ms')
            ms = float(ms)
        except Exception: return self.v
        if not self.hist or ms > self.hist[-1][0]:
            if self.hist and ms - self.hist[-1][0] > 2000.: self.hist = []   # gap / MCU restart: start over
            self.hist.append((ms, c)); self.t_new = now
            self.hist = [x for x in self.hist if ms - x[0] <= self.win * 1000. + 1.]
            old = [x for x in self.hist[:-1] if ms - x[0] >= self.span * 1000.]
            if old:
                m0, c0 = old[-1]
                self.v = abs(c - c0) / ((ms - m0) / 1000.)
        elif ms < self.hist[-1][0]: self.hist = [(ms, c)]; self.t_new = now; self.v = None   # counter went back (MCU reset)
        return self.v

    def fresh(self, now, stale_s):
        return self.v is not None and self.t_new is not None and now - self.t_new <= stale_s


def _pwm_pct(m):
    """(percent, dir) of one MCU motor status dict, or (None, None)."""
    try:
        if not isinstance(m, dict) or m.get('pwm_command') is None: return None, None
        p = float(m['pwm_command'])
        if not math.isfinite(p): return None, None
        return p * 100. / 255., m.get('dir')
    except Exception: return None, None


class OverspeedWatch:
    """log-only: measured wheel speed clearly above what the commanded (MCU) PWM gives on the flat. See the module docstring."""
    def __init__(self, c):
        self.c = c; self.tab = c.get('table') or WATCH_TABLE
        self.wa = WheelMeter('enc_a', c['win_s'], c['min_span_s']); self.wb = WheelMeter('enc_b', c['win_s'], c['min_span_s'])
        self.ms = None; self.x = 0.; self.u_prev = None; self.e = None
        self.t_unflat = None; self.t_pitch = None; self.t_step = None; self.t_ok = None
        self.reason = None; self.k = 0; self.kpk = None; self.active = False; self.n = 0; self.j = 0; self.ev = None
        self.steer = []; self.pending = []; self.last_rec = None; self.down = None

    def expected_ss(self, u):
        t = self.tab
        if u <= t[0][0]: return t[0][1]
        if u >= t[-1][0]: return t[-1][1]
        for (a, va), (b, vb) in zip(t, t[1:]):
            if a <= u <= b: return va + (u - a) / (b - a) * (vb - va)
        return t[-1][1]

    def _rec(self, d):
        d['n'] = self.n; d['j'] = self.j; self.pending.append(d)
        if len(self.pending) > MAX_PENDING: self.pending = self.pending[-MAX_PENDING:]

    def step(self, now, vehicle, pitch, downhill, pwm_cmd, can_log=True):
        """update with this frame; returns a list of log records (or None). Never raises (caller also guards)."""
        c = self.c
        # pitch / downhill bookkeeping on every call (frame rate)
        if downhill: self.t_unflat = now
        if pitch is None:
            if self.t_pitch is not None and now - self.t_pitch > c['pitch_gap_s']: self.t_unflat = now   # (before the first pitch: NO_PITCH)
        else:
            self.t_pitch = now
            if abs(pitch) > c['flat_deg']: self.t_unflat = now
        v = vehicle if isinstance(vehicle, dict) else {}
        ms = v.get('enc_sample_ms', v.get('ms'))
        try: ms = float(ms) if ms is not None else None
        except (TypeError, ValueError): ms = None
        va = self.wa.update(now, v); vb = self.wb.update(now, v)
        new = ms is not None and (self.ms is None or ms != self.ms)
        if new:
            if self.ms is None: self.t_step = now   # first sample of the run: the model starts at 0 (car at rest at a run start); treated like a throttle step
            dt = 0. if self.ms is None or ms < self.ms else min((ms - self.ms) / 1000., 1.)
            self.ms = ms
            pa, da = _pwm_pct(v.get('motor_a')); pb, db = _pwm_pct(v.get('motor_b'))
            u = None if pa is None or pb is None else (pa + pb) / 2.
            fwd = da == 1 and db == 1
            uin = self.u_prev if self.u_prev is not None else (u if u is not None and fwd else 0.)
            tg = self.expected_ss(uin); x0 = self.x
            if dt > 0:
                tau = c['tau_up_s'] if tg >= x0 else c['tau_down_s']
                self.x = x0 + (1. - math.exp(-dt / tau)) * (tg - x0)
            self.e = (x0 + self.x) / 2.
            if u is not None and self.u_prev is not None and fwd and abs(u - self.u_prev) >= c['step_pwm']: self.t_step = now
            self.u_prev = u if (u is not None and fwd) else 0.
            su = v.get('steer_us_command')
            if isinstance(su, (int, float)) and not isinstance(su, bool):
                self.steer.append((now, float(su))); self.steer = [s for s in self.steer if now - s[0] <= c['steer_win_s'] + 1e-6]
            fa = self.wa.fresh(now, c['stale_s']); fb = self.wb.fresh(now, c['stale_s'])
            # validity region (first failing check is the reason)
            if u is None: r = 'NO_PWM'
            elif not fa and not fb: r = 'NO_ENC'
            elif not (fa and fb): r = 'ONE_ENC'
            elif not fwd or abs(pa - pb) > c['pwm_split']: r = 'NOT_FWD'
            elif downhill: r = 'DOWNHILL'
            elif pitch is None: r = 'NO_PITCH'
            elif abs(pitch) > c['flat_deg']: r = 'SLOPE'
            elif self.t_unflat is not None and now - self.t_unflat < c['settle_s']: r = 'SETTLE'
            elif not (c['u_min'] <= uin <= c['u_max']): r = 'PWM_RANGE'
            elif self.t_step is not None and now - self.t_step < c['step_settle_s']: r = 'STEP'
            elif self.e < c['min_exp_cps']: r = 'SLOW'
            else: r = 'OK'
            res = None
            if r == 'OK':
                self.j += 1; self.t_ok = now
                res = min(va, vb) - self.e
                if res >= c['margin_cps']: self.k += 1; self.kpk = res if self.k == 1 else max(self.kpk, res)
                else: self.k = 0
            else: self.k = 0
            if r != self.reason: self.reason = r; self._rec(dict(r=r))
            # descent summary (log only): fastest wheel while the downhill flag was on, one record when it goes off
            if downhill and (fa or fb):
                vm = max(x for x, f in ((va, fa), (vb, fb)) if f)
                if self.down is None: self.down = dict(t0=now, pk=vm, pa=va if fa else None, pb=vb if fb else None)
                elif vm > self.down['pk']: self.down.update(pk=vm, pa=va if fa else None, pb=vb if fb else None)
            elif not downhill and self.down is not None:
                d = self.down; self.down = None
                self._rec(dict(ev='DOWN', dur_s=round(now - d['t0'], 2), peak=round(d['pk']), v_a=None if d['pa'] is None else round(d['pa']), v_b=None if d['pb'] is None else round(d['pb'])))
            if not self.active and r == 'OK' and self.k >= c['min_samples']:
                self.active = True; self.n += 1
                st = [s[1] for s in self.steer]; trav = sum(abs(b[1] - a[1]) for a, b in zip(self.steer, self.steer[1:]))
                self.ev = dict(t0=now, peak=self.kpk)
                self._rec(dict(ev='ON', v_a=round(va), v_b=round(vb), exp=round(self.e), res=round(res), u=round(uin, 1),
                               mcu=[v['motor_a'].get('pwm_command'), v['motor_b'].get('pwm_command')], pwm=None if pwm_cmd is None else round(pwm_cmd, 1), pitch=None if pitch is None else round(pitch, 2),
                               steer=st[-1] if st else None, st_rng=[min(st), max(st)] if st else None, st_trav=round(trav)))
            elif self.active:
                if r == 'OK':
                    self.ev['peak'] = max(self.ev['peak'], res)
                    if res < c['clear_cps']: self._end(now, 'CLEAR', res)
                elif self.t_ok is None or now - self.t_ok >= c['clear_unjudged_s']: self._end(now, r, None)
        elif self.active and (self.t_ok is None or now - self.t_ok >= c['clear_unjudged_s'] + c['stale_s']):
            self._end(now, 'NO_DATA' if self.reason == 'OK' else self.reason, None)   # no new MCU sample (UART silent) or nothing judged
        if not can_log or not self.pending: return None
        out, self.pending = self.pending, []; self.last_rec = out[-1]
        return out

    def _end(self, now, why, res):
        self.active = False
        self._rec(dict(ev='OFF', why=why, dur_s=round(now - self.ev['t0'], 2), peak=round(self.ev['peak']), res=None if res is None else round(res)))
        self.ev = None


class SpeedGuard:
    def __init__(self, config=None, run_id=None):
        self.c = config if config is not None else load_config()
        self.run_id = run_id
        self.ramp_on = bool(self.c['ramp_cap'].get('enabled'))
        self.dh_on = bool(self.c['downhill_guard'].get('enabled'))
        w = self.c.get('overspeed_watch') or {}
        self.watch_on = bool(w.get('enabled'))
        self.watch = OverspeedWatch(w) if self.watch_on else None
        d = self.c['downhill_guard']
        self.wa = WheelMeter('enc_a', d['win_s'], d['min_span_s']); self.wb = WheelMeter('enc_b', d['win_s'], d['min_span_s'])
        self.armed_until = None; self.active = False; self.blocked = False; self.t_trig = None; self.t_fresh = None; self.n_trig = 0; self.last = None

    @property
    def on(self):
        return self.ramp_on or self.dh_on or self.watch_on

    # ---- 1. ramp cap
    def ramp_limit(self, pitch_rel, route):
        """cap PWM or None. Uphill = pitch_rel > level_deg, or route CLIMBED (climbed_route true; covers a stale IMU / pitch None)."""
        if not self.ramp_on: return None
        c = self.c['ramp_cap']
        up = (pitch_rel is not None and _num(pitch_rel, 0.) > float(c['level_deg'])) or (bool(c.get('climbed_route')) and route == 'CLIMBED')
        return float(c['uphill_cap_pwm']) if up else None

    # ---- 2. downhill guard
    def dh_step(self, now, vehicle, armed_flag):
        """update the state; returns (cap PWM or None, info dict or None)."""
        if not self.dh_on: return None, None
        d = self.c['downhill_guard']
        va = self.wa.update(now, vehicle); vb = self.wb.update(now, vehicle)
        fa = self.wa.fresh(now, d['stale_s']); fb = self.wb.fresh(now, d['stale_s'])
        vs = [v for v, f in ((va, fa), (vb, fb)) if f and v is not None]
        vmax = max(vs) if vs else None
        if vs: self.t_fresh = now
        if armed_flag: self.armed_until = now + float(d['arm_after_s'])
        armed = self.armed_until is not None and now <= self.armed_until
        why = None
        if self.blocked and (not armed or (vmax is not None and vmax < float(d['release_cps']))): self.blocked = False
        if not self.active:
            if armed and not self.blocked and vmax is not None and vmax >= float(d['trigger_cps']):
                self.active = True; self.t_trig = now; self.n_trig += 1; why = 'TRIGGER'
        else:
            if vs and all(v < float(d['release_cps']) for v in vs): self.active = False; why = 'RELEASE'
            elif now - self.t_trig >= float(d['max_hold_s']): self.active = False; self.blocked = True; why = 'TIMEOUT'
            elif self.t_fresh is not None and now - self.t_fresh >= float(d['max_hold_s']): self.active = False; self.blocked = True; why = 'NO_WHEEL_DATA'
        if not armed and not self.active and why is None: return None, None
        info = dict(state='ACTIVE' if self.active else 'ARMED', why=why, v_a=None if va is None or not fa else round(va),
                    v_b=None if vb is None or not fb else round(vb), since_s=None if self.t_trig is None or not (self.active or why) else round(now - self.t_trig, 2))
        return (float(d['low_pwm']) if self.active else None), info

    # ---- 3. overspeed watch (log only)
    def watch_step(self, now, vehicle, pitch_rel, downhill, pwm, can_log=True):
        if self.watch is None: return None
        try:
            p = None if pitch_rel is None else _num(pitch_rel, None)
            return self.watch.step(now, vehicle, p, bool(downhill), None if pwm is None else _num(pwm, None), can_log)
        except Exception as e:
            return [dict(r='ERROR', error=type(e).__name__)] if can_log else None

    # ---- one entry point per frame type
    def apply(self, now, pwm, pitch_rel, route, vehicle, armed_flag, downhill=False, can_log=True):
        """returns (new pwm, log dict or None). pwm None / 0 is passed through unchanged (stop / start-centre frames).
        downhill: live_pid's steep-downhill flag (F.downhill) or a DOWNHILL_HOLD row (watch only); can_log False: the caller has no
        proposal to write into (DOWNHILL_HOLD), the watch keeps its records for the next frame. The watch never changes the returned pwm."""
        log = None
        wr = self.watch_step(now, vehicle, pitch_rel, downhill, pwm, can_log)
        rc = self.ramp_limit(pitch_rel, route)
        dc, di = self.dh_step(now, vehicle, armed_flag)
        if pwm is None or not pwm:
            if di or wr:
                log = dict(revision=REVISION)
                if di: log['dh'] = di
                if wr: log['watch'] = wr
            return pwm, log
        out = float(pwm)
        if rc is not None or di is not None or wr:
            log = dict(revision=REVISION)
            if rc is not None:
                log['ramp'] = dict(cap=rc, applied=out > rc, pwm_before=round(out, 2))
                out = min(out, rc)
            if di is not None:
                di = dict(di, low_pwm=dc, applied=dc is not None and out > dc, pwm_before=round(out, 2))
                log['dh'] = di
                if dc is not None: out = min(out, dc)
            if wr: log['watch'] = wr
        self.last = log
        return out, log


def get(lp):
    """the guard of this run, or None when all switches are off. Built at the first call of each run (config read once per run)."""
    try:
        rid = (getattr(lp, 'run', None) or {}).get('id')
        g = getattr(lp, '_speed_guard', None)
        if g is None or g.run_id != rid:
            g = SpeedGuard(load_config(), rid); lp._speed_guard = g
        return g if g.on else None
    except Exception: return None
