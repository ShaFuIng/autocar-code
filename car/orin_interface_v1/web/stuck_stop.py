"""stuck-stop-r1 (2026-11-14 offline candidate, NOT deployed): "commanded to drive but not moving" -> end the run.

Why: run-4879 (2026-11-13) lap 2 took the wrong lane at the circle crossing and ended against a wall; from about 117.9 s both
wheel encoders read 0-5 counts/s while the motors kept getting PWM 21-28 %, and nothing in live_pid noticed it. The same pattern
(wheels decay to 0 within ~0.6 s while the PWM stays at ~21 %) is in 7 more runs of 2026-11-10..11 (da93, 218c, 8be5, ebba, 28e0,
c87f, dca0): obstacle / wall contact.

Rule (one MCU status at a time; the status carries both encoders and the motor PWM the firmware is applying):
  u      = MCU-reported motor PWM in percent = min(motor_a.pwm_command, motor_b.pwm_command) * 100 / 255 (both dir == 1 = forward)
  v_a/b  = |delta encoder count| / delta enc_sample_ms of each wheel over the newest window (win_s, at least min_span_s)
  cond   = state 'MANUAL' and both forward and u >= u_min_pct and max(v_a, v_b) < frac * table(u)    (BOTH wheels slow)
  event  = cond held on every new MCU status from the first one that met it for >= hold_s of MCU time (enc_sample_ms).
  A status gap > max_gap_ms, a counter going back (MCU restart) or a not-met status restarts the timer.
mode 'log' : the event is recorded (ev ON once per episode, ev OFF when cond ends) and nothing else happens.
mode 'stop': the caller (live_pid) ends the run with STOP_REASON through LivePID.end() - the same path as the other run-ending faults.
stuck_stop.json is read once per run (from_file at LivePID.start). enabled false / file missing / any invalid value -> from_file
returns None and live_pid does nothing at all (no read, no log). No gyro term: in the 8 stalls found the gyro was ~0 or the path was lost
(no gz on those frames), the wheel rule alone had 0 false events on 61 runs, and an IMU read would change live_pid's read order. Python 3
standard library only."""
import json, math
from pathlib import Path

REVISION = 'stuck-stop-r1'
STOP_REASON = 'PID_STUCK_NO_MOTION'
CONFIG_FILE = Path(__file__).with_name('stuck_stop.json')
# steady wheel speed (counts/s) per MCU PWM percent on the flat: speed-guard-r2 table (17.6..49.8 measured on 23 runs, below assumed)
TABLE = ((0., 0.), (5., 350.), (10., 1000.), (15., 1700.), (17.6, 2235.), (18.8, 2370.), (20.8, 2735.), (27.8, 3822.), (29.8, 4159.), (49.8, 5450.))
DEFAULTS = dict(enabled=False, mode='log', u_min_pct=12., frac=.15, hold_s=1., win_s=.45, min_span_s=.15, max_gap_ms=600.)
MODES = ('log', 'stop')


def _f(v):
    if isinstance(v, bool): raise ValueError('bool')
    f = float(v)
    if not math.isfinite(f): raise ValueError('not finite')
    return f


def parse_config(d):
    """dict -> validated config dict, or None (= off). Invalid values make the whole feature off (it never half-runs)."""
    if not isinstance(d, dict) or d.get('enabled') is not True: return None
    c = dict(DEFAULTS); c.update({k: d[k] for k in DEFAULTS if k in d})
    if c['mode'] not in MODES: return None
    try:
        for k in ('u_min_pct', 'frac', 'hold_s', 'win_s', 'min_span_s', 'max_gap_ms'): c[k] = _f(c[k])
        t = tuple((_f(a), _f(b)) for a, b in d['table']) if 'table' in d else TABLE
    except Exception: return None
    if not (5. <= c['u_min_pct'] <= 50. and 0. < c['frac'] <= .6 and .4 <= c['hold_s'] <= 5. and .2 <= c['win_s'] <= 1.5
            and 0. < c['min_span_s'] < c['win_s'] and 200. <= c['max_gap_ms'] <= 2000.): return None
    if len(t) < 2 or any(b < 0 for _, b in t) or any(t[i + 1][0] <= t[i][0] for i in range(len(t) - 1)): return None
    c['table'] = t; c['revision'] = REVISION
    return c


def table_cps(t, u):
    if u <= t[0][0]: return t[0][1]
    for (a, x), (b, y) in zip(t, t[1:]):
        if u <= b: return x + (y - x) * (u - a) / (b - a)
    return t[-1][1]


def _count(e):
    if not isinstance(e, dict) or 'forward' not in e or 'reverse' not in e: return None
    return int(e['forward']) - int(e['reverse'])


def _pct(m):
    if not isinstance(m, dict) or m.get('pwm_command') is None: return None, None
    p = float(m['pwm_command'])
    if not math.isfinite(p): return None, None
    return p * 100. / 255., m.get('dir')


class StuckStop:
    def __init__(self, c):
        self.c = c; self.mode = c['mode']; self.hist = []; self.t0_ms = None; self.fired = False; self.n = 0; self.j = 0
        self.last_ms = None; self.why = None; self.announced = False; self.ep = None

    @classmethod
    def from_file(cls, path=None):
        """once per run; None = off (enabled false, missing, unreadable or invalid file)."""
        try: c = parse_config(json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8')))
        except Exception: return None
        return cls(c) if c is not None else None

    def _speeds(self, ms, ca, cb):
        if self.hist and ms - self.hist[-1][0] > self.c['max_gap_ms']: self.hist = []
        self.hist.append((ms, ca, cb))
        self.hist = [x for x in self.hist if ms - x[0] <= self.c['win_s'] * 1000. + 1.]
        old = [x for x in self.hist[:-1] if ms - x[0] >= self.c['min_span_s'] * 1000.]
        if not old: return None, None
        m0, a0, b0 = old[-1]; dt = (ms - m0) / 1000.
        va = abs(ca - a0) / dt if ca is not None and a0 is not None else None
        vb = abs(cb - b0) / dt if cb is not None and b0 is not None else None
        return va, vb

    def _reset(self):
        rec = None
        if self.fired and self.ep is not None:
            rec = dict(revision=REVISION, ev='OFF', n=self.n, j=self.j, dur_s=round((self.last_ms - self.ep['t0_ms']) / 1000., 2) if self.last_ms is not None else None)
        self.t0_ms = None; self.fired = False; self.ep = None
        return rec

    def step(self, now, vehicle):
        """-> (event or None, log record or None). event only once per episode, when cond has held hold_s; the caller stops on it in
        mode 'stop'. Only a NEW MCU status (enc_sample_ms changed) is judged; the same status seen again returns (None, None)."""
        rec = None
        if not self.announced:
            self.announced = True
            rec = dict(revision=REVISION, ev='CFG', mode=self.mode, u_min_pct=self.c['u_min_pct'], frac=self.c['frac'], hold_s=self.c['hold_s'])
        try:
            v = vehicle if isinstance(vehicle, dict) else {}
            ms = v.get('enc_sample_ms', v.get('ms'))
            ms = None if ms is None else float(ms)
        except Exception: ms = None
        if ms is None or (self.last_ms is not None and ms == self.last_ms): return None, rec
        if self.last_ms is not None and ms < self.last_ms: self.hist = []; r = self._reset(); rec = r or rec   # MCU restart: start over
        elif self.last_ms is not None and ms - self.last_ms > self.c['max_gap_ms']:
            r = self._reset(); rec = r or rec   # status gap: continuity unknown
        self.last_ms = ms
        try: ca = _count(v.get('enc_a')); cb = _count(v.get('enc_b'))
        except Exception: ca = cb = None
        va, vb = self._speeds(ms, ca, cb)
        ua, da = _pct(v.get('motor_a')); ub, db = _pct(v.get('motor_b'))
        why = None
        if v.get('state') != 'MANUAL': why = 'STATE'
        elif ua is None or ub is None: why = 'NO_PWM'
        elif da != 1 or db != 1: why = 'NOT_FWD'
        elif min(ua, ub) < self.c['u_min_pct']: why = 'LOW_PWM'
        elif va is None or vb is None: why = 'NO_ENC'
        else:
            u = min(ua, ub); lim = self.c['frac'] * table_cps(self.c['table'], u); self.j += 1
            if max(va, vb) >= lim: why = 'MOVING'
        if why is not None:
            r = self._reset(); return None, (r or rec)
        if self.t0_ms is None: self.t0_ms = ms; self.ep = dict(t0_ms=ms)
        held = (ms - self.t0_ms) / 1000.
        if held >= self.c['hold_s'] and not self.fired:
            self.fired = True; self.n += 1
            ev = dict(revision=REVISION, ev='ON', mode=self.mode, n=self.n, j=self.j, held_s=round(held, 2), u_pct=round(min(ua, ub), 1),
                      v_right=round(va), v_left=round(vb), limit_cps=round(lim), expected_cps=round(table_cps(self.c['table'], min(ua, ub))),
                      enc_sample_ms=int(ms), state=v.get('state'), reason=v.get('reason'))
            return ev, ev
        return None, rec
