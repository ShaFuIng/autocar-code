"""Record-only steering centre estimate from gyro straight windows (centre-estimator-r1).

User 2026-11-02/03: the car's straight point seems to drift; estimate it automatically, left and right
separately, with a low-pass (EMA). 2026-11-02 offline (offline-review-20261102/straight_point.py): steady
windows 1474-1528 us, but arriving from the left needed ~1593 and from the right ~1507 (play ~86 us).

Per dispatched PID command (steering, PWM) and the IMU latest.json (gyro z, pitch):
 a window is accepted when, over WIN_S ending LAG_S ago (dispatched command -> yaw response, 0.5 + 0.35 s
 measured in e5639ab0), the commanded steering moved < SPREAD_US, the
 mean |yaw rate| over the last WIN_S is < GZ_MAX deg/s, the pitch is within PITCH_TOL of flat and PWM > 0.
 Sample = mean steering of the window. Side = where the steering came from (the WIN_S before the window:
 mean more than SIDE_US above -> 'right', below -> 'left', else 'steady').
 Each side keeps an EMA (ALPHA); a sample more than GATE_US from its side's estimate is rejected
 (GATE_FIRST_US around the saved centre until the side has MIN_N samples).
 centre = mean of left and right once both have MIN_N samples, else the steady / any estimate;
 play = right-arrival minus left-arrival difference. Nothing here changes the steering (record only).
The estimate is saved to STATE_PATH at the end of each run and continued in the next run.
All numbers are experiment values.
"""
import json, math, time
import pause_clock   # pause-clock-r1 (pause-resume-20261120)
from pathlib import Path

REVISION = 'centre-estimator-r2'
WIN_S, LAG_S, SPREAD_US, GZ_MAX, PITCH_TOL, SIDE_US = .7, .85, 30., 1.5, 3., 20.
# r2 (user 2026-11-10: only 1-6 samples per run): window 0.5 s, |mean gz| < 3 deg/s, steering spread < 40 us -> 3-5x the samples on
# the same runs (62fa..4b53 replay); live_pid only allows samples on level straights (route WAIT_CLIMB, no mapped curve) and before a collision.
WIN_S, SPREAD_US, GZ_MAX = .5, 40., 3.
ALPHA, GATE_US, GATE_FIRST_US, MIN_N = .1, 150., 250., 3
STATE_PATH = Path.home() / '.config/orin_car/centre_estimate.json'


class CentreEstimator:
    def __init__(self, center_us, flat_pitch_deg, latest_path, state_path=STATE_PATH):
        self.saved = float(center_us); self.flat = float(flat_pitch_deg); self.latest = Path(latest_path); self.state_path = Path(state_path)
        self.cmd = []; self.gz = []; self.last_accept = None; self.samples = []
        self.est = {k: dict(value=None, n=0) for k in ('left', 'right', 'steady')}
        try:
            d = json.loads(self.state_path.read_text(encoding='utf8'))
            for k in self.est:
                if isinstance(d.get(k), dict) and d[k].get('value') is not None: self.est[k] = dict(value=float(d[k]['value']), n=int(d[k].get('n', 0)))
        except (OSError, ValueError, TypeError): pass

    def _imu(self, now):
        d = json.loads(self.latest.read_text(encoding='utf8'))
        t, g, p = pause_clock.imu_t(float(d['t'])), float(d['gyro_dps'][2]), float(d['pitch_deg'])   # pause-clock-r1
        if not all(map(math.isfinite, (t, g, p))) or not -.05 <= now - t <= .3: raise ValueError('stale imu')
        return t, g, p

    def step(self, now, steer_us, pwm, allow=True):
        """Call once per dispatched command; returns an accepted sample dict or None. allow=False keeps the history but takes no sample (r2)."""
        self.cmd.append((now, float(steer_us), float(pwm))); self.cmd = [c for c in self.cmd if now - c[0] <= 3.]
        try:
            t, g, p = self._imu(now)
            if not self.gz or t > self.gz[-1][0]: self.gz.append((t, g, p))
        except (OSError, ValueError, KeyError, TypeError, IndexError): return None
        self.gz = [x for x in self.gz if now - x[0] <= 2.]
        if not allow: return None   # r2
        if self.last_accept is not None and now - self.last_accept < WIN_S: return None
        w = [c for c in self.cmd if now - LAG_S - WIN_S <= c[0] <= now - LAG_S]
        gw = [x for x in self.gz if now - WIN_S <= x[0] <= now]
        if len(w) < 4 or len(gw) < 3 or w[-1][0] - w[0][0] < WIN_S * .7: return None
        st = [c[1] for c in w]
        if max(st) - min(st) >= SPREAD_US or min(c[2] for c in w) <= 0: return None
        if abs(sum(x[1] for x in gw) / len(gw)) >= GZ_MAX or abs(sorted(x[2] for x in gw)[len(gw) // 2] - self.flat) > PITCH_TOL: return None
        m = sum(st) / len(st)
        prev = [c[1] for c in self.cmd if now - LAG_S - 2 * WIN_S <= c[0] < now - LAG_S - WIN_S]
        side = 'steady' if not prev else 'right' if sum(prev) / len(prev) > m + SIDE_US else 'left' if sum(prev) / len(prev) < m - SIDE_US else 'steady'
        e = self.est[side]; ref = e['value'] if e['value'] is not None else self.saved
        self.last_accept = now
        s = dict(t=round(now, 3), steer_us=round(m, 1), side=side, accepted=abs(m - ref) <= (GATE_US if e['n'] >= MIN_N else GATE_FIRST_US))
        if s['accepted']: e['value'] = m if e['value'] is None else e['value'] + ALPHA * (m - e['value']); e['n'] += 1
        self.samples.append(s); self.samples = self.samples[-200:]
        return dict(s, estimate=self.summary())

    def summary(self):
        L, R, Sd = self.est['left'], self.est['right'], self.est['steady']
        if L['n'] >= MIN_N and R['n'] >= MIN_N: centre, src = (L['value'] + R['value']) / 2, 'left_right_mean'
        elif Sd['value'] is not None: centre, src = Sd['value'], 'steady'
        else:
            vals = [x['value'] for x in (L, R) if x['value'] is not None]; centre, src = (sum(vals) / len(vals), 'one_side') if vals else (None, 'none')
        play = None if L['value'] is None or R['value'] is None else round(L['value'] - R['value'], 1)
        r = lambda v: None if v is None else round(v, 1)
        return dict(revision=REVISION, record_only=True, saved_center_us=self.saved, centre_us=r(centre), centre_source=src, play_us=play,
                    left=dict(value=r(L['value']), n=L['n']), right=dict(value=r(R['value']), n=R['n']), steady=dict(value=r(Sd['value']), n=Sd['n']))

    def save(self):
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            d = dict(self.summary(), saved_at=time.time(), **{k: dict(v) for k, v in self.est.items()})
            tmp = self.state_path.with_suffix('.tmp'); tmp.write_text(json.dumps(d, indent=1), encoding='utf8'); tmp.replace(self.state_path)
        except OSError: pass
