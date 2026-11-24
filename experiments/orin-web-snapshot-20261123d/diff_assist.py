"""r3 (user 2026-11-06): manual runs 5a3aaf06 .. f849f9e9 with command right 30 / left 20 turned 82.6 deg/m at the right
lock (radius ~69 cm, 69 cm/s) against 52.2 deg/m with 20 / 20 (~110 cm, 59 cm/s), 46.1 vs 49.8 deg/m at the left lock, and turned
right with centred steering: the motor-A ("right") command drives the physical LEFT rear wheel. Yesterday's r1/r2 slowed the
wrong wheel on right turns. swap_sides true exchanges the two commands so the outer wheel is the faster one.
Rear-wheel PWM difference as steering assist on the level lower lap (diff-assist-r1).

User 2026-11-05: the right steering is at its mechanical limit (1750 us) and the car still runs wide on the flat right curves;
no slowing down (speed will go up later). Firmware already drives motor A (right wheel, 'right') and motor B (left wheel,
'left') from separate PWM values; the PID sent the same value to both.

Records 11-04/05 (runs-20261105j.json, encoder A/B + IMU): curvature per command rises from +180 us and stops growing at about
+300 us (1690 = 1750); right full lock is about 70 % of left full lock. Here, beyond START_US to the right of centre the inner
(right) wheel gets PWM - d and the outer (left) wheel PWM + d; the mean PWM stays the same. d grows linearly to MAX_DIFF_PCT at
FULL_US, moves at most RATE_PCT_S, and is limited so that the outer wheel <= ceiling and the inner wheel >= MIN_INNER_PCT.

Only on valid PID frames on the level lower lap (route in ROUTES, |pitch| <= MAX_PITCH_DEG) with no special control
(dashed turn, crest midpoint, heading hold, bottom commit) and not on / just after the downhill; every other frame sends equal
PWM (d = 0 at once). Left turns are unchanged (RIGHT_ONLY). All numbers are first experiment values, not safety limits.

r2 inner-only (user 2026-11-05: keep the straight speed, slow the inner wheel in the curve): with mode 'inner_only' the inner
(right) wheel gets PWM - d and the outer wheel keeps PWM (r1 'balanced' raised the outer wheel, which max_outer_pct 25 blocked
at PWM >= 25). live_pid sends the mean of the two as pwm (backend: mean = pwm, each wheel within 10 of it -> d <= 20).
"""
import json
from pathlib import Path

REVISION = 'diff-assist-r3'
CONFIG_FILE = Path(__file__).with_name('diff_assist.json')
DEFAULTS = dict(enabled=True, start_us=240., full_us=330., max_diff_pct=5., min_inner_pct=10., max_outer_pct=25.,
                rate_pct_s=40., restart_gap_s=.3, routes=['WAIT_CLIMB'], max_pitch_deg=3., right_only=True, mode='balanced')


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


class DiffAssist:
    def __init__(self, config=None):
        self.c = dict(config) if config is not None else load_config()
        self.d = 0.; self.t = None

    def target(self, dev):
        """Signed PWM difference wanted for a steering offset dev (us from centre); + = right turn (right wheel slower)."""
        c = self.c; a = abs(dev)
        if a <= c['start_us'] or (c.get('right_only') and dev < 0): return 0.
        span = max(1e-6, float(c['full_us']) - float(c['start_us']))
        return (1. if dev > 0 else -1.) * float(c['max_diff_pct']) * min(1., (a - float(c['start_us'])) / span)

    def step(self, now, pwm, steer_us, center_us, ceiling, eligible, why=None):
        """Returns (right_pct, left_pct, info). eligible False -> equal PWM and d reset."""
        c = self.c; pwm = float(pwm or 0.)
        info = dict(revision=REVISION, pwm=round(pwm, 2))
        if not c.get('enabled') or not eligible or pwm <= 0:
            self.d = 0.; self.t = now
            info.update(applied=False, reason='DISABLED' if not c.get('enabled') else (why or ('ZERO_PWM' if pwm <= 0 else 'NOT_ELIGIBLE')), d=0.)
            return pwm, pwm, info
        dev = float(steer_us) - float(center_us); want = self.target(dev)
        restart = self.t is None or now - self.t > c['restart_gap_s']   # after a gap (hold / other frames) ramp again from 0
        if restart: self.d = 0.
        dt = 0. if restart else max(0., now - self.t); self.t = now
        step = float(c['rate_pct_s']) * dt
        d = self.d + max(-step, min(step, want - self.d))
        inner = c.get('mode') == 'inner_only'
        top = min(float(ceiling), float(c['max_outer_pct']))
        lim = max(0., min(top - pwm, pwm - float(c['min_inner_pct'])))   # outer <= top, inner >= min_inner
        if inner: lim = max(0., min(20., pwm - float(c['min_inner_pct'])))   # r2: outer unchanged, inner >= min_inner
        limited = abs(d) > lim
        if limited: d = lim if d > 0 else -lim
        self.d = d
        right, left = (pwm - max(d, 0.), pwm + min(d, 0.)) if inner else (pwm - d, pwm + d)   # r2 inner-only: only the inner wheel slows
        if c.get('swap_sides'): right, left = left, right   # r3: the 'right' command (motor A) drives the physical LEFT rear wheel (2026-11-06 manual runs: R30/L20 turned right)
        info.update(applied=abs(d) > 1e-9, reason='OK', mode='inner_only' if inner else 'balanced', swap_sides=bool(c.get('swap_sides')), dev_us=round(dev, 1), want=round(want, 2), d=round(d, 2), limited=limited,
                    right_pct=round(right, 2), left_pct=round(left, 2))
        return right, left, info
