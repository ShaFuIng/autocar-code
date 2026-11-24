"""Uphill PWM boost from the MPU6050 pitch (record-only logger -> live PID).

Live 5c400352 (PWM15): about 17 s on the 13-15 deg ramp; near the top the
right boundary faded for ~3.5 s and the corridor lost support -> stop. More
power only on the slope shortens that time without changing the flat-ground
behaviour tuned at PWM15.

Reads logs/imu/latest.json written by imu_logger.py (imu-logger-r3) with
time.monotonic() timestamps (same clock as this process). Pitch relative to
flat = pitch_deg - flat_pitch_deg. Uphill turns on after pitch_rel >= on_deg
for hold_s, off when pitch_rel < off_deg (hysteresis); optional downhill PWM
when pitch_rel <= -downhill_on_deg. Anything wrong (no file, stale > max_age_s,
bad values, config off) -> base PWM and the reason is recorded. Never raises
into the PID loop; never exceeds cap_pwm or the run's started ceiling.
All numbers are experiment values in imu_boost.json, not safety limits.
"""
import json, math, time
import pause_clock   # pause-clock-r1 (pause-resume-20261120)
from pathlib import Path

REVISION = 'imu-boost-r2'
CONFIG_FILE = Path(__file__).resolve().with_name('imu_boost.json')
# r2 (live 899318f1): the ramp entry held 5.4-7.8 deg for 1.7 s below on_deg 8 with
# full steering lock -> the car nearly stalled, then PWM jumped 15 -> 20 in one frame.
# on 5 / off 3 deg and the output PWM moves at most pwm_rate per second.
DEFAULTS = dict(enabled=True, uphill_pwm=20, on_deg=5., off_deg=3., hold_s=.2, max_age_s=.3, flat_pitch_deg=1.66, pwm_rate=10.,
                downhill_pwm=None, downhill_on_deg=8., cap_pwm=25,
                latest_path='/home/jetson/orin_interface_v1/logs/imu/latest.json')


def load_config(path=None):
    try:
        c = dict(DEFAULTS, **json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8')))
    except (OSError, ValueError, TypeError):
        return dict(DEFAULTS, enabled=False, config_error='missing or invalid imu_boost.json')
    for k in ('uphill_pwm', 'cap_pwm'):
        if type(c[k]) not in (int, float) or not 0 <= c[k] <= 25: return dict(c, enabled=False, config_error='invalid ' + k)
    if c['downhill_pwm'] is not None and (type(c['downhill_pwm']) not in (int, float) or not 0 <= c['downhill_pwm'] <= 25):
        return dict(c, enabled=False, config_error='invalid downhill_pwm')
    if not c['off_deg'] < c['on_deg']: return dict(c, enabled=False, config_error='off_deg must be below on_deg')
    return c


class ImuBoost:
    def __init__(self, config=None, clock=pause_clock.monotonic):   # pause-clock-r1: was time.monotonic
        self.c = load_config() if config is None else dict(DEFAULTS, **config)
        self.clock = clock; self.mode = 'flat'; self.above_since = None; self.out = None; self.out_t = None

    def ceiling(self, base):
        """PWM ceiling to start the run with (manual_drive rejects frames above it)."""
        if not self.c.get('enabled'): return int(base)
        return int(min(self.c['cap_pwm'], max(base, self.c['uphill_pwm'])))

    def _read(self):
        d = json.loads(Path(self.c['latest_path']).read_text(encoding='utf8'))
        t, p = pause_clock.imu_t(float(d['t'])), float(d['pitch_deg'])   # pause-clock-r1
        if not (math.isfinite(t) and math.isfinite(p)): raise ValueError('non-finite IMU value')
        return t, p

    def step(self, base, ceiling):
        info = dict(revision=REVISION, base_pwm=base, mode=self.mode, applied=False)
        if not self.c.get('enabled') or not base:
            info.update(reason='DISABLED' if not self.c.get('enabled') else 'ZERO_BASE'); return base, info
        now = self.clock()
        try:
            t, p = self._read()
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.mode = 'flat'; self.above_since = None
            info.update(mode='flat', reason='IMU_UNAVAILABLE', error=type(e).__name__); return base, info
        age = now - t; rel = p - self.c['flat_pitch_deg']
        info.update(imu_t=t, imu_age_s=round(age, 3), pitch_rel_deg=round(rel, 2))
        if not -.05 <= age <= self.c['max_age_s']:
            self.mode = 'flat'; self.above_since = None
            info.update(mode='flat', reason='IMU_STALE'); return base, info
        if self.mode == 'uphill':
            if rel < self.c['off_deg']: self.mode = 'flat'; self.above_since = None
        else:
            if rel >= self.c['on_deg']:
                self.above_since = now if self.above_since is None else self.above_since
                if now - self.above_since >= self.c['hold_s']: self.mode = 'uphill'
            else:
                self.above_since = None
                self.mode = 'downhill' if self.c['downhill_pwm'] is not None and rel <= -self.c['downhill_on_deg'] else 'flat'
        target = base
        if self.mode == 'uphill': target = min(self.c['uphill_pwm'], self.c['cap_pwm'], ceiling)
        elif self.mode == 'downhill': target = min(self.c['downhill_pwm'], ceiling)
        pwm = self._slew(now, target, base)
        info.update(mode=self.mode, reason='OK', applied=pwm != base, pwm=pwm, target_pwm=target)
        return pwm, info

    def _slew(self, now, target, base):
        """r2: move the output PWM toward the target by at most pwm_rate per second,
        starting from the last output (or the base PWM after a gap / at the start)."""
        rate = self.c.get('pwm_rate')
        fresh = self.out is not None and self.out_t is not None and 0 < now - self.out_t <= .5
        prev = self.out if fresh else float(base)
        if not rate: out = float(target)
        else:
            step = rate * (now - self.out_t) if fresh else 0.
            out = min(max(float(target), prev - step), prev + step)
        self.out, self.out_t = round(out, 2), now
        return self.out
