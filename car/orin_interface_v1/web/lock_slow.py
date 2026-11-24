"""Slower at the right steering lock (lock-slow-r1).

User 2026-11-05: keep the right lock at 1750 but take the corners slower.
Records: right-lock curvature is speed dependent (diff-assist lock_curv.json: 10.9 deg / 1000 counts at 1400-1900 counts/s,
9.4 at 1900-2200; tighter when slower). Wheel speed at right lock by PWM (controls.jsonl 11-03..05, steer >= 1745): PWM 15
median 1828 counts/s (787 samples), PWM 13 1174 (68 samples, none stalled, p10 1040). Runs after the 17:06 deploy: right lock
0.9-5.8 s per run, PID desired up to 1858-1981.
While the sent steering is within ON_US of max_us (hysteresis: until it is more than OFF_US below), the PWM is capped at
LOCK_PWM. Only lowers: on the downhill (PWM 7) nothing changes; on the level bottom curve after the slope it applies too.
Not on the uphill (needs the drive). Left side unchanged. `downhill` is logged only.
Values are experiment values (13 = the upper-section lane cap already used on this car), not safety limits.
"""
REVISION = 'lock-slow-r1'
ENABLED = False   # 2026-11-05 18:5x user: restore the original turning (no PWM 13 at the right lock); flat right-lock episodes 6-12 s with it vs 2.8 s without
LOCK_PWM = 13.
ON_US = 30.
OFF_US = 60.


class LockSlow:
    def __init__(self):
        self.on = False

    def apply(self, proposal, params, downhill):
        if not ENABLED:
            self.on = False; return None
        s = proposal.get('steer_us'); pw = proposal.get('pwm')
        uphill = (proposal.get('imu_boost') or {}).get('mode') == 'uphill' or (proposal.get('speed_adapt') or {}).get('reason') == 'UPHILL'
        if s is None or not pw or uphill:
            self.on = False; return None
        mx = float(params['max_us']); s = float(s)
        if s >= mx - ON_US: self.on = True
        elif s < mx - OFF_US: self.on = False
        if not self.on or float(pw) <= LOCK_PWM: return None
        info = dict(revision=REVISION, pwm_before=round(float(pw), 2), pwm=LOCK_PWM, steer_us=round(s, 1), downhill_window=bool(downhill))
        proposal['pwm'] = LOCK_PWM; proposal['lock_slow'] = info
        return info
