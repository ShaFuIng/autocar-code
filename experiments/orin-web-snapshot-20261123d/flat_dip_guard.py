"""Flat-dip guard (flat-dip-guard-r1, 2026-11-05).

Problem (live 6a5d13d4, new MPU6050 since 15:39): on the flat part before the climb (route WAIT_CLIMB) the
filtered pitch went below -2 deg 7 times for 0.3 s or more (min -14.9 deg, longest 2.51 s, mostly in sharp
turns). The old chip never did this in 20 runs on 2026-11-05 (flat minimum -1.3 deg). The pitch gyro of the
new mount is about 3x noisier on the flat (std 6.2 vs 2.0 dps, bursts > 15 dps 4.9 % vs 0 %). The dips set
speed-adapt DOWNHILL (PWM to 10 at once: 7 drops >= 5 PWM within 0.5 s) and armed bottom-commit on the flat
(recorded active at 22.4 s and 54.4 s: right floor 1569-1675 us while the PID asked 1190-1507 us).

A duration rule does not separate them: the 2.51 s / -14.9 deg false dip is close to a real slope (3.0-3.7 s
below -2, -15..-17 deg). The track does: the downhill always comes after the climb, and the route leaves
WAIT_CLIMB once the climb is seen (8 deg for 2 s; the real climb read 12-14 deg for about 10 s). So while the
route is WAIT_CLIMB the downhill consumers get at least DIP_DEG + 0.1 (level for them). Not applied in the
first ARM_S of WAIT_CLIMB, so a lap step at the slope bottom keeps the old behaviour. Other routes unchanged.
Trade-off: if a climb were missed, that lap's downhill would run without the downhill rules (as before the
IMU existed). Values are experiment values, not safety limits.
"""
REVISION = 'flat-dip-guard-r1'
DIP_DEG = -2.0      # downhill consumers treat < -2 as pitched down (bottom_commit / bottom_left / downhill_bottom LEVEL_PITCH)
ARM_S = 3.0


class FlatDipGuard:
    def __init__(self):
        self.wait_t0 = None; self.active = False; self.guarding = False; self.info = None

    def step(self, now, pitch, route):
        """Returns the pitch the downhill logic should use this frame."""
        self.info = None; self.active = False; self.guarding = False
        if route != 'WAIT_CLIMB':
            self.wait_t0 = None; return pitch
        if self.wait_t0 is None: self.wait_t0 = now
        if now - self.wait_t0 < ARM_S: return pitch
        self.guarding = True
        if pitch is None or pitch >= DIP_DEG: return pitch
        self.active = True
        self.info = dict(revision=REVISION, raw_pitch_rel_deg=round(float(pitch), 2))
        return DIP_DEG + .1

    def clamp(self, pitch):
        """Same decision for another pitch reading of this frame (speed adapt uses the imu_boost value)."""
        if self.guarding and pitch is not None and pitch < DIP_DEG: return DIP_DEG + .1
        return pitch
