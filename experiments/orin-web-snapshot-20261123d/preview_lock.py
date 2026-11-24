"""Earlier turn-in from the far preview row (preview-lock-r1).

User 2026-11-05: keep the straight speed (30 / 35) and turn in earlier at the curves. 96cc5fb6 entry 2 (28.0 s): at PWM 30 the
gain schedule (k = 15 / PWM = 0.5) halved the steering while the far preview row already rose (27.6-28.0 s far 0.27 -> 0.61,
sent +40..+100 us instead of +100..+200), and the lock reached the MCU 0.5-0.6 s after the Orin sent it (steer rate 1500 us/s
is the backend maximum). Two parts, both on normal-path frames of the lower lap only (live_pid decides eligibility):
 1 k_for: the gain-schedule cut fades out while |far| rises from K_FAR0 to K_FAR1 (both directions).
 2 PreviewLock.step: right curves only, while far > ON_FAR (off below OFF_FAR) and the near error is not to the left,
   the steering is at least centre + (max_us - centre) * (far - ON_FAR) / (FULL_FAR - ON_FAR), i.e. full right lock at FULL_FAR.
Records 2026-11-05 (114 lock onsets, 515 s of calm straight frames): far above 0.5 came before 107 onsets (median 0.95 s) and
on 14.5 s of calm frames; above 0.6: 104, 0.72 s, 3.4 s; at the fast 96cc entry only about 0.1 s. Experiment values, not
safety limits.
Replay of these records (test.json): FULL_FAR 0.8 moved no fast (PWM >= 25) lock earlier; 0.6 moved 6 of 10 by 0.08-0.29 s
with the floor on 15.2 s of 1167 s calm frames (0.65: 12.7 s, 0.45 / 0.6: 22.8 s).
"""
REVISION = 'preview-lock-r1-off'
# user 2026-11-05 22:1x: switched off. f3e60d73 2.43 s: the straight weave made the far row read 0.60 and the floor held the
# full right lock 0.6 s -> swing to -0.77; 5 of 32 triggers in 5 runs overshot (run-f3e6-20261105/findings.json).
ENABLED = False
K_FAR0, K_FAR1 = .3, .6
ON_FAR, OFF_FAR, FULL_FAR = .5, .4, .6
NEAR_MIN = -.1


def k_for(k, far):
    """Returns (k used, info or None)."""
    if not ENABLED or far is None or k >= 1.: return k, None
    a = max(0., min(1., (abs(float(far)) - K_FAR0) / (K_FAR1 - K_FAR0)))
    if a <= 0.: return k, None
    kk = k + (1. - k) * a
    return kk, dict(revision=REVISION, k_before=round(k, 3), k=round(kk, 3), far=round(float(far), 3))


class PreviewLock:
    def __init__(self):
        self.on = False

    def step(self, steer_us, far, err, center_us, max_us, eligible):
        """Returns (steer_us, info or None)."""
        if not ENABLED or not eligible or far is None or err is None or float(err) < NEAR_MIN:
            self.on = False; return steer_us, None
        far = float(far)
        if self.on and far < OFF_FAR: self.on = False
        elif not self.on and far > ON_FAR: self.on = True
        if not self.on: return steer_us, None
        a = max(0., min(1., (far - ON_FAR) / (FULL_FAR - ON_FAR)))
        floor = float(center_us) + (float(max_us) - float(center_us)) * a
        info = dict(revision=REVISION, far=round(far, 3), floor_us=round(floor, 1), steer_before_us=round(float(steer_us), 1))
        if float(steer_us) >= floor:
            info['applied'] = False; return steer_us, info
        info['applied'] = True
        return min(float(max_us), floor), info
