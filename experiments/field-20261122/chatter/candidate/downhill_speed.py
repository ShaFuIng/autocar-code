"""Speed-scaled steering on the downhill (downhill-speed-r1).

User 2026-11-03 (live 9fd69ba7): at PWM 10 the car still sped up on the downhill (encoder B 4400-6500 counts/s, flat
PWM 15 about 2500), the right-turn assist (centre + 300 us) made it yaw -53 .. -107 deg/s and it went off. The motors
cannot brake (PWM 0 only coasts), so the steering is made gentler instead: the offset from centre is scaled by
REF_SPEED / measured speed (not below K_MIN), and the assist is centre + ASSIST_US.

Speed: encoder B forward - reverse counts from the vehicle status, rate over the last WIN_S. Experiment values.
"""
REVISION = 'downhill-speed-r5'
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value
import json as _json   # speed-meter-mcu-r1 (field-20261122/chatter): web/speed_meter.json
from pathlib import Path as _Path   # speed-meter-mcu-r1
MCU_REVISION = 'speed-meter-mcu-r1'
MCU_CONFIG = _Path(__file__).with_name('speed_meter.json')
REF_SPEED, K_MIN, WIN_S, ASSIST_US, HOLD_RIGHT_US, K_RISE = 3000., .75, .6, 150., 150., .1   # 0.3 s window was too noisy (1349-3820/s frame to frame); r2: REF 2500 -> 2000
# r5 (user 2026-11-06, live 342643ee / 3d74f15f, new linkage, kp 220): on the slope the car slid to the INSIDE of the bottom right
# bend (error -0.53) while the PID asked 1096-1290 (left) and the r4 scale 0.46-0.58 at 3600-4400 counts/s sent only 1229-1390; at
# the bottom the error flipped and the car zigzagged. Left commands are now scaled like the right ones (REF 2000 -> 3000, K_MIN
# 0.4 -> 0.75): at 4400 counts/s k 0.75 instead of 0.45. The yaw-rate limit, holds and the right side are unchanged. Experiment values.
REF_SPEED_RIGHT, K_MIN_RIGHT = 3000., .75   # downhill-right-r1 (user 2026-11-05): right commands scaled less (the slope is a right curve)
# r2 (live 8fb463fa 26.8-27.5 s): the car still yawed right at -42 .. -87 deg/s on the slope. Yaw-rate limit: beyond +-GZ_MAX
# deg/s the steering gets K_GZ us per deg/s against the rotation (at most +-GZ_CAP_US). Any hold on the downhill (also the
# plain 0.4 s PID hold, which kept 1031 us in 8fb463fa) stays within centre -+HOLD_US. Experiment values.
GZ_MAX, K_GZ, GZ_CAP_US, HOLD_US = 25., 5., 150., 150.   # r3: the one successful downhill (1f3d7ba7) stayed within -30..+25 deg/s, the failed ones whipped -78..-98 / +55..+90; 45/4 -> 25/5
# r2 (user: all three runs went out on the outside of the right curve, too far right): the right assist is off
# (ASSIST_US 0: the floor becomes the centre, below what the P term gives for an error > 0.4).
ASSIST_US = 0.
# r4 (live 8d889766 lap 2, 166.21 s): 0.4 s after the slope start the car still turned left at +42 deg/s from the crest and the
# 25 deg/s limit added +49.5 us to the right before the big right curve. No yaw-rate limit within ENTRY_DELAY_S of the first
# pitch below -8 deg (downhill_entry slope start). Experiment value.
ENTRY_DELAY_S = .5


def gz_term(gz):
    """Steering added against rotations faster than +-GZ_MAX (us), 0 inside the band."""
    ex = gz - GZ_MAX if gz > GZ_MAX else (gz + GZ_MAX if gz < -GZ_MAX else 0.)
    return max(-GZ_CAP_US, min(GZ_CAP_US, K_GZ * ex))



class SpeedMeter:
    def __init__(self):
        self.hist = []; self.k_last = None
        self._mcu_init()   # speed-meter-mcu-r1: self.mcu None (file absent / mode not 'mcu') = the old rate on every call

    def _mcu_init(self):
        """speed-meter-mcu-r1 (field-20261122/chatter): the frame-time rate below jumps +30 % for one frame on 37 % of the new 5 Hz
        MCU statuses (the count steps by 0.2 s of travel while the Orin-clock window does not), 7.5 % of all frames; curve-2 / curve-4
        bands turn each one into a one-frame 30-60 us command pulse and two play-comp flips. mode 'mcu': rate over MCU status time."""
        self.mcu = None; self.mh = []; self.m_last = None; self.m_seen = None
        try:
            c = _json.loads(MCU_CONFIG.read_text(encoding='utf8'))
            if isinstance(c, dict) and c.get('mode') == 'mcu':
                self.mcu = dict(win_s=float(c.get('win_s', WIN_S)), stale_s=float(c.get('stale_s', .45)))
        except Exception: self.mcu = None

    def _mcu_add(self, now, vehicle, b):
        ms = (vehicle or {}).get('ms')
        if isinstance(ms, bool) or not isinstance(ms, (int, float)): return
        ms = float(ms)
        if self.m_last is not None and ms == self.m_last: return
        if self.m_last is not None and (ms < self.m_last or ms - self.m_last > 2000.): self.mh = []   # MCU restart / long gap
        self.m_last = ms; self.m_seen = now; t = ms / 1000.
        self.mh.append((t, float(b.get('forward', 0)) - float(b.get('reverse', 0))))
        self.mh = [x for x in self.mh if t - x[0] <= 1.]

    def _mcu_rate(self):
        """None = not usable (old rate is used)."""
        if len(self.mh) < 2 or self.m_seen is None or not self.hist: return None
        if self.hist[-1][0] - self.m_seen > self.mcu['stale_s']: return None
        t1, c1 = self.mh[-1]; old = [x for x in self.mh if t1 - x[0] >= self.mcu['win_s']]
        t0, c0 = old[-1] if old else self.mh[0]
        return None if t1 - t0 < .05 else abs(c1 - c0) / (t1 - t0)

    def update(self, now, vehicle):
        b = (vehicle or {}).get('enc_b') or {}
        if 'forward' not in b: return self.rate()
        self.hist.append((now, float(b.get('forward', 0)) - float(b.get('reverse', 0))))
        self.hist = [x for x in self.hist if now - x[0] <= 1.]
        if self.mcu is not None: self._mcu_add(now, vehicle, b)   # speed-meter-mcu-r1
        return self.rate()

    def rate(self):
        if self.mcu is not None:   # speed-meter-mcu-r1
            r = self._mcu_rate()
            if r is not None: return r
        if len(self.hist) < 2: return None
        t1, c1 = self.hist[-1]; old = [x for x in self.hist if t1 - x[0] >= WIN_S]
        t0, c0 = old[-1] if old else self.hist[0]
        return None if t1 - t0 < .05 else abs(c1 - c0) / (t1 - t0)

    def k(self, speed, right=False):
        """Scale for the steering offset; it may fall at once but rises by at most K_RISE per frame."""
        ref, kmin = (REF_SPEED_RIGHT, K_MIN_RIGHT) if right else (REF_SPEED, K_MIN)   # downhill-right-r1
        k = 1. if speed is None or speed <= ref else max(kmin, ref / speed)
        if self.k_last is not None: k = min(k, self.k_last + _RI.step(K_RISE))
        self.k_last = k
        return k
