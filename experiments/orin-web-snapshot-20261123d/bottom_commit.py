"""Level bottom of the downhill: keep turning right once the right turn has started (bottom-commit-r1).

User 2026-11-05 (raw-20261105d adc646d1, with bottom-left-r1): the car reached the flat bottom, steered 1750 and turned
right ~41 deg in 2 s (gyro -20..-31 deg/s), then the first frame after a 0.4 s path loss had error +1.0 -> +0.08, D -103 gave
1240, and the camera saw only the curve's line at the right edge, taken as the lane's right edge -> target straight ahead,
1340-1540 (gyro ~-2 deg/s), out on the outside of the right curve. The laps that made the curve turned 58-83 deg right within
6 s of the bottom (bottom-left-20261105/yaw_after_bottom.json), several with only 1430-1510 us in this phase, so a fixed
right minimum (+300 tried first) would have changed most of their frames; they unwound the turn slowly (a 200 us/s limit
changes them by at most ~105 us, mostly < 70), adc646d1 dropped 1690 -> 1240 in one frame.
Window: starts on the first frame with pitch >= LEVEL_PITCH after a pitch < STEEP_PITCH; yaw (gyro, right negative) is
summed from there. Active while the car has turned at least START_DEG right and less than DONE_DEG; ends for this slope at
DONE_DEG, after MAX_S, or on the slope again. A car turning left (7cc8c7af / 9ce62fb8) never activates it.
While active every sent command (valid, PID hold, downhill hold, lost heading, heading blind) moves towards the left by at
most UNWIND_US_S per second from the previous sent one, and is never left of centre. Right is not limited.
Experiment values from these records; not a safety limit, the closed-loop effect is not verified.

r2 (user 2026-11-05, raw-20261105g 6b9b724b second slope): at the bottom only the outer line of curve 11 was in view; the
PID steered 1617-1750 for 0.5 s, then the fixed-width target slid to the centre (error +0.5 -> 0) and D -6 .. -157 took the
command to 1190 while the car had turned ~15 deg. The window activated only at 20.6 deg, when the command was already 1190, so
its lower bound was the centre (1390); after the path loss the holds repeated 1390 and the car went straight out of 11.
Now the window is also armed by the command: the first command >= centre + ARM_US inside the window activates it at once (the
20 deg yaw condition stays). Open loop (upper-lateral-20261105/commit_start_sim.py): this bottom up to +388 us, the laps that
made curve 11 mostly as r1. live_pid no longer restarts the window on the lap step at WAIT_CLIMB (it comes ~3 s after the
bottom, during curve 11). Experiment values.
"""
import json, math
from pathlib import Path
CONFIG_FILE = Path(__file__).with_name('bottom_commit.json')   # bottom-commit-switch-r1 (user 2026-11-07, live 8c270b55)


def enabled():
    """bottom_commit.json {'enabled': false} turns the commit window off; file missing or key absent = on (r2)."""
    try: return bool(json.loads(CONFIG_FILE.read_text(encoding='utf8')).get('enabled', True))
    except (OSError, ValueError, AttributeError): return True

REVISION = 'bottom-commit-r2'
LEVEL_PITCH, STEEP_PITCH = -2., -8.
START_DEG, DONE_DEG, MAX_S, UNWIND_US_S = 20., 60., 6., 200.
ARM_US = 200.   # r2


class BottomCommit:
    def __init__(self):
        self.steep = False; self._clear()

    def _clear(self):
        self.t0 = None; self.turned = None; self.yaw_last = None; self.done = None; self.prev = None; self.prev_t = None; self.armed = False

    def update(self, now, pitch, yaw):
        """Call once per frame. Returns None outside the window, else an info dict with 'active'."""
        if pitch is not None and pitch < STEEP_PITCH: self.steep = True; self._clear(); return None
        if not self.steep: return None
        y = yaw if yaw is not None and math.isfinite(yaw) else None
        if self.t0 is None:
            if pitch is None or pitch < LEVEL_PITCH: return None
            self.t0 = now; self.turned = 0.; self.yaw_last = y
        elif y is not None:
            if self.yaw_last is not None: self.turned += (y - self.yaw_last + 180.) % 360. - 180.
            self.yaw_last = y
        since = now - self.t0
        if since > MAX_S: self.done = 'TIMEOUT'
        elif self.turned <= -DONE_DEG: self.done = 'TURNED'
        info = dict(revision=REVISION, since_s=round(since, 3), turned_deg=round(self.turned, 1), yaw_ok=y is not None)
        if self.done: self.steep = False; return dict(info, active=False, ended=self.done)
        return dict(info, active=self.turned <= -START_DEG or self.armed, armed=self.armed)


    def limit(self, now, steer_us, center_us, info):
        """Call with every command to be sent. Returns (steer, applied, lower bound)."""
        s = float(steer_us)
        if info and not info.get('ended') and not info.get('active') and s >= float(center_us) + ARM_US:   # r2: armed by the command
            self.armed = True; info['active'] = True; info['armed'] = True; info['armed_by_us'] = round(s, 1)
        if not info or not info.get('active'): self.prev = self.prev_t = None; return s, False, None
        lo = float(center_us)
        if self.prev is not None and now >= self.prev_t: lo = max(lo, self.prev - UNWIND_US_S * (now - self.prev_t))
        out = max(s, lo); self.prev, self.prev_t = out, now
        return out, out > s, round(lo, 1)
