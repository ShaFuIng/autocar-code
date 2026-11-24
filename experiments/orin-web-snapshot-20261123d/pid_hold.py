"""Short hold of the last valid PID command over invalid path frames (pid-hold-r1).

Until 2026-11-01 the live PID ended the run on the first invalid path frame;
the 0.4 s max_gap only limited the frame age. In the 12 latest live runs all 10
PID_INVALID_OR_STALE_PATH stops had exactly one invalid frame in the whole run
with the 5 frames before it valid (91a23044 33.06 s: a hand-back refused by
1 px; b3a817b4 / de4888e4: a baseline remnant 0.3 px short of the control row).

On an invalid (fresh) frame the last valid steering and PWM are sent again,
for at most HOLD_S after the last valid command; a valid frame inside that
time continues the run, otherwise the run ends as before. Stale frames
(age > max_gap), emergency stop, faults and the other stop reasons are
unchanged. HOLD_S reuses the existing 0.4 s frame-age limit; it is an
experiment value, not a verified safety limit.
"""
REVISION = 'pid-hold-r4'
HOLD_S = .4
# r2: live 8f2e7fbc stuttered in the dashed-line section: each hold was followed by a controller
# WARMUP frame (steering centre 1580, PWM 0) because invalid frames still stepped the controller
# with an empty path; the next hold then repeated PWM 0. live_pid no longer steps the controller on
# invalid run frames, a WARMUP right after a recent valid command repeats that command, and WARMUP
# commands are not recorded as the command to hold.


class PidHold:
    def __init__(self, hold_s=HOLD_S):
        self.hold_s = float(hold_s); self.last = None; self.last_t = None; self.frames = 0

    def valid(self, now, steer_us, pwm):
        self.last = dict(steer_us=float(steer_us), pwm=float(pwm)); self.last_t = now; self.frames = 0

    # r3: hold and heading-blind frames are commands too (sent()); after them the controller restarts
    # with WARMUP / INVALID_OR_STALE_TIME (live 69f00708 stopped at 60.22 s: valid path back after 0.8 s
    # of heading-blind frames, the controller's own 0.4 s time gap rejected it). last_sent() gives the
    # last dispatched command for that restart frame.
    def sent(self, now, steer_us, pwm):
        self.sent_cmd = dict(steer_us=float(steer_us), pwm=float(pwm)); self.sent_t = now
        h = getattr(self, 'hist', []); h.append((now, float(steer_us))); self.hist = [x for x in h if now - x[0] <= 1.]

    # r4 (live b9fa9cff 105.6-106.9 s): the downhill hold kept the last command 880 us (left lock) for 1.2 s;
    # median of the commands sent in the last span_s instead
    def median_recent(self, now, span_s=.5):
        xs = sorted(s for t, s in getattr(self, 'hist', []) if 0 <= now - t <= span_s)
        return None if not xs else xs[len(xs) // 2]

    def last_sent(self, now, max_s=.5):
        t = getattr(self, 'sent_t', None)
        if t is None or not 0 <= now - t <= max_s: return None
        return dict(self.sent_cmd)

    def recent(self, now):
        """The last valid command if it is at most HOLD_S old (no hold frame counted)."""
        if self.last is None or self.last_t is None or not 0 <= now - self.last_t <= self.hold_s + 1e-9: return None
        return dict(self.last)

    def invalid(self, now):
        """The command to keep, or None when the run has to stop."""
        if self.last is None or self.last_t is None or not 0 <= now - self.last_t <= self.hold_s + 1e-9: return None
        self.frames += 1
        return dict(self.last, revision=REVISION, hold_s=round(now - self.last_t, 3), hold_frames=self.frames)
