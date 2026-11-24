"""cmd-comp-r1: subtract the steering already sent but not yet acted on from the camera command (user 2026-11-07 06:20).

The lateral loop has ~1 s between a steering command and the camera seeing its effect, so on a straight the PID keeps adding
corrections for an error that the commands already in flight will remove: a stable S weave (period 3-6 s, +-0.45) on every
straight tonight. Offline on five runs (a83a, bd78, a3a7, e624, 9ac5, flat straight frames): e - k * sum(u dt) over the last
window_s seconds predicts the error 0.6 / 0.8 / 1.0 s later better than the error itself (mean corr 0.67/0.51/0.32 ->
0.76/0.62/0.45 at window 1.2 s, k 0.001, RMSE equal or lower); k 0.002 over-corrects. Applied as a steering offset after the
PID and before the inner yaw loop: steer -= kp * k * sum(u dt), capped. u = the final steering actually sent (offset from the
centre), recorded by live_pid at each send. Only on flat straights (the caller gates: route WAIT_CLIMB, |pitch| <= 3, no mapped
curve, |gz| < gz_max, no lane/dash layers). Experiment values; closed loop not verified."""
import json, math
from pathlib import Path

REVISION = 'cmd-comp-r1'
CONFIG_FILE = Path(__file__).with_name('cmd_comp.json')
DEFAULTS = dict(enabled=False, window_s=1.2, k_per_us_s=.001, cap_us=120., gz_max=15., min_frames=3)


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


class CmdComp:
    def __init__(self, config=None):
        self.c = dict(DEFAULTS, **config) if config is not None else load_config(); self.hist = []   # (t, sent offset us)

    def sent(self, now, offset_us):
        """Record the steering actually sent (offset from the centre, us)."""
        if offset_us is None or not math.isfinite(float(offset_us)): return
        self.hist.append((float(now), float(offset_us))); w = float(self.c['window_s']) + 1.
        while self.hist and now - self.hist[0][0] > w: self.hist.pop(0)

    def pending(self, now):
        """sum(u dt) over the window, us*s; the last entry is held until now (capped 0.3 s per step)."""
        w = float(self.c['window_s']); h = [x for x in self.hist if now - x[0] <= w]
        if len(h) < int(self.c['min_frames']): return None
        acc = 0.
        for i, (t, u) in enumerate(h):
            t1 = h[i + 1][0] if i + 1 < len(h) else now; acc += u * max(0., min(.3, t1 - t))
        return acc

    def step(self, now, steer, center, lo, hi, kp, gz, straight=True):
        """Returns (steer, info). info None when disabled."""
        c = self.c
        if not c.get('enabled'): return steer, None
        if not straight: return steer, dict(revision=REVISION, reason='NOT_STRAIGHT')
        if gz is None or not math.isfinite(float(gz)) or abs(float(gz)) >= float(c['gz_max']): return steer, dict(revision=REVISION, reason='ROTATING')
        acc = self.pending(now)
        if acc is None: return steer, dict(revision=REVISION, reason='NO_HISTORY')
        cap = float(c['cap_us']); comp = max(-cap, min(cap, -float(kp) * float(c['k_per_us_s']) * acc))
        out = max(float(lo), min(float(hi), float(steer) + comp))
        return out, dict(revision=REVISION, reason='OK', pending_us_s=round(acc, 1), comp_us=round(comp, 1), capped=abs(comp) >= cap - 1e-9, pid_steer_us=round(float(steer), 1), steer_us=round(out, 1))
