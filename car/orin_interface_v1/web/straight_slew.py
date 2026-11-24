"""Straight-only steering slew + cap (straight-slew-r1).

User 2026-11-07 (runs 9904, 74e7, 8eb8): on flat straights the ~1 s lateral loop is marginally stable; once kicked by a curve-exit
overshoot a ~2 s limit cycle builds up (e +-0.6..1.0, gz +-35..55 dps, final steering +-300..594 us, the inner loop saturating
+-120 each half cycle) and does not decay. Sustaining a 2 s cycle of amplitude A needs a slew of ~3.14*A us/s; the servo got up to
~940 us/s on the weave (the run steer_rate 800 us/s is applied later in manual_drive and does not bound it). The calm run a83a stayed
within |out| P90 226 us. On frames the caller flags as straight the final steering is rate-limited (rate_us_s) and capped to
centre +- cap_us; curve frames pass through unchanged and only reset the internal position, so curves keep full authority.
Straight gate (coordinator 2026-11-07): the instantaneous |gz| < 15 gate excluded the weave itself (gz +-35..55, 9904 only 17/122
frames); the caller now uses the 2 s mean of gz (mean_gz, >= 5 samples): the weave cancels to ~0, real curves average -25..-35.
Experiment values; closed loop not verified."""
import json, math
from pathlib import Path

REVISION = 'straight-slew-r1'
CONFIG_FILE = Path(__file__).with_name('straight_slew.json')
DEFAULTS = dict(enabled=False, rate_us_s=300., cap_us=250., mean_gz_max=12., hold_s=.3, mean_s=2., mean_n=5)   # hold_s: longest dt one frame may use; mean_*: 2 s gz mean gate


def load_config(path=None):
    c = dict(DEFAULTS)
    try: c.update(json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


def mean_gz(buf, now, gz, c=None):
    """Append (now, gz) to buf (list kept by the caller), drop samples older than mean_s; mean of gz, None with < mean_n samples."""
    c = c or DEFAULTS; w = float(c.get('mean_s', 2.)); n = int(c.get('mean_n', 5))
    if gz is not None and math.isfinite(float(gz)): buf.append((float(now), float(gz)))
    while buf and buf[0][0] < float(now) - w: buf.pop(0)
    return sum(g for _, g in buf) / len(buf) if len(buf) >= n else None


class StraightSlew:
    def __init__(self, config=None):
        self.c = dict(DEFAULTS, **config) if config is not None else load_config()
        self.last = None; self.last_t = None

    def step(self, now, steer, center, lo, hi, straight):
        s0 = steer; steer = float(steer)
        if not self.c.get('enabled') or not straight:   # pass through, but remember where the servo really is
            self.last = steer; self.last_t = float(now)
            return s0, (None if not self.c.get('enabled') else dict(revision=REVISION, reason='CURVE'))
        rate = float(self.c['rate_us_s']); cap = float(self.c['cap_us']); prev = self.last; dt = None
        if prev is None or self.last_t is None: out = steer   # first frame: no slew, just initialise
        else:
            dt = max(0., min(float(self.c.get('hold_s') or .3), float(now) - self.last_t)); lim = rate * dt
            out = prev + max(-lim, min(lim, steer - prev))
        slewed = abs(out - steer) > 1e-9
        o2 = max(center - cap, min(center + cap, out)); capped = abs(o2 - out) > 1e-9
        out = max(float(lo), min(float(hi), o2))
        self.last = out; self.last_t = float(now)
        return out, dict(revision=REVISION, reason='OK', slewed=slewed, capped=capped, pid_steer_us=round(steer, 1), steer_us=round(out, 1),
                         rate_used_us_s=None if not dt or prev is None else round(abs(out - prev) / dt, 1))
