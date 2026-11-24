"""Gain-schedule switch (gain-schedule-r4).

The gain schedule in live_pid (r1 2026-11-02, r2 reference 15, r3 uphill exclusion) scales the PID steering offset by
ref_pwm / current PWM above ref_pwm. It was tuned for the old linkage and kp 180. User 2026-11-06 (new linkage, kp 220, runs
82218bb3 / 9783d8cd): with the schedule the car at 18-21 PWM weaves wider than at a fixed 15 (|err| > 0.6 on 8-17 % of the
flat frames vs 1 %) and the speed never builds; on the tunnel approach the cut left the PID at 1181-1268 while the car sat at
error -0.5 for 3 s. The constants now come from gain_schedule.json (absent = the r3 behaviour): enabled, ref_pwm, k_min.
"""
import json
from pathlib import Path

REVISION = 'gain-schedule-r4'
CONFIG_FILE = Path(__file__).with_name('gain_schedule.json')
DEFAULTS = dict(enabled=True, ref_pwm=15., k_min=0.)


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


def k_for_pwm(c, pwm):
    """k for the current PWM, or None when the schedule is off / below the reference (no cut)."""
    if not c.get('enabled'): return None
    ref = float(c.get('ref_pwm') or 15.); pwm = float(pwm)
    if pwm <= ref: return None
    return max(float(c.get('k_min') or 0.), ref / pwm)
