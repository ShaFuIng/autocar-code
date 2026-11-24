"""Automatic steering centre between runs (auto-centre-r1, user 2026-11-10: "the program should work out the next centre itself").
At the end of a run the record-only centre estimator (gyro straight windows, EMA kept across runs, left / right
arrival averaged to cancel the steering play) gives the steering command at which the car goes straight. If it
differs from the saved shared centre, the saved centre moves towards it by at most max_step_us per run (saving
does not move the servo; the BMduino synchronises the new profile). At the next start the PID centre follows the
saved shared centre instead of refusing a stale workbench value. auto_centre.json enabled=false -> old behaviour.
Thresholds are experiment values.
"""
import json
from pathlib import Path
REVISION = 'auto-centre-r2'
CONFIG = Path(__file__).with_name('auto_centre.json')
DEFAULTS = dict(enabled=False, max_step_us=20., min_change_us=3., min_n=20, margin_us=200., min_run_s=30.)

def load():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c

def propose(summary, profile, run_s, c=None, collision=None):
    """(new_centre or None, info). r2: a run with a collision (wheel speed to ~0 at drive PWM) does not move the centre."""
    c = c or load(); info = dict(revision=REVISION, enabled=bool(c.get('enabled')))
    if not c.get('enabled'): return None, dict(info, reason='DISABLED')
    if collision: return None, dict(info, reason='COLLISION', collision=collision)
    if not summary or summary.get('centre_source') != 'left_right_mean' or summary.get('centre_us') is None: return None, dict(info, reason='NO_ESTIMATE')
    if min(summary['left']['n'], summary['right']['n']) < int(c['min_n']): return None, dict(info, reason='FEW_SAMPLES', left_n=summary['left']['n'], right_n=summary['right']['n'])
    if run_s is None or run_s < float(c['min_run_s']): return None, dict(info, reason='SHORT_RUN', run_s=run_s)
    cur = float(profile['center_us']); target = float(summary['centre_us']); d = target - cur
    info.update(saved_centre_us=cur, estimate_us=round(target, 1))
    if abs(d) < float(c['min_change_us']): return None, dict(info, reason='CLOSE_ENOUGH')
    step = max(-float(c['max_step_us']), min(float(c['max_step_us']), d))
    new = int(round(cur + step))
    lo, hi = float(profile['min_us']) + float(c['margin_us']), float(profile['max_us']) - float(c['margin_us'])
    if not lo <= new <= hi: return None, dict(info, reason='OUT_OF_RANGE', new_centre_us=new)
    return new, dict(info, reason='MOVED', new_centre_us=new, step_us=round(step, 1))
