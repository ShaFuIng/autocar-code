"""start-bias-r1 (2026-11-22, user: the car always drifts right at the start): a short steering offset right after the start.

Measured on 17 short starts of 2026-11-22: standing on the start grid the camera error is +0.02..+0.15, so the first commands are
centre +7..+31 us; the straight-running point is about 8 us left of the centre (1467 vs 1475) and the linkage has ~130 us of play, so
the wheel is pushed to the right side of the play and stays there until the command is 50-60 us left of the centre. The heading goes
2-6 deg to the right in the first 1-1.5 s and the car reaches the right of the line (error -0.2..-0.3) before it comes back.

This layer adds bias_us (negative = left) to the final steering command of valid frames for hold_s seconds after the start hold
(start_centre) ended, then fades it to 0 over fade_s. It runs last, after every other valid-frame layer. web/start_bias.json is read
once per run start; absent / enabled not true / unusable values = None = the layer is not in the run (frame-identical to before).
Experiment values, closed loop not verified."""
import json
from pathlib import Path

REVISION = 'start-bias-r1'
CONFIG_FILE = Path(__file__).with_name('start_bias.json')


def _num(x): return isinstance(x, (int, float)) and not isinstance(x, bool) and x == x and abs(x) != float('inf')


def load_config(path=None):
    try: c = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
    except (OSError, ValueError): return None
    if not isinstance(c, dict) or c.get('enabled') is not True: return None
    b, h, f = c.get('bias_us'), c.get('hold_s'), c.get('fade_s', 0.)
    if not (_num(b) and _num(h) and _num(f)) or b == 0 or abs(b) > 100. or not 0. < h <= 3. or not 0. <= f <= 2.: return None
    return dict(bias_us=float(b), hold_s=float(h), fade_s=float(f))


def bias(c, elapsed_s):
    """(bias us, finished). elapsed_s = seconds since the start hold ended."""
    if elapsed_s < 0.: return 0., False
    h, f = c['hold_s'], c['fade_s']
    if elapsed_s < h: return c['bias_us'], False
    if f <= 0. or elapsed_s >= h + f: return 0., True
    return c['bias_us'] * (1. - (elapsed_s - h) / f), False


def apply(c, elapsed_s, steer_us, min_us, max_us):
    """(steer, info or None, finished)."""
    b, done = bias(c, elapsed_s)
    if b == 0.: return float(steer_us), None, done
    st = max(float(min_us), min(float(max_us), float(steer_us) + b))
    return st, dict(revision=REVISION, bias_us=round(b, 1), elapsed_s=round(elapsed_s, 2), pid_steer_us=round(float(steer_us), 1), steer_us=round(st, 1)), done
