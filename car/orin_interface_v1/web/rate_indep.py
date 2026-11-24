"""rate-indep-r1 (pipeline2-20261116): frame-count / per-frame parameters of the control side expressed in time.

The control layers were tuned at ~9.5 frames/s (nominal frame interval DT0 = 0.105 s). With rate_indep.json {"enabled": true}
every converted site uses a per-frame WEIGHT w = (capture interval of this frame) / DT0 instead of 1:
  counters      n += 1           ->  n += w            test  n >= N  ->  n >= N - 0.5   (thr)
  step limits   max change md    ->  md * w           (per-frame limit -> the same limit per second)
  EMA           a                ->  1 - (1 - a) ** w (same time constant)
  last-K lists  last K entries   ->  newest entries with cumulative weight <= K + 0.5
w is clipped to [0, w_cap] (default 2: after a gap a counter / step does not jump by more than two nominal frames).
At exactly nominal intervals w == 1 and every converted site gives the same result as the frame-count code; at other intervals
the counts follow time (see SUMMARY_FOR_REVIEW.txt). File absent / enabled false -> ON False: w() == 1 (int), thr(n) == n,
alpha(a) == a, step(md) == md -> every site is the original expression (bit-identical).
live_pid calls load() at each run start and frame(captured) once per processed frame."""
import json, math
from pathlib import Path

REVISION = 'rate-indep-r1'
ON = False
DT0 = .105
W_CAP = 2.
QUANTUM = .25
DT0_OPEN = .15   # upper section (dash route OPEN): today ~0.15 s per frame (dash_route.try_path on the main thread), its layers were tuned there
OPEN = False
FRAME_DT = None
_prev_cap = None


def load(path=None):
    """read web/rate_indep.json; returns the start-record info dict (or None when off). Never raises."""
    global ON, DT0, W_CAP, QUANTUM, DT0_OPEN, OPEN, FRAME_DT, _prev_cap
    ON = False; DT0 = .105; W_CAP = 2.; QUANTUM = .25; DT0_OPEN = .15; OPEN = False; FRAME_DT = None; _prev_cap = None
    try:
        c = json.loads(Path(path or Path(__file__).with_name('rate_indep.json')).read_text(encoding='utf-8'))
        if not isinstance(c, dict) or c.get('enabled') is not True: return None
        dt0 = float(c.get('nominal_dt', .105)); wc = float(c.get('w_cap', 2.)); qu = float(c.get('w_quantum', .25)); do = float(c.get('nominal_dt_open', .15))
        if not (.02 <= dt0 <= .3 and .02 <= do <= .3 and math.isfinite(dt0) and 1. <= wc <= 5. and 0. <= qu <= .5): return None
        ON = True; DT0 = dt0; W_CAP = wc; QUANTUM = qu; DT0_OPEN = do
        return dict(revision=REVISION, nominal_dt=DT0, nominal_dt_open=DT0_OPEN, w_cap=W_CAP, w_quantum=QUANTUM)
    except Exception:
        return None


def frame(captured, section_open=False):
    """once per processed frame (valid or not): capture interval to the previous processed frame of this run; section_open = the
    previous frame was in the dash-route OPEN window (upper section: nominal_dt_open)."""
    global FRAME_DT, _prev_cap, OPEN
    if not ON: return
    OPEN = bool(section_open)
    try:
        c = float(captured)
        FRAME_DT = None if _prev_cap is None or c <= _prev_cap else c - _prev_cap
        _prev_cap = c
    except Exception:
        FRAME_DT = None


def w():
    if not ON: return 1
    if FRAME_DT is None: return 1.
    r = FRAME_DT / dt0()
    if QUANTUM > 0: r = max(QUANTUM, round(r / QUANTUM) * QUANTUM)   # w_quantum: today's jitter (0.09-0.12 s) stays w = 1; 0 = continuous
    return max(0., min(W_CAP, r))


def dt0():
    return DT0_OPEN if OPEN else DT0


def thr(n):
    return n if not ON else float(n) - .5


def alpha(a):
    if not ON: return a
    ww = w()
    return a if ww == 1 else 1. - (1. - float(a)) ** ww   # w == 1 (nominal interval) -> the configured value itself (bit-identical)


def step(md):
    if not ON: return md
    return float(md) * w()


def tail_by_weight(items, weights, k):
    """newest entries whose cumulative weight (newest first) is <= k + 0.5; with all weights 1 this is items[-k:]."""
    out = []; s = 0.
    for x, ww in zip(reversed(items), reversed(weights)):
        s += ww
        if s > k + .5 and out: break
        out.append(x)
    return out[::-1]


def sync(items, weights):
    """weights list matching items (lists reset elsewhere -> missing weights count as 1)."""
    weights = list(weights or [])
    if len(weights) > len(items): weights = weights[len(weights) - len(items):]
    if len(weights) < len(items): weights = [1.] * (len(items) - len(weights)) + weights
    return weights


def trim(items, weights, k):
    """(items, weights) cut to the newest entries of cumulative weight <= k + 0.5 (all weights 1 -> [-k:])."""
    t = tail_by_weight(items, weights, k); n = len(t)
    return list(items[len(items) - n:]), list(weights[len(weights) - n:])
