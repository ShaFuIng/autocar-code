"""Level bottom of the downhill: no left switch onto the neighbouring track (bottom-left-r1).

User 2026-11-05 (raw-20261105c 9ce62fb8, after bottom-curve-r1): on the downhill the car may cross lines, it only has to turn
right into the big flat right curve. At the level bottom the junction switch took its corridor path to the left
(STRONG_CORRIDOR_DISAGREES, error -0.61 while the baseline said +1.0), the baseline followed that line 0.5 s later, and after the
downhill-bottom hold the PID steered 880-1060 into the left track. Same left corridor takes at the bottom: 4459e292 / 3b8b5596
(failed), 7cc8c7af / 0fb475b7 (survived). bottom-curve-20261105/right_only.json: the laps that made the right curve steered
right there, or a little left (lowest about centre -210: 0fb4 1181, 4932 1186; 7cc8 961 only while the corridor had it).
Window: inside live_pid's 4 s down_tail, after this slope reached DOWN_STEEP, pitch back at or above LEVEL_PITCH.
 1. gate(): a corridor path that points left (error < 0) and more than LEFT_GAP left of the baseline path is replaced by the
    baseline; without a baseline at the sample row a corridor more than LEFT_GAP left of centre makes the frame invalid
    (lost-path hold) instead of being followed (nearly straight ones, e.g. 17eb58a0 -0.00, 0bf4f209 -0.05, are kept).
    A corridor to the right, or one agreeing with the baseline, is kept.
 2. floor(): the sent command is not further left than centre - LEFT_FLOOR_US. Right is not limited.
Experiment values from these records; not a safety limit, the closed-loop effect is not verified.
"""
REVISION = 'bottom-left-r1'
LEVEL_PITCH, LEFT_GAP, LEFT_FLOOR_US = -2., .3, 200.


def in_window(down_tail, down_steep, pitch):
    return bool(down_tail) and bool(down_steep) and pitch is not None and pitch >= LEVEL_PITCH


def _r(v):
    return None if v is None else round(float(v), 3)


def gate(result, err):
    """err(points) -> image error at the sample row or None. Returns (action, info); action is 'BASELINE', 'INVALID' or None."""
    j = (result.get('path_diagnostic') or {}).get('junction') or {}
    if not result.get('valid') or result.get('selected_source') is not None or not str(j.get('source') or '').startswith('corridor'):
        return None, None
    ce = err(result.get('path') or [])
    bp = j.get('baseline_path') or []
    be = err(bp) if len(bp) >= 2 else None
    info = dict(revision=REVISION, junction_source=j.get('source'), corridor_e=_r(ce), baseline_e=_r(be), baseline_points=len(bp))
    if ce is None or ce >= 0: return None, dict(info, action='KEEP')
    if be is not None and be - ce <= LEFT_GAP: return None, dict(info, action='KEEP')
    if be is not None: return 'BASELINE', dict(info, action='BASELINE')
    return ('INVALID', dict(info, action='INVALID')) if ce < -LEFT_GAP else (None, dict(info, action='KEEP'))


def floor(steer_us, center_us):
    lo = float(center_us) - LEFT_FLOOR_US
    return (lo, True) if float(steer_us) < lo else (float(steer_us), False)
