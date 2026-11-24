"""Single-line width on sharp curves (corner-width-r1).

User 2026-11-03 (live a0e410ed / 3fd37b76): at the corner before the ramp only the outer left line is seen and the
car turned in too late and hung off the ramp. The single-line path is x_c(y) = x_L(y) + w(y) with w the horizontal
lane width of a straight lane at that row. When the outer line turns flat in the image (dx/dy -3.0 .. -3.6 instead of
-1.64 on straights) the same road is wider horizontally, so the centre sits too close to the line: the error fell to
0.11-0.24 exactly when the curve was sharpest (3fd37b76 74.7-75.6 s) and the steering went back to ~1430.

Horizontal width of a road of fixed perpendicular width grows as 1 / cos(line angle from vertical):
ratio = cos(atan |a_ref|) / cos(atan |a|), a_ref = median slope on straight double-line frames of those runs
(L -1.64, R 1.33). Applied only when |a| >= A_MIN, with STRENGTH 0.5 (half of the geometric ratio) and capped at
RATIO_MAX: at full strength every flat right curve with a flat outer line would also steer much harder
(3fd37b76 11.4 s error 0.12 -> 0.70), which may cut the inner line. Experiment values.
"""
import math
import numpy as np

REVISION = 'corner-width-r3'
A_REF = {'L': -1.64, 'R': 1.33}
A_MIN, STRENGTH, RATIO_MAX, MIN_POINTS = 2.6, .25, 2., 6   # r2 (user, live 902ad277): right curves crossed the line a little at 0.5; r3 (a591a0ec): still a little at 0.35
SOURCE = {'left+width': 'L', 'right+width': 'R'}


def widen(result):
    """Returns (path, info) with the single-line centre moved away from the line, or None."""
    le = ((result.get('path_diagnostic') or {}).get('line_estimate') or {})
    src = set((le.get('sources') or {}).keys())
    if len(src) != 1 or next(iter(src)) not in SOURCE: return None
    side = SOURCE[next(iter(src))]
    B = (((le.get('offscreen_center') or {}).get('current_boundaries') or {}).get(side)) or []
    path = result.get('path') or []
    if len(B) < MIN_POINTS or not path: return None
    P = np.array(B, float); a = float(np.polyfit(P[:, 1], P[:, 0], 1)[0])
    if abs(a) < A_MIN or (side == 'L') != (a < 0): return None
    geo = math.cos(math.atan(abs(A_REF[side]))) / math.cos(math.atan(abs(a)))
    k = min(RATIO_MAX, 1. + STRENGTH * (geo - 1.))
    bx = {int(round(y)): float(x) for x, y in B}
    out = [[bx[int(round(p[1]))] + (float(p[0]) - bx[int(round(p[1]))]) * k, float(p[1])] for p in path if int(round(p[1])) in bx]
    if len(out) < max(3, len(path) // 2): return None
    return out, dict(revision=REVISION, side=side, slope=round(a, 2), ratio=round(k, 3), points=len(out))
