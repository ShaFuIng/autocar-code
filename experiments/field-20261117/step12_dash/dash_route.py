"""Post-crest dashed-line left turn for the live PID (dash-route-r1).

User 2026-11-01: after the ramp top the road opens up; at the white dashed
line the car must turn left; afterwards two lanes are separated by the dashed
line, either lane is fine and following the dashed line itself is acceptable;
the car should be straight before the next (down) ramp.

Route window (IMU, record-only logger -> imu_boost reader): opens after the
car was pitched up >= CLIMB_DEG for CLIMB_S and is level again (< LEVEL_DEG
for LEVEL_S); closes at the downhill (<= DOWN_DEG) or WINDOW_S after opening;
opens at most once per run. Outside the window nothing changes.

Inside the window, per camera frame (JPEG, 640x360): white = bright and
unsaturated below row Y_TOP; dash blocks = compact components; the dashed
line = >= 3 collinear blocks (RANSAC) with one block below NEAR_Y
(blue-turn-20261101/dash_bright.py r2: ACROSS -> ALONG sequence on three
hand-pushes). ACROSS (< ACROSS_DEG from horizontal) for ACROSS_FRAMES frames
-> TURN: path from the vehicle axis toward the row's left end. Otherwise
-> FOLLOW: path on the dashed line. The target x at the control row moves at
most MAX_SHIFT_PX_S (two dashed rows near the intersection). No dashes for
LOST_S -> release to the normal path. Experiment values, not safety limits.
"""
import numpy as np
import cv2
import json as _json
from pathlib import Path as _Path
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

# upper-turn-r1 (upper-20261115, offline candidate): optional dashed-turn timing, upper_turn.json next to this file, read when the
# DashRoute is built (run start). Missing / unreadable / enabled false -> {} -> dash-route-r27 behaviour exactly. Keys:
#  delay_slack {d0, a, b, v_min}: the r23 TURN_DELAYED delay is capped at max(0, (d0 - a - b * v) / v), v = wheel speed (m/s, set by
#    live_pid as self.v_mps). d0 = measured distance of the dashed line ahead of the front tyre at the TURN decision without delay
#    (0.93 m), a + b * v = distance the turn needs to end on the line at entry speed v (0.61 + 0.46 v, 29 passes of 11-14, geom.py).
#    At the r23 design speed (PWM 13, ~0.3 m/s) the cap is >= 0.6 s (no change); at 0.65-0.75 m/s it is 0-0.04 s.
#  along_min_turn: an ALONG dashed line (>= ACROSS_DEG) does not end the TURN while live_pid's align_ok is False (< 50 deg turned,
#    dash-align-min-r1); the TURN target is held (HOLD_LAST). 11-14: 5 of 29 TURNs ended at 24-34 deg on such a line and ALIGN
#    then ran at PWM 13 for 1.2-1.6 s.
TT_REVISION = 'upper-turn-r1'
# dash-pos-backup-r1 (dash-late-20261116, offline candidate): upper_turn.json key pos_backup {after_m, max_m, x_row, max_pitch_deg, min_lap}.
# Absent -> nothing below runs (try_path is the camera path only). When present: if the camera has not started the TURN and lap_pos
# (live_pid sets self.lp = last lap_pos state) is anchored at TOP, not lost, lap >= min_lap, after_m <= metres since TOP < max_m and
# |pitch_rel| < max_pitch_deg (route OPEN = after the climb, level), the TURN is started by position: same state as a camera TURN
# (mode TURN, turned True, target x_row at the control row = the median first camera-TURN x_row, 139 px). 0e6a lap 2 (11-16): the camera
# TURN came at TOP+4.17 m (clutter beyond the deck made RANSAC prefer a steep line through it, ALONG_BEFORE_TURN); 87 normal passes
# 11-14..16 decided at TOP+2.47..2.85 m. Experiment values.
PB_REVISION = 'dash-pos-backup-r1'

# dash-fast-r1 (dash-speed-20261117, offline candidate): same results, less Python. web/dash_fast.json {"enabled": true}, read when the
# DashRoute is built (run start, load_fast). File absent / unreadable / enabled not true -> _FAST False -> every function below is
# unused and dash-route-r27 runs line by line. When on, precompute() / detect() / side_lines() compute exactly the same values:
#  * the white mask and its connected components are computed once per frame (r27 did both twice: dash_blocks and side_lines);
#  * dash_blocks / the side-line row scan are numpy filters over the component table / the mask rows (same order, same floats);
#  * RANSAC: the random pairs depend only on (seed, number of points, iterations) -> drawn with the very same generator calls
#    (np.random.default_rng(seed).choice(n, 2, replace=False), in the same order) once per (seed, n, iters) on this machine and
#    cached; all candidate lines are then scored at once with the same element-wise IEEE operations; the chosen line is the FIRST
#    iteration with the largest inlier count among those r27 would accept (r27: "best is None or inl.sum() > best.sum()").
# Proof: dash-speed-20261117/proof (precompute outputs and DashRoute state machine identical on the recorded upper-deck frames).
FAST_REVISION = 'dash-fast-r1'
_FAST = False
_PAIRS = {}


def load_fast(path=None):
    global _FAST
    try:
        c = _json.loads(_Path(path or _Path(__file__).with_name('dash_fast.json')).read_text(encoding='utf8'))
        _FAST = isinstance(c, dict) and c.get('enabled') is True
    except Exception: _FAST = False
    return _FAST


def _pairs(seed, n, iters):
    k = (seed, n, iters); v = _PAIRS.get(k)
    if v is None:
        rng = np.random.default_rng(seed); I = np.empty(iters, np.intp); J = np.empty(iters, np.intp)
        for t in range(iters):
            i, j = rng.choice(n, 2, replace=False); I[t] = i; J[t] = j
        if len(_PAIRS) > 1024: _PAIRS.clear()
        v = _PAIRS[k] = (I, J)
    return v


def _blocks_cc(cc):
    n, lab, st, cen = cc
    x, w, h, a = st[:, 0], st[:, 2], st[:, 3], st[:, 4]
    sel = (MIN_A <= a) & (a <= MAX_A)
    sel &= ~((np.maximum(w, h) > .45 * 640) | ((w > 180) & (h > 60)) | (a / (w * h).astype(np.float64) < .35))
    sel &= ~((x <= 1) | (x + w >= 639)); sel[0] = False
    near = (w >= NEAR_W) & (w >= NEAR_AR * h)
    return [(float(cen[k][0]), float(cen[k][1]), float(near[k])) for k in np.flatnonzero(sel)]


def _ransac_fast(B, iters=200, tol=10., seed=0, near_shape=True):
    if len(B) < 3: return None
    P = np.array(B); I, J = _pairs(seed, len(P), iters); Pi, Pj = P[I], P[J]
    ok = ~(np.abs(Pi[:, 1] - Pj[:, 1]) < 4)
    with np.errstate(divide='ignore', invalid='ignore', over='ignore'):
        a = (Pj[:, 0] - Pi[:, 0]) / (Pj[:, 1] - Pi[:, 1]); b = Pi[:, 0] - a * Pi[:, 1]
        a2 = a[:, None]
        inl = np.abs(P[None, :, 0] - (a2 * P[None, :, 1] + b[:, None])) / np.sqrt(1 + a2 * a2) < tol * (1 + P[None, :, 1] / 360)
    nr = inl & (P[None, :, 1] >= NEAR_Y)
    if near_shape: nr &= (P[None, :, 2] > 0)
    cnt = inl.sum(1); q = ok & (cnt >= 3) & nr.any(1)
    if not q.any(): return None
    best = inl[int(np.argmax(np.where(q, cnt, -1)))]
    a, b = np.polyfit(P[best, 1], P[best, 0], 1)
    return dict(a=float(a), b=float(b), n=int(best.sum()), xs=P[best, 0].tolist(), ys=P[best, 1].tolist(), near=int((P[best, 1] >= NEAR_Y).sum()))


def _detect_cc(cc, near_shape=True):
    L = _ransac_fast(_blocks_cc(cc), near_shape=near_shape)
    if L is None: return None
    L['angle'] = 90 - abs(float(np.degrees(np.arctan(L['a']))))
    return L


_SIDE_YS = np.arange(120, 359, 3)


def _side_lines_cc(cc):
    n, lab, st, _ = cc
    w, h, a = st[:, 2], st[:, 3], st[:, 4]
    keep = (np.maximum(w, h) >= 120) & (a >= 400) & (a / (w * h).astype(np.float64) < .5); keep[0] = False
    R = keep[lab[_SIDE_YS]]; out = {}
    for side in ('left', 'right'):
        if side == 'left': S = R[:, :300]; has = S.any(1); xs = 299 - np.argmax(S[:, ::-1], 1)
        else: S = R[:, 341:]; has = S.any(1); xs = 341 + np.argmax(S, 1)
        if int(has.sum()) < SIDE_MIN_ROWS: continue
        P = np.column_stack((_SIDE_YS[has].astype(np.float64), xs[has].astype(np.float64)))
        I, J = _pairs(1, len(P), 150); Pi, Pj = P[I], P[J]
        ok = ~(np.abs(Pi[:, 0] - Pj[:, 0]) < 20)
        if not ok.any(): continue
        with np.errstate(divide='ignore', invalid='ignore', over='ignore'):
            aa = (Pj[:, 1] - Pi[:, 1]) / (Pj[:, 0] - Pi[:, 0]); bb = Pi[:, 1] - aa * Pi[:, 0]
            inl = np.abs(P[None, :, 1] - (aa[:, None] * P[None, :, 0] + bb[:, None])) < 5
        best = inl[int(np.argmax(np.where(ok, inl.sum(1), -1)))]
        if best.sum() < SIDE_MIN_ROWS or P[best, 0].max() < SIDE_NEAR_Y: continue
        a_, b_ = np.polyfit(P[best, 0], P[best, 1], 1); out[side] = dict(a=round(float(a_), 5), b=round(float(b_), 2), n=int(best.sum()))
    return out


def _precompute_fast(jpeg):
    out = dict(jpeg=jpeg, img=None, L={}, sides=None, sides_ok=False, fast=FAST_REVISION)
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None: return out
    img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA); out['img'] = img
    cc = cv2.connectedComponentsWithStats(white(img), 8)
    B = _blocks_cc(cc)
    for ns in (True, False):
        L = _ransac_fast(B, near_shape=ns)
        if L is not None: L['angle'] = 90 - abs(float(np.degrees(np.arctan(L['a']))))
        out['L'][ns] = L
    try: out['sides'] = _side_lines_cc(cc); out['sides_ok'] = True
    except Exception: pass
    return out


def load_turn_timing(path=None):
    try:
        c = _json.loads(_Path(path or _Path(__file__).with_name('upper_turn.json')).read_text(encoding='utf8'))
        return c if isinstance(c, dict) and c.get('enabled') else {}
    except Exception: return {}

REVISION = 'dash-route-r27'
CLIMB_DEG, CLIMB_S, LEVEL_DEG, LEVEL_S, DOWN_DEG, WINDOW_S = 8., 2., 3., .5, -5., 60.
V_MIN, S_MAX, Y_TOP, NEAR_Y, MIN_A, MAX_A = 185, 60, 100, 175, 25, 9000   # r20 (user, live 4de77f74 / 1c87c960 at PWM 13): 150 turned a little early -> 175
# r19 (user 2026-11-03: the left turn starts too late, live 373328c8 / eb7f7e3d): the dashed row was found as 7-11 blocks
# 1.5-2 s before the turn but all above row 200, so no line was reported. NEAR_Y 200 -> 150: video replay of 8 runs starts the
# turn 0.4-1.7 s earlier; 130 started b80b77e1 3.4 s early (likely false). test_near_y.py. Experiment value.
NEAR_BLOCKS = 2   # live 95d42be8 46.6 s: background blocks + one side-rail piece formed a false row
# r22 (live bb013bf3 lap 2, 146.9 s): the only block below NEAR_Y was a small white piece beyond the right rail (w 20, h 16,
# area 128) and the far row (y 115-138) started the turn early. Real near dashes in 5 runs: w >= 73, area >= 599, w/h >= 2.6.
# A block counts as near only when w >= NEAR_W and w/h >= NEAR_AR. Experiment values between the two groups.
NEAR_W, NEAR_AR = 45, 2.
# r24 (user 2026-11-04, live 480ddbc3 / 1b20e7b0 / ff9a6385): once the dashed line has been followed, the car drives ALONG the
# dashes and the nearest one looks tall (137x100, 122x108, 115x69 px at 20.9-24 s of 480ddbc3), so the r22 shape rule found no
# near block and the clearly visible line was reported as NO_DASH_LINE for 4-5 s; the lane segment fell back to the heading
# alone and the car crossed the road (480ddbc3) or stayed in the left lane past the downhill entrance (ff9a6385). The shape rule
# is meant for the across row that starts the turn; after the first FOLLOW frame any block below NEAR_Y counts as near again
# (as before r22). Video replay of the straights of 12 runs (raw-20261104e/f/g nearshape.py): line found 12 -> 32 frames in
# 480ddbc3 (all 20 new ones within 10 deg of the heading reference), 13 -> 46 in 1b20e7b0, 10 -> 50 in ff9a6385; new fits pointing
# elsewhere were mostly at the slope edge (floor circuit) and meet the existing direction gate (expect_rel). Turn start and the
# TURN itself keep the r22 rule.
ACROSS_DEG, ACROSS_FRAMES, LOST_S, MAX_SHIFT_PX_S = 35., 3, .5, 300.
# r2: start the turn on a nearly horizontal row: live 899318f1 never turned (true row 6-7 deg,
# 4-7 blocks, but < 2 blocks below NEAR_Y); false rows were tilted (95d42be8 46.2 s 19 deg,
# 899318f1 51.2 s 26-28 deg, 3-4 blocks).
START_DEG, START_N, START_FRAMES = 15., 4, 2
# r14 (live 902ad277, 2026-11-03): the car reached the crest at an angle and the real dashed row was tilted 18-26 deg with
# 6 blocks (95.08-95.32 s), so the turn never started (ACROSS_NOT_START_ROW); rows under 15 deg had only 3 blocks or came
# between along-line frames. A tilted row also starts the turn when it has START_N2 blocks; the known false rows were
# 19 / 26-28 deg with 3-4 blocks (95d42be8, 899318f1). Experiment values.
START_DEG2, START_N2 = 26., 5
# r16 (live 470fb7e7, crest start): the row was seen at 7.94 s (10.0 deg, 4 blocks) and 10.41 s (11.7 deg, 5 blocks) but the
# next frame had 3 blocks (13.3 / 13.6 deg) and reset the count, so the turn never started. After one start-quality frame
# (ACROSS_PENDING) the next across frame needs only CONT_N blocks below START_DEG2. Experiment value.
CONT_N = 3
# r26 (user 2026-11-07, live 69f80d3f 93.4-94.8 s): the row was seen with 3 blocks at 7-12 deg for 1.4 s (part of it out of view with
# the car right of the lane) and never started the turn. A row with START3_N blocks below START3_DEG in START3_FRAMES frames, each
# within START3_GAP_S of the previous, passes the start test; the known false rows were tilted 19-28 deg. Experiment values.
START3_N, START3_DEG, START3_FRAMES, START3_GAP_S = 3, 13., 3, .3   # r27 (user 2026-11-07 e624, pre-turn PWM 18 / 0.40 m/s: the row was seen in only 3 frames below 13 deg -> no turn); replay of 21 runs: 3 frames triggers within 0.3 s of every real start, no early trigger
# r17 (live 037eb84e, crest start): a clear row at 6.60 s (9.0 deg, 9 blocks) was followed by a frame without any line and
# the turn waited for the next row, 1.1 s later. A row below START_DEG with at least STRONG_N blocks starts the turn on its
# own. The known false rows had 3-4 blocks. Experiment value.
STRONG_N = 6
# r3: live 0e5ab60e turned at 53.69 s and followed the dashed line from 55.1 s, then stopped at
# 58.68 s NO_SUPPORTED_FIXED_ROW: the FOLLOW path ended at the lowest dash block (row 202, control
# row 215.7). The FOLLOW path now runs along the fitted line down to the image bottom (like TURN).
# Also 57.72 / 57.80 / 58.13 / 58.38 s: a crossing dash row (19-21 deg) switched the steady FOLLOW
# back to TURN (target to the row's left end). After FOLLOW has lasted FOLLOW_LOCK_S, crossing rows
# are ignored (the last target is held as for a missing line). Experiment value.
FOLLOW_LOCK_S = 1.0
# r6: live 29b9fd2b went back to TURN at 60.35 / 61.15 / 61.40-62.04 s (crossing rows inside the first
# second of FOLLOW): turn floor 1132 us for 0.7 s, heading 30 deg too far left. Once FOLLOW was
# reached, crossing rows never start TURN again (the heading hold finishes the turn).
# r4: live f79e17c9 turned too slowly (user): during TURN the PID only reached 1260 us once and
# then 1350-1390 (target = the row's left end, x 82-142 -> error about -0.7; integral pinned at
# +40 us from the flat-ground offset), the dashes left the image and the run stopped at 56.87 s.
# The successful turn in 0e5ab60e reached 1124-1137 us. While TURN is active (OK or HOLD_LAST)
# the steering target is the more-left of the PID value and TURN_FRAC of the left range
# (centre 1580, min 1020 -> 1132 us). The saved steering rate limit still applies. Experiment value.
TURN_FRAC = .75   # r18 (user 2026-11-03: the dashed turn is too hard): 0.8 -> 0.95 (centre 1390, min 880 -> 905 us); dash-turn-stop-r1
# r25 (user 2026-11-06, live 6fc436d5): with the new linkage min_us is 750 and TURN_FRAC 1.0 steered to the full lock; at PWM 13 the
# turn cut far inside, the car then followed the dashes along the platform edge and drove off it. 0.75 x (1390 - 750) = 480 ->
# 910 us, the geometry of the 905 us turn before the linkage change. Experiment value.
# (user 2026-11-05): the left limit is now the measured stop (1000), 0.95 gave 1020 (short of the stop, radius 80 -> 91 cm) -> 1.0 = 1000
# r5: live cd6f0c63 lost the dashed line at 59.37 s, held the stale target x 511 for 1.3 s (steering
# right, gyro -7 deg) and then never followed the dashes again (ALONG_BEFORE_TURN needs a new TURN);
# the normal path in the two-lane section drifted 36 deg right before the downhill. Now: the turn
# happens once per window (self.turned); after it the dashed line is picked up again whenever seen
# (FOLLOW), crossing rows never start a second turn, and the stale hold is LOST_S 0.5 s (was 1.5).

# r7 (user 2026-11-02): (1) crest start for quick downhill tests: the route window is open from the run
# start (path option camera-pair-v2-junction-crest); (2) in that option the two-lane section follows the
# LEFT lane: centre between the dashed line and the solid line to its left (long thin white components,
# at least LEFT_MARGIN px left of the dashes); without a visible left line the target is the dashed line
# shifted left by lane_k * (y - HORIZON_Y) (lane_k learnt from the last frame with both lines, default
# LANE_K from c1f62feb 61.0 s). Experiment values.
HORIZON_Y, LANE_K, LANE_K_RANGE, LEFT_ROWS = 60., .85, (.4, 1.5), 12
# r8 (user 2026-11-02, live 3917715e): pulling the car into the left lane needed 1187-1279 us right after the
# turn and swung back to 1948 us; the learnt lane_k jumped 0.4 <-> 1.5. The crest option now keeps the car
# between the dashed line and the right lane (its natural exit side): target = dashes + RIGHT_K * (y - HORIZON_Y),
# a fixed quarter lane (half of LANE_K), and the target moves at most LANE_SHIFT_PX_S (was MAX_SHIFT_PX_S).
RIGHT_K, LANE_SHIFT_PX_S = .42, 120.
# r9 (user 2026-11-02, live 2155b25b): keep to the right lane, a little left of its centre: RIGHT_K .42 -> .65
# (right-lane centre would be LANE_K .85). In 2155b25b the quarter-lane target (x 238-298 at the control row)
# was still left of the car after the turn.
RIGHT_K = .65
# r12 (user 2026-11-02, live 310b6ae8): .65 put the target 100-190 px right of centre and took the car to the
# right platform edge; .42 (r8) was still left of the car -> .5.
RIGHT_K = .5
# r11 (live 05c8ec88): the 120 px/s target limit left the target 160 px behind the dashes (17.9 s) -> back to
# MAX_SHIFT_PX_S; near the left edge the solid edge line was taken as the dashed line (20.1-20.5 s, direction
# 13-17 deg off the reference). A dashed line whose direction differs from the heading reference by more than
# DIR_GATE_DEG is rejected (expect_rel = yaw - reference, set each frame by live_pid from heading-hold; None ->
# no gate). Direction from the line at HORIZON_Y against the straight-line vanishing point (VP_X, F_PX).
LANE_SHIFT_PX_S, DIR_GATE_DEG, VP_X, F_PX = 300., 10., 333., 493.


def solid_left(img, L):
    m = white(img)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    keep = np.zeros(n, bool)
    for k in range(1, n):
        x, y, w, h, a = st[k]
        if max(w, h) >= 120 and a >= 400 and a / float(w * h) < .5: keep[k] = True
    big = keep[lab]; pts = []
    for y in range(110, 359, 3):
        xd = L['a'] * y + L['b']
        xs = np.flatnonzero(big[y, :max(0, int(xd - max(25., .35 * (y - 80))))])
        if len(xs): pts.append((y, float(xs.max())))
    if len(pts) < LEFT_ROWS: return None
    P = np.array(pts); best = None; rng = np.random.default_rng(0)
    for _ in range(200):
        i, j = rng.choice(len(P), 2, replace=False)
        if abs(P[i, 0] - P[j, 0]) < 20: continue
        a = (P[j, 1] - P[i, 1]) / (P[j, 0] - P[i, 0]); b = P[i, 1] - a * P[i, 0]
        inl = np.abs(P[:, 1] - (a * P[:, 0] + b)) < 5
        if best is None or inl.sum() > best.sum(): best = inl
    if best is None or best.sum() < LEFT_ROWS: return None
    a, b = np.polyfit(P[best, 0], P[best, 1], 1); return dict(a=float(a), b=float(b), n=int(best.sum()))


# r13 (user 2026-11-03, live 43340794): in the two-lane section use the dashed line's extension (it leads to the
# downhill entrance centre) and, without dashes, the solid side lines. side_lines() fits the inner edge of the
# long thin white components on each side (left: x < 300, right: x > 340 at the bottom rows) as x = a*y + b.
SIDE_MIN_ROWS = 12
# r20: live_pid ends the TURN once the gyro says TURN_MAX_DEG were turned (1c87c960: at PWM 13 the row stayed in view, TURN
# lasted > 5.8 s and turned 104 deg of the ~88 needed; the dashes seen at 27-30 deg afterwards kept it in TURN).
TURN_MAX_DEG = 70.
# r21 lap re-arm (user, live 1f3d7ba7: the route stayed DONE from 26.7 s for the next 120 s, so the second crest had no dashed
# turn / two-lane logic): after a real downhill (pitch <= REARM_DOWN_DEG at some point after DONE) and REARM_S of level
# ground the route goes back to WAIT_CLIMB with the turn state cleared; self.lap counts the re-arms (live_pid resets the
# heading hold, lane segment and downhill entry). A crest-edge bump (-6 deg) does not re-arm. Experiment values.
REARM_DOWN_DEG, REARM_S = -10., 3.
# r23 (user 2026-11-04, live 0ef89769 vs 9eedee0a): (1) after the turn, 13.27-13.96 s, the real dashed line was in view at
# 25-28 deg (4-7 blocks), its direction 0-1.5 deg from the heading reference and 0.34-0.78 right of the car axis, but every
# line below ACROSS_DEG counted as a crossing row and was ignored: HEADING_BLIND 942 us went on and the car turned ~10 deg
# past the reference. Once turned, a line from ALIGN_MIN_DEG with at least START_N blocks whose direction is within
# ALIGN_GATE_DEG of the reference (expect_rel) is followed (FOLLOW). The lower lines at 11.6-12.2 s (6-13 deg) were 7-123 deg
# off and stay unused. (2) Both runs turned with the same commands; run 1 started with the car left of the lane (normal-path
# error +0.12..+0.28) and the fixed arc took the left wheels off, run 2 started right of it (-0.19..-0.29) and stayed on.
# A positive normal-path error (car left) delays the turn start by DELAY_K_S * error, at most DELAY_MAX_S. Experiment values.
ALIGN_MIN_DEG, ALIGN_GATE_DEG, DELAY_K_S, DELAY_MAX_S = 20., 10., 1., .4
# r15 (live a591a0ec 108.6-110.7 s): at the crest edge before the downhill the floor track below was taken as the lane's
# side lines (only in the upper half, slopes -2.65 / 3.47) and the lane segment jumped 1662 -> 955 -> 1521 us. A side line
# must reach SIDE_NEAR_Y (lower part of the 640x360 image). Experiment value.
SIDE_NEAR_Y = 250


def side_lines(img):
    if _FAST: return _side_lines_cc(cv2.connectedComponentsWithStats(white(img), 8))   # dash-fast-r1
    m = white(img)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    keep = np.zeros(n, bool)
    for k in range(1, n):
        x, y, w, h, a = st[k]
        if max(w, h) >= 120 and a >= 400 and a / float(w * h) < .5: keep[k] = True
    big = keep[lab]; out = {}
    for side in ('left', 'right'):
        pts = []
        for y in range(120, 359, 3):
            xs = np.flatnonzero(big[y])
            if side == 'left': xs = xs[xs < 300]; v = float(xs.max()) if len(xs) else None
            else: xs = xs[xs > 340]; v = float(xs.min()) if len(xs) else None
            if v is not None: pts.append((y, v))
        if len(pts) < SIDE_MIN_ROWS: continue
        P = np.array(pts); best = None; rng = np.random.default_rng(1)
        for _ in range(150):
            i, j = rng.choice(len(P), 2, replace=False)
            if abs(P[i, 0] - P[j, 0]) < 20: continue
            a = (P[j, 1] - P[i, 1]) / (P[j, 0] - P[i, 0]); b = P[i, 1] - a * P[i, 0]
            inl = np.abs(P[:, 1] - (a * P[:, 0] + b)) < 5
            if best is None or inl.sum() > best.sum(): best = inl
        if best is None or best.sum() < SIDE_MIN_ROWS or P[best, 0].max() < SIDE_NEAR_Y: continue
        a, b = np.polyfit(P[best, 0], P[best, 1], 1); out[side] = dict(a=round(float(a), 5), b=round(float(b), 2), n=int(best.sum()))
    return out


def turn_steer(center_us, min_us):
    return float(center_us) - TURN_FRAC * (float(center_us) - float(min_us))
AXIS = 338


def white(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 2] >= V_MIN) & (hsv[..., 1] <= S_MAX); m[:Y_TOP] = False
    return cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def dash_blocks(m):
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8); out = []
    for k in range(1, n):
        x, y, w, h, a = st[k]
        if not MIN_A <= a <= MAX_A: continue
        long_side = max(w, h)
        if long_side > .45 * 640 or (w > 180 and h > 60) or a / float(w * h) < .35: continue
        if x <= 1 or x + w >= 639: continue                                     # side rails / partial blocks at the border
        out.append((float(cen[k][0]), float(cen[k][1]), float(w >= NEAR_W and w >= NEAR_AR * h)))   # r22: 3rd = near-row shape
    return out


def ransac_line(B, iters=200, tol=10., seed=0, near_shape=True):
    if len(B) < 3: return None
    P = np.array(B); rng = np.random.default_rng(seed); best = None
    for _ in range(iters):
        i, j = rng.choice(len(P), 2, replace=False)
        if abs(P[i, 1] - P[j, 1]) < 4: continue
        a = (P[j, 0] - P[i, 0]) / (P[j, 1] - P[i, 1]); b = P[i, 0] - a * P[i, 1]
        inl = np.abs(P[:, 0] - (a * P[:, 1] + b)) / np.sqrt(1 + a * a) < tol * (1 + P[:, 1] / 360)
        nr = inl & (P[:, 1] >= NEAR_Y) & ((P[:, 2] > 0) if near_shape else True)   # r22: the near block must be dash-shaped (r24: not after FOLLOW)
        if inl.sum() >= 3 and nr.any() and (best is None or inl.sum() > best.sum()): best = inl
    if best is None: return None
    a, b = np.polyfit(P[best, 1], P[best, 0], 1)
    return dict(a=float(a), b=float(b), n=int(best.sum()), xs=P[best, 0].tolist(), ys=P[best, 1].tolist(), near=int((P[best, 1] >= NEAR_Y).sum()))


def detect(img, near_shape=True):
    if _FAST: return _detect_cc(cv2.connectedComponentsWithStats(white(img), 8), near_shape)   # dash-fast-r1
    L = ransac_line(dash_blocks(white(img)), near_shape=near_shape)
    if L is None: return None
    L['angle'] = 90 - abs(float(np.degrees(np.arctan(L['a']))))
    return L


def _line_path(x0, y0, x1, y1, y_top):
    """Points (1280 scale) every 4 px (640 scale) on the line through (x0,y0)-(x1,y1), from y_top to 359."""
    k = (x1 - x0) / (y1 - y0) if y1 != y0 else 0.
    ys = np.arange(min(y_top, 358.), 359.01, 4.)
    return [[float(np.clip(x0 + k * (y - y0), 0, 639.5)) * 2, float(y) * 2] for y in ys]


# dash-offthread-r1 (pipeline2-20261116): the image part of _try_path_cam (decode, resize, white mask, dash blocks, RANSAC for both
# near_shape values, side lines) as a pure function of the JPEG, run by live_pid on a helper thread while the worker infers the same
# frame. _try_path_cam(pre=...) uses it only for the very same JPEG object; the state machine still runs in the main thread on this
# frame, so the result is the one the synchronous code computes (same seeds, same functions).
def precompute(jpeg):
    if _FAST: return _precompute_fast(jpeg)   # dash-fast-r1
    out = dict(jpeg=jpeg, img=None, L={}, sides=None, sides_ok=False)
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None: return out
    img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA); out['img'] = img
    B = dash_blocks(white(img))
    for ns in (True, False):
        L = ransac_line(B, near_shape=ns)
        if L is not None: L['angle'] = 90 - abs(float(np.degrees(np.arctan(L['a']))))
        out['L'][ns] = L
    try: out['sides'] = side_lines(img); out['sides_ok'] = True
    except Exception: pass
    return out


def _pre_get(pre, jpeg):
    if pre is None: return None
    try:
        P = pre.result(.25) if hasattr(pre, 'result') else pre
        return P if isinstance(P, dict) and P.get('jpeg') is jpeg else None
    except Exception: return None


class DashRoute:
    def seed(self, plan):
        from ops07_route import seed_dash
        return seed_dash(self, plan)

    def __init__(self, boost, crest_start=False, left_lane=False, lane=None):
        self.boost = boost; self.state = 'WAIT_CLIMB'; self.climb_since = self.level_since = self.opened = None
        self.across_n = 0; self.mode = None; self.last_seen = None; self.x_prev = None; self.t_prev = None; self.follow_since = None; self.turned = False; self.followed = False
        self.left_lane = left_lane; self.lane_k = LANE_K; self.lane = lane; self.expect_rel = None
        self.delay_since = self.delay_s = self.delay_last = None   # r23
        if crest_start: self.state = 'OPEN'   # r7: opened time is set on the first route update
        self.lap = 0; self.saw_down = False; self.rearm_since = None
        self.tt = load_turn_timing(); self.v_mps = None   # upper-turn-r1
        load_fast()   # dash-fast-r1: web/dash_fast.json (absent = off)

    def _cap_delay(self, info):
        """upper-turn-r1: the r23 delay may not use more than the slack d0 - (a + b * v); unknown / low speed -> unchanged."""
        try:
            ds = self.tt['delay_slack']; v = float(self.v_mps)
            if not v > float(ds.get('v_min', .2)): return
            cap = max(0., (float(ds['d0']) - float(ds['a']) - float(ds['b']) * v) / v)
            if cap < self.delay_s:
                info['turn_timing'] = dict(revision=TT_REVISION, delay_s=round(self.delay_s, 3), capped_s=round(cap, 3), v=round(v, 3)); self.delay_s = cap
        except Exception: pass

    def update_route(self, now):
        """IMU route window; returns (open, info). Never raises."""
        info = dict(revision=REVISION, route=self.state)
        try:
            t, p = self.boost._read()
        except Exception as e:
            info.update(reason='IMU_UNAVAILABLE', error=type(e).__name__); return self.state == 'OPEN', info
        age = now - t; rel = p - self.boost.c['flat_pitch_deg']; info.update(pitch_rel_deg=round(rel, 2), imu_age_s=round(age, 3)); self.pitch_rel = rel   # dash-pos-backup-r1: attribute only
        if not -.05 <= age <= self.boost.c['max_age_s']:
            info.update(reason='IMU_STALE'); return self.state == 'OPEN', info
        if self.state == 'WAIT_CLIMB':
            if rel >= CLIMB_DEG:
                self.climb_since = now if self.climb_since is None else self.climb_since
                if now - self.climb_since >= CLIMB_S: self.state = 'CLIMBED'
            else: self.climb_since = None
        elif self.state == 'CLIMBED':
            if rel < LEVEL_DEG:
                self.level_since = now if self.level_since is None else self.level_since
                if now - self.level_since >= LEVEL_S: self.state, self.opened = 'OPEN', now
            else: self.level_since = None
        elif self.state == 'OPEN':
            if self.opened is None: self.opened = now
            if rel <= DOWN_DEG or now - self.opened > WINDOW_S: self.state = 'DONE'; self.saw_down = False; self.rearm_since = None
        elif self.state == 'DONE':   # r21: re-arm for the next lap
            if rel <= REARM_DOWN_DEG: self.saw_down = True
            if self.saw_down and abs(rel) < LEVEL_DEG:
                self.rearm_since = now if self.rearm_since is None else self.rearm_since
                if now - self.rearm_since >= REARM_S:
                    self.state = 'WAIT_CLIMB'; self.climb_since = self.level_since = self.opened = None; self.lap += 1
                    self.across_n = 0; self.mode = None; self.last_seen = None; self.x_prev = None; self.t_prev = None; self.follow_since = None
                    self.turned = False; self.followed = False; self.saw_down = False; self.rearm_since = None
                    self.delay_since = self.delay_s = self.delay_last = None; self.tt_along = False   # upper-turn-r1 (lap re-arm)
            else: self.rearm_since = None
        info['route'] = self.state
        return self.state == 'OPEN', info

    def try_path(self, now, jpeg, row_ratio, normal_err=None, pre=None):   # dash-offthread-r1: pre = precompute job of this JPEG (None = today)
        if not self.tt.get('pos_backup'): return self._try_path_cam(now, jpeg, row_ratio, normal_err, pre)   # dash-pos-backup-r1: key absent = camera only
        path, info = self._try_path_cam(now, jpeg, row_ratio, normal_err, pre)
        try:
            if self.turned or self.mode is not None or self.state != 'OPEN': return path, info
            pb = self.tt['pos_backup']; lp = getattr(self, 'lp', None) or {}; s = lp.get('since_anchor_m'); rel = getattr(self, 'pitch_rel', None)
            ok = (lp.get('anchor') == 'TOP' and lp.get('lost') is False and s is not None and int(lp.get('lap') or 0) >= int(pb.get('min_lap', 1))
                  and float(pb.get('after_m', 2.9)) <= float(s) < float(pb.get('max_m', 4.0)) and rel is not None and abs(float(rel)) < float(pb.get('max_pitch_deg', 3.)))
            if not ok: return path, info
            row_y = row_ratio * 719 / 2; x_row = float(pb.get('x_row', 139.))
            self.turned = True; self.follow_since = None; self.across_n = 0
            self.mode, self.last_seen, self.x_prev, self.t_prev = 'TURN', now, x_row, now
            info = dict(info, reason='OK', used=True, mode='TURN', x_row=round(x_row, 1), pos_backup=dict(revision=PB_REVISION, since_m=round(float(s), 2), cam_reason=info.get('reason')))
            return self._path_to(x_row, row_y), info
        except Exception: return path, info

    def _try_path_cam(self, now, jpeg, row_ratio, normal_err=None, pre=None):
        """normal_err: image error of the normal path of the same frame (> 0: target right of the car = car left), or None."""
        row_y = row_ratio * 719 / 2; info = dict(revision=REVISION, used=False)
        P = _pre_get(pre, jpeg)   # dash-offthread-r1: None = compute here (today)
        if pre is not None: info['pre'] = 'USED' if P is not None else 'FALLBACK'
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) if P is None else P['img']
        if img is None: info.update(reason='JPEG_DECODE'); return None, info
        if P is None: img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
        ns_ = not (self.followed or getattr(self, 'tt_along', False))
        L = detect(img, near_shape=ns_) if P is None else P['L'].get(ns_)   # r24; upper-turn-r1: also once an along line was held in the TURN (near block looks tall, as after FOLLOW)
        if self.turned:
            if P is not None: info['sides'] = {k: dict(v) for k, v in P['sides'].items()} if P['sides_ok'] else {}
            else:
                try: info['sides'] = side_lines(img)   # r13
                except Exception: info['sides'] = {}
        if L is None:
            self.across_n = 0
            if self.mode and self.last_seen is not None and now - self.last_seen <= LOST_S and self.x_prev is not None:
                info.update(reason='HOLD_LAST', mode=self.mode); self.t_prev = now   # keep the last target briefly
                return self._path_to(self.x_prev, row_y), dict(info, used=True, x_row=round(self.x_prev, 1))
            self.mode = None; info.update(reason='NO_DASH_LINE'); return None, info
        info.update(n=L['n'], angle=round(L['angle'], 1))
        aligned = False
        if self.turned and ALIGN_MIN_DEG <= L['angle'] < ACROSS_DEG and L['n'] >= START_N and self.expect_rel is not None:   # r23
            exp = self.expect_rel[0] if isinstance(self.expect_rel, tuple) else self.expect_rel
            rel = float(np.degrees(np.arctan((L['a'] * HORIZON_Y + L['b'] - VP_X) / F_PX)))
            if abs(rel - exp) <= ALIGN_GATE_DEG:
                if getattr(self, 'align_ok', True): aligned = True; info.update(aligned=True)
                else: info.update(align_blocked='TURN_TOO_SMALL')   # dash-align-min-r1 (user 2026-11-11 7b24 58.1 s: "aligned" at 32 deg of yaw, the turn ended early)
        if not aligned and L['angle'] < ACROSS_DEG and (self.followed or (self.mode is None and self.turned)):
            self.across_n = 0
            if self.last_seen is not None and now - self.last_seen <= LOST_S and self.x_prev is not None:
                info.update(reason='HOLD_LAST', mode='FOLLOW', ignored='ACROSS_AFTER_FOLLOW'); self.t_prev = now
                return self._path_to(self.x_prev, row_y), dict(info, used=True, x_row=round(self.x_prev, 1))
            self.mode = None; info.update(reason='NO_DASH_LINE', ignored='ACROSS_AFTER_FOLLOW'); return None, info
        if not aligned and L['angle'] < ACROSS_DEG:
            # starting the turn needs NEAR_BLOCKS near blocks (false row at 46.6 s in 95d42be8);
            # once the turn/follow is active one near block is enough
            ok3 = L['angle'] < START3_DEG and L['n'] >= START3_N   # r26
            if ok3:
                if getattr(self, 'n3_t', None) is None or now - self.n3_t > START3_GAP_S: self.n3 = 0
                self.n3 = getattr(self, 'n3', 0) + _RI.w(); self.n3_t = now
            if self.mode is None and not ((L['angle'] < START_DEG and L['n'] >= START_N) or (L['angle'] < START_DEG2 and L['n'] >= START_N2)
                                          or ((self.across_n >= 1 if not _RI.ON else self.across_n > 0) and L['angle'] < START_DEG2 and L['n'] >= CONT_N) or (ok3 and getattr(self, 'n3', 0) >= _RI.thr(START3_FRAMES))):
                self.across_n = 0; info.update(reason='ACROSS_NOT_START_ROW'); return None, info
            self.across_n += _RI.w()
            if self.mode is None and self.across_n < _RI.thr(START_FRAMES) and not (L['angle'] < START_DEG and L['n'] >= STRONG_N) and self.delay_since is None:   # r23: after a delay the row was already confirmed
                info.update(reason='ACROSS_PENDING'); return None, info
            if self.mode is None and not self.turned:   # r23: car left of the lane -> start the turn a little later
                if self.delay_since is not None and now - self.delay_last > LOST_S: self.delay_since = None
                if self.delay_since is None and normal_err is not None and normal_err > 0:
                    self.delay_since, self.delay_s = now, min(DELAY_MAX_S, DELAY_K_S * float(normal_err))
                    if self.tt.get('delay_slack'): self._cap_delay(info)   # upper-turn-r1
                if self.delay_since is not None:
                    self.delay_last = now
                    if now - self.delay_since < self.delay_s:
                        info.update(reason='TURN_DELAYED', delay_s=round(self.delay_s, 2), normal_err=round(float(normal_err), 3) if normal_err is not None else None); return None, info
            i = int(np.argmin(L['xs'])); tx, ty = L['xs'][i], L['ys'][i]
            path = _line_path(AXIS, 359., tx, ty, min(ty, row_y - 1)); mode = 'TURN'
        else:
            self.across_n = 0
            if self.mode is None and not self.turned: info.update(reason='ALONG_BEFORE_TURN'); return None, info   # follow only after the turn
            if self.mode == 'TURN' and self.tt.get('along_min_turn') and not getattr(self, 'align_ok', True) and self.x_prev is not None:   # upper-turn-r1
                self.last_seen = self.t_prev = now; self.tt_along = True; info.update(reason='HOLD_LAST', mode='TURN', ignored='ALONG_BEFORE_MIN_TURN', turn_timing=dict(revision=TT_REVISION))
                return self._path_to(self.x_prev, row_y), dict(info, used=True, x_row=round(self.x_prev, 1))
            if self.expect_rel is not None:   # r11: direction gate against the heading reference
                exp, gate = self.expect_rel if isinstance(self.expect_rel, tuple) else (self.expect_rel, DIR_GATE_DEG)
                rel = float(np.degrees(np.arctan((L['a'] * HORIZON_Y + L['b'] - VP_X) / F_PX))); info.update(rel_deg=round(rel, 1), expect_deg=round(exp, 1), gate_deg=gate)
                if abs(rel - exp) > gate:
                    info.update(reason='DIRECTION_MISMATCH')
                    if self.mode and self.last_seen is not None and now - self.last_seen <= LOST_S and self.x_prev is not None:
                        self.t_prev = now; return self._path_to(self.x_prev, row_y), dict(info, used=True, mode=self.mode, x_row=round(self.x_prev, 1))
                    self.mode = None; return None, info
            if self.mode is None and L['n'] < START_N: info.update(reason='REACQUIRE_TOO_FEW'); return None, info   # cd6f0c63 67.27 s: 3 blocks on the curves beyond the ramp
            y_top = min(min(L['ys']), row_y - 1); ys = np.arange(y_top, 359.01, 4.)
            path = [[float(np.clip(L['a'] * y + L['b'], 0, 639.5)) * 2, float(y) * 2] for y in ys]; mode = 'FOLLOW'
            if self.lane == 'right_quarter':   # r8: between the dashes and the right lane
                path = [[float(np.clip(L['a'] * y + L['b'] + RIGHT_K * max(0., y - HORIZON_Y), 0, 639.5)) * 2, float(y) * 2] for y in ys]; info.update(lane='RIGHT_QUARTER')
            elif self.left_lane:   # r7: left lane centre
                S = solid_left(img, L)
                if S is not None:
                    xd, xl = L['a'] * row_y + L['b'], S['a'] * row_y + S['b']
                    self.lane_k = float(np.clip((xd - xl) / 2 / max(1., row_y - HORIZON_Y), *LANE_K_RANGE))
                    path = [[float(np.clip(((L['a'] + S['a']) * y + L['b'] + S['b']) / 2, 0, 639.5)) * 2, float(y) * 2] for y in ys]; info.update(lane='LEFT_LINE', left_rows=S['n'])
                else:
                    path = [[float(np.clip(L['a'] * y + L['b'] - self.lane_k * max(0., y - HORIZON_Y), 0, 639.5)) * 2, float(y) * 2] for y in ys]; info.update(lane='LEFT_OFFSET')
                info['lane_k'] = round(self.lane_k, 3)
        x_row = float(np.interp(row_y, [p[1] / 2 for p in path], [p[0] / 2 for p in path]))
        if self.x_prev is not None and self.t_prev is not None:
            lim = (LANE_SHIFT_PX_S if self.lane else MAX_SHIFT_PX_S) * max(.0, now - self.t_prev); x_lim = float(np.clip(x_row, self.x_prev - lim, self.x_prev + lim))
            if x_lim != x_row: path = [[min(max(p[0] + (x_lim - x_row) * 2, 0.), 1279.), p[1]] for p in path]; info['shift_limited'] = round(x_row - x_lim, 1); x_row = x_lim
        if mode == 'TURN': self.turned = True
        if mode == 'FOLLOW': self.followed = True
        if mode == 'FOLLOW' and self.mode != 'FOLLOW': self.follow_since = now
        elif mode != 'FOLLOW': self.follow_since = None
        self.mode, self.last_seen, self.x_prev, self.t_prev = mode, now, x_row, now
        info.update(reason='OK', used=True, mode=mode, x_row=round(x_row, 1))
        if mode == 'FOLLOW': info.update(line_a=round(L['a'], 5), line_b=round(L['b'], 2))   # r10: dashed-line fit for heading-hold
        return path, info

    def _path_to(self, x_row, row_y):
        return _line_path(AXIS, 359., x_row, row_y - 1, row_y - 1)
