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
# dash-start-r2 (dash-start-20261117, offline candidate): upper_turn.json key start_gate {min_top_m, min_open_m, extrap_max_m, confirm_s,
# gap_s, enabled}. Absent / null / enabled false -> nothing below runs (dash-route-r27 + upper-turn-r1 + dash-pos-backup-r1 exactly).
# When present, a camera TURN start (mode None, not turned, a start-quality across row) is refused on this frame when it is too early:
#  * position: every available measure says early -- lap_pos metres since TOP (live_pid sets self.lp only while upper_turn.json has
#    pos_backup; anchored TOP and not lost; the value steps with the 5 Hz encoder, so it is extrapolated with this route's own odometry by
#    at most extrap_max_m) < min_top_m, and the metres since this route OPEN (sum of v_mps * dt over try_path calls; unknown once v_mps was
#    None) < min_open_m. No measure available -> no position block (camera only, as before).
#  * confirm_s (optional, null = off): the start-quality across rows of this approach (frames at most gap_s apart) must span >= confirm_s.
# 7afb lap 2 (11-17, ~30 frames/s): a floor-mat seam (jagged edge of the darker mat, blobs at y 216-222) gave 7-11 blocks at 8-13 deg and
# started the TURN at TOP+1.81 m / OPEN+1.43 m; 7afb lap 1 saw the same seam at TOP+1.66-1.82 m (ACROSS_PENDING, 16-24 deg, no start)
# and started on the real row at TOP+2.44 m. 140 normal starts 11-14..17: TOP+2.44..2.88 m (value the dash code sees: >= 2.35),
# OPEN+2.06..2.48 m. Experiment values.
SG_REVISION = 'dash-start-r2'

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
    if _WREL:   # dash-white-rel-r1
        try: _precompute_rel(out, img)
        except Exception: out.pop('Lrel', None); out.pop('sides_rel', None); out['wrel_error'] = True
    return out


def load_turn_timing(path=None):
    try:
        c = _json.loads(_Path(path or _Path(__file__).with_name('upper_turn.json')).read_text(encoding='utf8'))
        return c if isinstance(c, dict) and c.get('enabled') else {}
    except Exception: return {}

# dash-white-rel-r1 (light-adapt-20261118, offline candidate): the white threshold of the dash search follows the floor brightness.
# upper_turn.json key white_rel {enabled, band, pct, delta, ref_on, v_min, v_max, scope}; read when the DashRoute is built (run start, with the
# rest of upper_turn.json). Key absent / null / not a dict / enabled false / invalid values -> _WREL None -> nothing below runs and every
# function is the dash-fast-r1 / dash-route-r27 code exactly.
# When on: ref = percentile(V of rows band[0]..band[1]-1, pct) per frame (HSV V of the 640x360 image, all columns); thr = v_min while
# ref < ref_on, else clip(ref + delta, v_min, v_max); the white mask for the dash search is V >= thr (S <= S_MAX and the 3x3 opening
# unchanged). ref_on keeps the normal-light frames on the base threshold (dim days stay bit-identical). thr == V_MIN (185) gives exactly
# the base mask. scope "pre" (default): only before the dashed TURN (not turned): the across row that starts the turn; after the turn
# (TURN / FOLLOW / side lines) the base mask is used as today. scope "all": also after the turn and for the side lines.
# Why: 7392 (11-18 17:45, manual exposure 30 ms + gamma 300) the far floor (rows 100-170) was V 184-196 (morning 154-160, afternoon
# 138-147); the dashes of the across row merged with the floor at V >= 185 into components rejected as too big, the fitted row tilted
# 14-22 deg with 3-5 blocks and the TURN started 0.2-0.3 m late (dash-late-20261118). Experiment values (light-adapt-20261118/dash).
WR_REVISION = 'dash-white-rel-r1'
_WREL = None


def set_white_rel(c):
    """validate upper_turn.json white_rel; sets the module global _WREL (None = off). Never raises."""
    global _WREL
    _WREL = None
    try:
        if not isinstance(c, dict) or c.get('enabled', True) is not True: return None
        b0, b1 = int(c.get('band', [100, 175])[0]), int(c.get('band', [100, 175])[1])
        pct, delta, ref_on = float(c.get('pct', 50.)), float(c.get('delta', 30.)), float(c.get('ref_on', 165.))
        lo, hi = int(c.get('v_min', V_MIN)), int(c.get('v_max', 235))
        scope = c.get('scope', 'pre')
        if not (Y_TOP <= b0 < b1 <= 360 and 0. <= pct <= 100. and -50. <= delta <= 100. and 0. <= ref_on <= 255. and 100 <= lo <= hi <= 255 and scope in ('pre', 'all')): return None
        _WREL = dict(band=(b0, b1), pct=pct, delta=delta, ref_on=ref_on, v_min=lo, v_max=hi, scope=scope)
    except Exception: _WREL = None
    return _WREL


def _wr_thr(V):
    """threshold for this frame from the V channel (uint8 HxW); returns (thr, ref)."""
    c = _WREL; S_ = V[c['band'][0]:c['band'][1]]
    ref = float(np.median(S_)) if c['pct'] == 50. else float(np.percentile(S_, c['pct']))
    if ref < c['ref_on']: return int(c['v_min']), ref
    return int(min(max(round(ref + c['delta']), c['v_min']), c['v_max'])), ref


def _white_hsv(hsv, vmin):
    m = (hsv[..., 2] >= vmin) & (hsv[..., 1] <= S_MAX); m[:Y_TOP] = False
    return cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def white_rel(img):
    """(mask, thr, ref) with the relative threshold; requires _WREL."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV); thr, ref = _wr_thr(hsv[..., 2])
    return _white_hsv(hsv, thr), thr, ref


def detect_rel(img, near_shape=True):
    """detect() on the relative mask; returns (L, info)."""
    m, thr, ref = white_rel(img)
    if _FAST: L = _detect_cc(cv2.connectedComponentsWithStats(m, 8), near_shape)
    else:
        L = ransac_line(dash_blocks(m), near_shape=near_shape)
        if L is not None: L['angle'] = 90 - abs(float(np.degrees(np.arctan(L['a']))))
    return L, dict(revision=WR_REVISION, thr=thr, ref=round(ref, 1), scope=_WREL['scope'])


def _side_lines_mask(m):
    if _FAST: return _side_lines_cc(cv2.connectedComponentsWithStats(m, 8))
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


def side_lines_rel(img):
    return _side_lines_mask(white_rel(img)[0])


def _precompute_rel(out, img):
    """dash-white-rel-r1 part of precompute (only when _WREL): out['Lrel'][ns], out['wrel'] info; scope all -> also out['sides_rel']."""
    m, thr, ref = white_rel(img); out['wrel'] = dict(revision=WR_REVISION, thr=thr, ref=round(ref, 1), scope=_WREL['scope'])
    if thr == V_MIN and out.get('L') is not None and len(out['L']) == 2:   # same mask as the base part above -> same results, no second search
        out['Lrel'] = {ns: (None if L is None else dict(L, xs=list(L['xs']), ys=list(L['ys']))) for ns, L in out['L'].items()}
        if _WREL['scope'] == 'all': out['sides_rel'] = {k: dict(v) for k, v in out['sides'].items()} if out.get('sides_ok') else None
        return out
    if _FAST:
        cc = cv2.connectedComponentsWithStats(m, 8); B = _blocks_cc(cc); out['Lrel'] = {}
        for ns in (True, False):
            L = _ransac_fast(B, near_shape=ns)
            if L is not None: L['angle'] = 90 - abs(float(np.degrees(np.arctan(L['a']))))
            out['Lrel'][ns] = L
        if _WREL['scope'] == 'all':
            try: out['sides_rel'] = _side_lines_cc(cc)
            except Exception: out['sides_rel'] = None
    else:
        B = dash_blocks(m); out['Lrel'] = {}
        for ns in (True, False):
            L = ransac_line(B, near_shape=ns)
            if L is not None: L['angle'] = 90 - abs(float(np.degrees(np.arctan(L['a']))))
            out['Lrel'][ns] = L
        if _WREL['scope'] == 'all':
            try: out['sides_rel'] = _side_lines_mask(m)
            except Exception: out['sides_rel'] = None
    return out

# dash-range-start-r1 (dash-range-20261119, offline candidate): start the dashed TURN by the RANGE of the across row, not by "seen clearly
# enough". upper_turn.json key range_start {enabled, a, b, v_ref, v_lo, v_hi, r_max_cm, g_max_deg, tol_cm, min_obs, gap_s, dr_max_s}; read when
# the DashRoute is built (run start). Absent / null / not a dict / enabled false / invalid values / camera_ground_calibration.json unreadable
# -> self.rs None -> nothing below runs (dash-route-r27 + upper-turn-r1 + dash-pos-backup-r1 + dash-start-r2 + dash-white-rel-r1 exactly).
# When on, before the TURN (mode None, not turned), every across-row fit L (the same fit the start test uses today, i.e. with white_rel when
# that is on) gives an observation: range R = ground distance (cm, ahead of the front-tyre foremost edge) where the row crosses the car centre
# line: the inlier block centres projected with the flat-ground homography (camera_ground_calibration.json next to this file), line
# z = c + d x fitted, R = c (x = 0). Usable when R <= r_max_cm and |atan d| <= g_max_deg (80: only near-vertical fits are refused; with the
# 11-09 homography and today's camera pose a square row reads about -35 deg on the ground, 49 frames, 5..95 % -62..-32: a pose artefact,
# constant, so R at x = 0 is still usable; R0 is calibrated on outcomes, not on absolute range). A track = consecutive consistent observations:
# line position P = own odometry (sum of v_mps * dt) + R; the first observation needs today's first-frame start quality (< START_DEG with
# >= START_N blocks or < START_DEG2 with >= START_N2), the next ones the continuation quality (< START_DEG2, >= CONT_N blocks), within gap_s
# of the previous one and |P - P_track| <= tol_cm (else a new track starts / the frame is ignored). Confirmed = >= min_obs observations, or
# one STRONG_N row. The TURN starts when the track is confirmed and the range (this frame's R, or between observations P_track - odometry,
# at most dr_max_s after the last observation) is <= R0 = a + b * v (m; v = v_mps clipped to v_lo..v_hi, v_ref when unknown). Count-based,
# not a frame-rate-based time (rate_indep not used). Before that: reason RANGE_UNCONFIRMED / RANGE_WAIT (no path, as ACROSS_PENDING today).
# start_gate (TURN_TOO_EARLY) and the r23 delay still apply to a camera start; pos_backup is unchanged (it runs when nothing started).
# A start between observations (odometry) needs a track last seen beyond R0 and uses that observation's target x_row, no r23 delay.
# A start refused by start_gate drops the track (7a70: the floor-mat seam was tracked at TOP+1.4-1.6 m and, without this, started by odometry
# at TOP+1.89 m once open_m passed 1.8 m).
# Why (dash-range-20261119): with white_rel the camera already started at R 97-109 cm in both dark (7a70, 4 passes, 99 median) and bright
# (11-18 evening, 17 passes, 102.5); the late turns before white_rel started at R 72-87 cm. R is consistent: in-pass slope dR/d(odometry)
# -100 cm/m (17 bright passes, median) -> no range-scale correction needed in 70-140 cm. a / b: b 0.46 s (upper-20261115 geom fit, 29 passes),
# a from the measured relation X(after turn) = -1.27 R + 109 cm with the TURN steering of dark days (8 passes) -> R0 0.83 m at 0.78 m/s.
RS_REVISION = 'dash-range-start-r1'
_RS_H = None
RS_DEFAULTS = dict(a=.471, b=.46, v_ref=.78, v_lo=.4, v_hi=1., r_max_cm=160., g_max_deg=80., tol_cm=12., min_obs=2, gap_s=.6, dr_max_s=.6)


def _rs_h():
    global _RS_H
    if _RS_H is None:
        d = _json.loads(_Path(__file__).with_name('camera_ground_calibration.json').read_text(encoding='utf8'))
        H = np.asarray(d['homography_image_to_ground_cm'], float)
        if H.shape != (3, 3) or not np.isfinite(H).all(): raise ValueError('bad homography')
        _RS_H = H
    return _RS_H


def rs_config(c):
    """validate upper_turn.json range_start; returns the config dict or None (off). Never raises."""
    try:
        if not isinstance(c, dict) or c.get('enabled', True) is not True: return None
        r = dict(RS_DEFAULTS)
        for k in RS_DEFAULTS:
            if c.get(k) is not None: r[k] = float(c[k])
        r['min_obs'] = int(r['min_obs'])
        if not (0. <= r['a'] <= 2. and 0. <= r['b'] <= 2. and .05 <= r['v_lo'] <= r['v_ref'] <= r['v_hi'] <= 3. and 50. <= r['r_max_cm'] <= 400.
                and 0. < r['g_max_deg'] <= 89. and 0. < r['tol_cm'] <= 100. and 1 <= r['min_obs'] <= 10 and 0. < r['gap_s'] <= 3. and 0. <= r['dr_max_s'] <= 3.): return None
        _rs_h(); return r
    except Exception: return None


def row_range_cm(L):
    """(R cm, row ground angle deg) of a dash-row fit: inlier centres -> ground (homography), z = c + d x, R = c; (None, None) if < 5 cm wide."""
    H = _rs_h(); xs = np.asarray(L['xs'], float); ys = np.asarray(L['ys'], float)
    G = np.column_stack((xs, ys, np.ones(len(xs)))) @ H.T; gx = G[:, 0] / G[:, 2]; gz = G[:, 1] / G[:, 2]
    if len(gx) < 2 or float(np.ptp(gx)) <= 5.: return None, None
    d, c = np.polyfit(gx, gz, 1); return float(c), float(np.degrees(np.arctan(d)))


# dash-turn-rel-r1 (dash-range-20261119, offline candidate): upper_turn.json key turn_mask_rel (true / absent). True: while the dashed TURN is
# active (mode TURN, before any FOLLOW) the row fit uses the white_rel mask too (as before the turn); FOLLOW and the side lines keep the base
# mask. Needs white_rel on (no effect otherwise). Absent / not true -> exactly as before.
# Why (dash-range-20261119): the TURN steering follows x_row (dash-turn-vision-r1: centre + 800 (x_row - 338) / 320), x_row aims at the
# LEFTMOST block of the row. After the start the base mask (V >= 185) was used: on bright evenings the far-left dashes merged with the floor
# and the leftmost block was near the image centre (first 0.8 s of TURN: image x 235 median, video, 90 frames) -> x_row 222, steer 1210 us,
# gz 26 deg/s; on dark days the whole row is found (x 49) -> x_row 138-142, steer 980-1020 us, gz 33-35 -> the turn radius differed by
# brightness and the car ended 15-20 cm further inside on dark days (11-19 8545 / 9b2e: X -17 median, 9 passes; bright 11-18: +3.8, 17).
# With the relative mask the bright TURN frames find the row like dark ones (image x 42 vs 41).
TR_REVISION = 'dash-turn-rel-r1'

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
    if _WREL:   # dash-white-rel-r1
        try: _precompute_rel(out, img)
        except Exception: out.pop('Lrel', None); out.pop('sides_rel', None); out['wrel_error'] = True
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
        g_ = self.tt.get('start_gate'); self.sg = g_ if isinstance(g_, dict) and g_.get('enabled', True) else None   # dash-start-r2 (None = off)
        self.sg_open = self.sg_tl = self.sg_d = self.sg_s = self.sg_sd = self.sg_c0 = self.sg_cl = None   # dash-start-r2
        load_fast()   # dash-fast-r1: web/dash_fast.json (absent = off)
        set_white_rel(self.tt.get('white_rel'))   # dash-white-rel-r1: key absent = off (_WREL None)
        self.rs = rs_config(self.tt.get('range_start'))   # dash-range-start-r1 (None = off)
        self.rs_open = self.rs_tl = self.rs_od = self.rs_P = self.rs_t = self.rs_x = self.rs_R = None; self.rs_n = 0; self.rs_strong = False
        self.tmr = self.tt.get('turn_mask_rel') is True   # dash-turn-rel-r1

    def _cap_delay(self, info):
        """upper-turn-r1: the r23 delay may not use more than the slack d0 - (a + b * v); unknown / low speed -> unchanged."""
        try:
            ds = self.tt['delay_slack']; v = float(self.v_mps)
            if not v > float(ds.get('v_min', .2)): return
            cap = max(0., (float(ds['d0']) - float(ds['a']) - float(ds['b']) * v) / v)
            if cap < self.delay_s:
                info['turn_timing'] = dict(revision=TT_REVISION, delay_s=round(self.delay_s, 3), capped_s=round(cap, 3), v=round(v, 3)); self.delay_s = cap
        except Exception: pass

    def _sg_tick(self, now):
        """dash-start-r2, every try_path call: metres since this route OPEN (sum of v_mps * dt, dt <= 0.5 s; None for the rest of the
        opening once v_mps is unknown) and the odometry at the moment the lap_pos metres-since-TOP value last changed."""
        try:
            if self.sg_tl is None or self.sg_open != self.opened:
                self.sg_open, self.sg_tl, self.sg_d, self.sg_s, self.sg_sd = self.opened, now, 0., None, None
            else:
                dt = min(max(0., now - self.sg_tl), .5); self.sg_tl = now
                if self.sg_d is not None and self.v_mps is not None: self.sg_d += max(0., float(self.v_mps)) * dt
                else: self.sg_d = None
            lp = getattr(self, 'lp', None); s = lp.get('since_anchor_m') if isinstance(lp, dict) else None
            if s != self.sg_s: self.sg_s, self.sg_sd = s, self.sg_d   # new lap_pos value (steps with the encoder)
        except Exception: self.sg_d = None

    def _sg_block(self, now, info):
        """dash-start-r2: True = refuse the TURN start on this frame (info gets reason and start_gate). Never raises (error -> False)."""
        try:
            g = self.sg; d = dict(revision=SG_REVISION); early = []
            lp = getattr(self, 'lp', None); mt = g.get('min_top_m')
            if mt is not None and isinstance(lp, dict) and lp.get('anchor') == 'TOP' and lp.get('lost') is False and lp.get('since_anchor_m') is not None:
                s = float(lp['since_anchor_m'])   # value of the last lap_pos update; extrapolated with our odometry since it last changed
                ex = 0. if self.sg_d is None or self.sg_sd is None or lp['since_anchor_m'] != self.sg_s else min(max(0., self.sg_d - self.sg_sd), float(g.get('extrap_max_m', .3)))
                d['top_m'] = round(s + ex, 3); early.append(s + ex < float(mt))
            mo = g.get('min_open_m')
            if mo is not None and self.sg_d is not None: d['open_m'] = round(self.sg_d, 3); early.append(self.sg_d < float(mo))
            why = 'TURN_TOO_EARLY' if early and all(early) else None
            cs = g.get('confirm_s')
            if cs is not None:
                if self.sg_cl is None or now - self.sg_cl > float(g.get('gap_s', .25)): self.sg_c0 = now
                self.sg_cl = now; d['seen_s'] = round(now - self.sg_c0, 3)
                if why is None and now - self.sg_c0 < float(cs): why = 'TURN_UNCONFIRMED'
            info['start_gate'] = d
            if why: info.update(reason=why); return True
            return False
        except Exception: return False

    def _rs_tick(self, now):
        """dash-range-start-r1, every try_path call: own odometry since this route OPEN (sum of v_mps * dt, dt <= 0.5 s; None for the rest of
        the opening once v_mps was unknown); a new opening (or lap) clears the track."""
        try:
            if self.rs_tl is None or self.rs_open != self.opened:
                self.rs_open, self.rs_tl, self.rs_od, self.rs_P, self.rs_t, self.rs_x, self.rs_R, self.rs_n, self.rs_strong = self.opened, now, 0., None, None, None, None, 0, False
            else:
                dt = min(max(0., now - self.rs_tl), .5); self.rs_tl = now
                if self.rs_od is not None and self.v_mps is not None: self.rs_od += max(0., float(self.v_mps)) * dt
                else: self.rs_od = None
        except Exception: self.rs_od = None

    def _rs_r0(self):
        c = self.rs; v = self.v_mps
        try: v = float(v) if v is not None and float(v) == float(v) else c['v_ref']
        except Exception: v = c['v_ref']
        return c['a'] + c['b'] * min(max(v, c['v_lo']), c['v_hi'])

    def _rs_step(self, now, L, info, row_y):
        """dash-range-start-r1: one across-row fit before the TURN. True = start the TURN on this frame (start_gate / r23 delay follow).
        Any error -> False with reason RANGE_ERROR (pos_backup still runs)."""
        try:
            c = self.rs; R0 = self._rs_r0(); d = dict(revision=RS_REVISION, R0_cm=round(100. * R0, 1)); info['range'] = d
            first = (L['angle'] < START_DEG and L['n'] >= START_N) or (L['angle'] < START_DEG2 and L['n'] >= START_N2)
            cont = L['angle'] < START_DEG2 and L['n'] >= CONT_N; strong = L['angle'] < START_DEG and L['n'] >= STRONG_N
            R, g = row_range_cm(L)
            if R is not None: d.update(R_cm=round(R, 1), g_deg=round(g, 1))
            if R is None or R > c['r_max_cm'] or abs(g) > c['g_max_deg']:
                info.update(reason='RANGE_NO_FIX' if first or cont else 'ACROSS_NOT_START_ROW'); return False
            P = None if self.rs_od is None else self.rs_od + R / 100.
            recent = self.rs_t is not None and now - self.rs_t <= c['gap_s']
            same = recent and (P is None or self.rs_P is None or abs(P - self.rs_P) * 100. <= c['tol_cm'])
            if same and cont: self.rs_n += 1; self.rs_strong = self.rs_strong or strong
            elif first: self.rs_n = 1; self.rs_strong = strong
            else: info.update(reason='ACROSS_NOT_START_ROW'); return False
            self.rs_P, self.rs_t, self.rs_R = P, now, R
            i = int(np.argmin(L['xs'])); tx, ty = L['xs'][i], L['ys'][i]; p_ = _line_path(AXIS, 359., tx, ty, min(ty, row_y - 1))
            self.rs_x = float(np.interp(row_y, [q[1] / 2 for q in p_], [q[0] / 2 for q in p_]))
            d.update(n_obs=self.rs_n, strong=self.rs_strong)
            if not (self.rs_n >= c['min_obs'] or self.rs_strong): info.update(reason='RANGE_UNCONFIRMED'); return False
            if R / 100. > R0: info.update(reason='RANGE_WAIT'); return False
            return True
        except Exception as e:
            info.update(reason='RANGE_ERROR', range=dict(revision=RS_REVISION, error=type(e).__name__)); return False

    def _rs_dr(self, now, row_ratio, path, info):
        """dash-range-start-r1: no camera start on this frame -> start by odometry when the confirmed track's predicted range <= R0
        (last observation at most dr_max_s ago; not while an r23 delay runs). Never raises."""
        try:
            c = self.rs
            if path is not None or self.turned or self.mode is not None or self.state != 'OPEN' or (self.delay_since is not None and now - self.delay_last <= LOST_S): return path, info
            if self.rs_P is None or self.rs_od is None or self.rs_x is None or not (self.rs_n >= c['min_obs'] or self.rs_strong) or now - self.rs_t > c['dr_max_s']: return path, info
            Rp = self.rs_P - self.rs_od; R0 = self._rs_r0()
            if Rp > R0 or self.rs_R is None or self.rs_R / 100. <= R0: return path, info   # only a track that was still waiting (last seen beyond R0)
            info = dict(info)
            if self.sg and self._sg_block(now, info): self.rs_P = None; self.rs_n = 0; self.rs_strong = False; return path, info   # refused target dropped
            row_y = row_ratio * 719 / 2; x_row = self.rs_x
            self.turned = True; self.follow_since = None; self.across_n = 0
            self.mode, self.last_seen, self.x_prev, self.t_prev = 'TURN', now, x_row, now
            info.update(reason='OK', used=True, mode='TURN', x_row=round(x_row, 1), range=dict(revision=RS_REVISION, dr=True, Rp_cm=round(100. * Rp, 1), R0_cm=round(100. * R0, 1), n_obs=self.rs_n, cam_reason=info.get('reason')))
            return self._path_to(x_row, row_y), info
        except Exception: return path, info

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
        if self.sg: self._sg_tick(now)   # dash-start-r2
        if self.rs: self._rs_tick(now)   # dash-range-start-r1
        if not self.tt.get('pos_backup') and not self.rs: return self._try_path_cam(now, jpeg, row_ratio, normal_err, pre)   # dash-pos-backup-r1: key absent = camera only
        path, info = self._try_path_cam(now, jpeg, row_ratio, normal_err, pre)
        if self.rs: path, info = self._rs_dr(now, row_ratio, path, info)   # dash-range-start-r1: start between observations (odometry)
        if not self.tt.get('pos_backup'): return path, info
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
        if self.sg: info['start_gate_rev'] = SG_REVISION   # dash-start-r2
        P = _pre_get(pre, jpeg)   # dash-offthread-r1: None = compute here (today)
        if pre is not None: info['pre'] = 'USED' if P is not None else 'FALLBACK'
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) if P is None else P['img']
        if img is None: info.update(reason='JPEG_DECODE'); return None, info
        if P is None: img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
        ns_ = not (self.followed or getattr(self, 'tt_along', False))
        L = detect(img, near_shape=ns_) if P is None else P['L'].get(ns_)   # r24; upper-turn-r1: also once an along line was held in the TURN (near block looks tall, as after FOLLOW)
        wr = _WREL is not None and (_WREL['scope'] == 'all' or not self.turned or (self.tmr and self.mode == 'TURN' and not self.followed))   # dash-white-rel-r1; dash-turn-rel-r1
        if wr:
            try:
                if P is not None and 'Lrel' in P: L = P['Lrel'].get(ns_); info['white_rel'] = dict(P['wrel'])
                else: L, info['white_rel'] = detect_rel(img, near_shape=ns_)
            except Exception: info['white_rel'] = dict(revision=WR_REVISION, error=True)   # keep the base L
        if self.turned:
            if wr and _WREL['scope'] == 'all' and (P is None or P.get('sides_rel') is not None):   # dash-white-rel-r1
                try: info['sides'] = {k: dict(v) for k, v in P['sides_rel'].items()} if P is not None else side_lines_rel(img)
                except Exception: info['sides'] = {}
            elif P is not None: info['sides'] = {k: dict(v) for k, v in P['sides'].items()} if P['sides_ok'] else {}
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
            rs_ = self.rs and self.mode is None and not self.turned   # dash-range-start-r1: the range rule replaces the start tests below
            if rs_:
                if not self._rs_step(now, L, info, row_y): return None, info
                if self.sg and self._sg_block(now, info): self.rs_P = None; self.rs_n = 0; self.rs_strong = False; return None, info   # dash-start-r2; a refused target is dropped
            ok3 =(not rs_) and L['angle'] < START3_DEG and L['n'] >= START3_N   # r26
            if ok3:
                if getattr(self, 'n3_t', None) is None or now - self.n3_t > START3_GAP_S: self.n3 = 0
                self.n3 = getattr(self, 'n3', 0) + _RI.w(); self.n3_t = now
            if not rs_ and self.mode is None and not ((L['angle'] < START_DEG and L['n'] >= START_N) or (L['angle'] < START_DEG2 and L['n'] >= START_N2)
                                          or ((self.across_n >= 1 if not _RI.ON else self.across_n > 0) and L['angle'] < START_DEG2 and L['n'] >= CONT_N) or (ok3 and getattr(self, 'n3', 0) >= _RI.thr(START3_FRAMES))):
                self.across_n = 0; info.update(reason='ACROSS_NOT_START_ROW'); return None, info
            if not rs_: self.across_n += _RI.w()
            if not rs_ and self.sg and self.mode is None and not self.turned and self._sg_block(now, info): return None, info   # dash-start-r2
            if not rs_ and self.mode is None and self.across_n < _RI.thr(START_FRAMES) and not (L['angle'] < START_DEG and L['n'] >= STRONG_N) and self.delay_since is None:   # r23: after a delay the row was already confirmed
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
