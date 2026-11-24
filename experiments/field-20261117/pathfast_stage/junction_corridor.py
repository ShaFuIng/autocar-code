"""Perspective corridor tracker (offline candidate, junction-corridor-r1).

One road corridor on flat ground is modelled in the 640x360 mask as two
boundary lines through a common vanishing point on a fixed horizon:
    center(y) = vx + s*(y-yh),  half_width(y) = k*a*(y-yh)/2
yh and a come from the entrance width fit learned from the first three
consecutive frames with bracketing near boundaries (same source as r2).
Each frame searches (vx, x216, k) near the previous state and scores
boundary support minus white pixels inside the corridor. One boundary plus
the width model already fixes the corridor, so the junction mouth (one side
open) keeps the same corridor instead of switching drawing methods.

Inputs per step: current mask, current time, own past state only.
No K, optical flow, future frames, route labels or actuator commands.
"""
import json
from pathlib import Path
import cv2
import numpy as np

REVISION = 'junction-corridor-r1.9'   # + corridor-fast-r1 (float32 search counts; see _counts)
# r1.6: saved current-pose calibration (horizon_y, width_slope), measured by
# corridor_default.py from the straight-entrance calibrations of current-pose
# runs. Used only when no live calibration happened for DEFAULT_AFTER_S.
DEFAULT_FILE = Path(__file__).resolve().with_name('junction_corridor_default.json')


def load_default():
    try:
        d = json.loads(DEFAULT_FILE.read_text(encoding='utf8'))
        return dict(horizon_y=float(d['horizon_y']), width_slope=float(d['width_slope']), revision=d.get('revision'))
    except (OSError, ValueError, KeyError, TypeError):
        return None
SAMPLE_Y = 216
# pathfast-r1: coarse-search counts on the separable candidate grid (CorridorTracker._counts_grid). COUNTS_GRID False = the
# r1.9 _counts path. COUNTS_CHUNK = elements (candidates x rows) per vx slice, 0 = one slice; any value gives the same counts.
# COUNTS_THREADS > 1: the vx slices are counted in that many threads (numpy releases the GIL inside its loops); each slice is
# independent and the results are joined in slice order, so the counts are the same. pathfast.json next to this file may set
# counts_chunk / counts_threads (read at import = at each model load; missing file or key = these defaults).
COUNTS_GRID = True
COUNTS_CHUNK = 100000
COUNTS_THREADS = 1
try:
    _pf = json.loads(Path(__file__).resolve().with_name('pathfast.json').read_text(encoding='utf8'))
    COUNTS_CHUNK = int(_pf.get('counts_chunk', COUNTS_CHUNK)); COUNTS_THREADS = int(_pf.get('counts_threads', COUNTS_THREADS))
except (OSError, ValueError, TypeError, AttributeError):
    pass
_SUM_DTYPE = np.add.reduce(np.zeros((1, 1), bool), -1).dtype
# pathfast2-r1: COUNTS_REMOTE is set only inside corridor_proc.py (the corridor child process) when helper processes are
# configured: the float32 coarse passes hand the upper vx part of the grid to the helpers (each vx slice is independent, the
# slices are joined in vx order -> the same counts; a helper that fails is replaced by the same computation here).
# CUMS_ROWS: the row-wise running sums are computed only for the rows the search / scoring read (the other rows are never
# read); the same int32 values on those rows. DT_CROP: the distance transform is computed on the band from DT_MARGIN rows
# above the first search row to the bottom (see CorridorTracker._dist). Both read from pathfast.json (cums_rows / dt_crop).
COUNTS_REMOTE = None
# pathfast2 C (coarse-skip-r1, CHANGES THE OUTPUT, evaluation candidate, default off): pathfast.json "coarse_skip": n >= 2.
# While the corridor tracks steadily (previous frame BOTH with >= SKIP_MIN_ROWS rows on each side, no coasting, frame gap
# <= SKIP_MAX_GAP_S, worker zone flag absent), only every n-th such frame runs the full coarse window; the others search the
# same 4 px lattice restricted to +-8 px (vx, x216) and +-4 px (dy) around the prediction, then the unchanged refinement.
# The result differs from the full search only when the full-window optimum lies outside that small window.
COARSE_SKIP = None
SKIP_MIN_ROWS, SKIP_MAX_GAP_S = 20, .15
CUMS_ROWS = True
DT_CROP = False
DT_MARGIN = 16
try:
    CUMS_ROWS = bool(_pf.get('cums_rows', CUMS_ROWS)); DT_CROP = bool(_pf.get('dt_crop', DT_CROP))
    COARSE_SKIP = int(_pf['coarse_skip']) if _pf.get('coarse_skip') else None
except (NameError, ValueError, TypeError, AttributeError):
    pass
_POOL = []


def _counts_pool():
    if not _POOL or _POOL[0][1] != COUNTS_THREADS:
        from concurrent.futures import ThreadPoolExecutor
        if _POOL: _POOL[0][0].shutdown(wait=False); _POOL.clear()
        _POOL.append((ThreadPoolExecutor(COUNTS_THREADS, thread_name_prefix='corridor-counts'), COUNTS_THREADS))
    return _POOL[0][0]


HORIZON_RANGE = (-60., 160.)   # r1.7, px in the 640x360 mask
ENTRANCE_JUMP_PX = 12   # between sampled rows 2 px apart; lane lines move ~3 px (experiment value)


def entrance_sample(mask, axis=338):
    # r2 used fixed rows 220-300; the newer camera pose has its near
    # boundaries off-image there. Use the contiguous bracketed row run that
    # starts nearest the vehicle instead (at most the same 80 px span).
    near = []
    for y in range(340, 63, -2):
        row = mask[y - 1:y + 2].any(axis=0).astype(np.int8)
        d = np.diff(np.r_[0, row, 0]); a = np.flatnonzero(d == 1); b = np.flatnonzero(d == -1)
        xs = np.array([(lo + hi - 1) / 2 for lo, hi in zip(a, b) if 2 <= hi - lo <= 100 and lo > 1 and hi < 638])
        left = xs[xs < axis]; right = xs[xs > axis]
        if len(left) and len(right):
            l, r = left.max(), right.min()
            # r1.4: the run must also be continuous in x. With the current pose
            # the near rows are off-image, so the run reaches the junction where
            # an island line replaces a lane boundary (live runs b52c5d53 /
            # c80f2fa6 never calibrated). Keep only the part nearest the car.
            if near and (near[-1][0] - y > 6 or abs(l - near[-1][1]) > ENTRANCE_JUMP_PX or abs(r - near[-1][2]) > ENTRANCE_JUMP_PX): break
            near.append([y, l, r])
            if len(near) == 41: break
    if len(near) < 18: return None
    near = np.array(near)
    wf = np.polyfit(near[:, 0], near[:, 2] - near[:, 1], 1); cf = np.polyfit(near[:, 0], (near[:, 2] + near[:, 1]) / 2, 1)
    residual = float(np.percentile(abs(np.polyval(wf, near[:, 0]) - (near[:, 2] - near[:, 1])), 90))
    return (wf, cf) if wf[0] > 0 and residual < 8 else None


class CorridorTracker:
    # Search windows / weights are experiment constants, not safety limits.
    COAST_S = 1.0
    AXIS_X, AXIS_Y = 338, 340   # vehicle reference (same as entrance calibration)
    OFF_S = .3              # vehicle outside the tracked corridor this long -> recalibrate
    MIN_SIDE_ROWS = 12   # 24 px of rows on one boundary
    K_RANGE = (1., 1.)     # width scale vs entrance calibration (held)
    # r1.5: keep the straight direction at forks. User: at both junctions the
    # correct branch continues the straight approach (junction 1 left side,
    # junction 2 right side). The motion prior follows the current heading, so
    # a yawed car let the vanishing point jump to the diverging branch (live
    # a52c8058, junction 2: vx 250 -> 107 in one frame). A slow reference
    # (EMA, tau VX_REF_TAU s, BOTH support only) adds |vx - vx_ref| / VX_REF_W
    # to the search cost. fork_straight.py: W 4 fixes a52c8058, leaves
    # hand-push 853059f6 unchanged, no new stops/curve take-overs on 5 other
    # live runs; W 2 caused stops. Experiment values, not safety limits.
    VX_REF_W = 4.
    VX_REF_TAU = 1.5
    DY_RANGE = 20          # horizon shift (px); r1 used 10, second 116 s needed 15-26 (investigate-116.json)
    # r1.6: live 1a8abe8f started 7 s before the ramp corner where only the
    # left line is visible; the corridor stayed CALIBRATING (needs a straight
    # with both lines) and could not back up the baseline. After
    # DEFAULT_AFTER_S without a live calibration, start from the saved
    # current-pose default: wide first search around the vehicle axis,
    # accepted only with one boundary on at least DEFAULT_MIN_ROWS rows and
    # the vehicle inside the corridor; otherwise keep calibrating. Normal
    # starts calibrate within ~3 frames and are unchanged. Experiment values.
    DEFAULT_AFTER_S = 1.0
    DEFAULT_MIN_ROWS = 40
    COARSE_W, COARSE_COAST_S = 1.3, .3   # r1.9
    # crossings-worker-r1 (crossings-20261114): a live entrance calibration whose horizon is below cal_h_max px is refused
    # (None = off). 114 live calibrations 11-12..11-14: flat track 55-91 px, all six on the slope 132-160 px; a slope
    # calibration stayed 44 s / 55 s in ac4f and put the curve-2 exit corridor 110-120 px left of the lane.
    cal_h_max = None
    cal_rejects = 0

    def __init__(self, default=None):
        self.samples = []; self.entrance = None; self.state = None; self.velocity = np.zeros(2)
        self.t = None; self.last_seen = None; self.off_since = None
        self.vx_ref = None; self.vx_ref_t = None
        self.default = load_default() if default is None else (default or None)
        self.calib_since = None; self.cold = False

    def _cold_start(self, t):
        d = self.default
        self.entrance = dict(width_fit=[d['width_slope'], -d['width_slope'] * d['horizon_y']], center_fit=None, horizon_y=d['horizon_y'],
                             width_slope=d['width_slope'], capture_t=t, default=True, default_revision=d.get('revision'),
                             source='saved current-pose default (corridor_default.py); no live calibration yet')
        self.state = np.array([float(self.AXIS_X), float(self.AXIS_X), 1.0, 0.0]); self.velocity = np.zeros(2)
        self.last_seen = t; self.cold = True

    def _calibrate(self, mask, t):
        s = entrance_sample(mask)
        self.samples = self.samples + [s] if s is not None else []
        if len(self.samples) < 3: return
        wf = np.median([x[0] for x in self.samples[-3:]], axis=0); cf = np.median([x[1] for x in self.samples[-3:]], axis=0)
        yh = float(-wf[1] / wf[0])
        # r1.7: the worker crashed 5 times (pid-model.log, latest 2026-11-01 14:44)
        # with IndexError: a nearly flat width fit put the horizon at -379 ... -1422
        # px, the search rows became negative. Reject calibrations whose horizon is
        # outside HORIZON_RANGE (live run-start calibrations: 51-72 px).
        if not HORIZON_RANGE[0] <= yh <= HORIZON_RANGE[1]:
            self.samples = []; return
        if self.cal_h_max is not None and yh > self.cal_h_max:   # crossings-worker-r1
            self.samples = []; self.cal_rejects += 1; return
        self.entrance = dict(width_fit=wf.tolist(), center_fit=cf.tolist(), horizon_y=yh, width_slope=float(wf[0]), capture_t=t,
                             source='past/current near bracketing boundaries; not human route labels')
        self.state = np.array([float(np.polyval(cf, yh)), float(np.polyval(cf, SAMPLE_Y)), 1.0, 0.0])
        self.last_seen = t

    def _geometry(self, params, ys):
        # params: vx, x216, k (width scale), dy (horizon shift from pitch)
        vx, x216, k, dy = (params[:, i:i + 1] for i in range(4))
        y0 = self.entrance['horizon_y'] + dy; a = self.entrance['width_slope']
        c = vx + (x216 - vx) * (ys - y0) / (SAMPLE_Y - y0); h = k * a * np.maximum(ys - y0, 0) / 2
        return c, c - h, c + h

    def _score(self, params, dist, cums, mask_rows, ys):
        c, xl, xr = self._geometry(params, ys)
        tol = 2 + .02 * np.maximum(ys - self.entrance['horizon_y'], 0)
        def side(x):
            inside = (x >= 0) & (x <= 639)
            xi = np.clip(np.round(x).astype(int), 0, 639)
            return inside & (dist[mask_rows, xi] <= tol), inside
        sl, il = side(xl); sr, ir = side(xr)
        # White pixels strictly inside the corridor (margin keeps the
        # boundary line's own thickness out). Only visible image span.
        m = np.maximum(6, .12 * (xr - xl))
        lo = np.clip(np.round(xl + m).astype(int), 0, 640); hi = np.clip(np.round(xr - m).astype(int), 0, 640)
        hits = np.where(hi > lo, cums[mask_rows, hi] - cums[mask_rows, lo], 0)
        intrude = hits > 0
        return sl, sr, intrude, il, ir

    def _counts(self, params, dist, cums, mask_rows, ys, f32=True):
        """corridor-fast-r1 (crest-latency-20261105): the per-candidate row counts of _score (left / right support rows,
        intruded rows) for the search only. Same formulas, evaluated in float32 on one boolean support table per row with
        flat gathers (r1.9 _score: float64, ~3900 candidates x 74 rows per coarse pass, ~85 % of the path stage; at the
        crest the stage reached 116 ms on Orin and 99de198b stopped with PID_VISION_STALE). float32 can move a boundary
        across a pixel rounding edge, so equality is not guaranteed by construction: check.py replays recorded masks and
        compares every output with r1.9. Refinement passes (f32=False) use float64 like r1.9. _score (float64) still gives
        the final frame's support and status."""
        ft = np.float32 if f32 else np.float64
        p = params.astype(ft); y = ys.astype(ft)
        vx, x216, k, dy = (p[:, i:i + 1] for i in range(4))
        y0 = ft(self.entrance['horizon_y']) + dy; a = ft(self.entrance['width_slope'])
        d = y - y0
        c = vx + (x216 - vx) * d / (ft(SAMPLE_Y) - y0); h = k * a * np.maximum(d, ft(0)) / ft(2)
        xl = c - h; xr = c + h
        tol = 2 + .02 * np.maximum(ys - self.entrance['horizon_y'], 0)
        R = len(mask_rows)
        ok = (dist[mask_rows] <= tol[:, None]).ravel()
        off = (np.arange(R, dtype=np.int32) * 640)[None, :]
        def side(x):
            xi = np.round(x); np.clip(xi, 0, 639, out=xi)
            return ((x >= 0) & (x <= 639) & ok[xi.astype(np.int32) + off]).sum(1)
        nl = side(xl); nr = side(xr)
        m = np.maximum(ft(6), ft(.12) * (xr - xl))
        lo = np.round(xl + m); np.clip(lo, 0, 640, out=lo); hi = np.round(xr - m); np.clip(hi, 0, 640, out=hi)
        cf = cums[mask_rows].ravel(); off2 = (np.arange(R, dtype=np.int32) * 641)[None, :]
        ni = ((hi > lo) & (cf[hi.astype(np.int32) + off2] > cf[lo.astype(np.int32) + off2])).sum(1)
        return nl, nr, ni

    def _counts_grid(self, g0, g1, g2, g3, dist, cums, mask_rows, ys, f32=True):
        """pathfast-r1: _counts(params, ..., f32) for params = the C-order cartesian product of the axis values g0 (vx),
        g1 (x216), g2 (k), g3 (dy) (what _search builds: its K / DY filter is separable, so the filtered set is the product of the
        filtered axes). Same float32 operations element by element in the same order; quantities that depend only on (k, dy, row)
        are computed once instead of per candidate; in-place temporaries; the redundant hi > lo test is dropped (cf is a running
        sum along the row, so cf[hi] > cf[lo] already implies hi > lo); boolean row counts summed as uint8 (rows < 255) and cast to
        the dtype bool.sum gives. Candidates are processed in vx slices of about COUNTS_CHUNK elements (0 = all at once), optionally in
        COUNTS_THREADS threads; the slicing does not change any value. Integer counts are identical to _counts (checked by test_pathfast.py and the replays)."""
        ft = np.float32 if f32 else np.float64
        R = len(mask_rows)
        y = ys.astype(ft)
        vx_all = g0.astype(ft)[:, None, None, None, None]; x216 = g1.astype(ft)[None, :, None, None, None]
        k = g2.astype(ft)[None, None, :, None, None]; dy = g3.astype(ft)[None, None, None, :, None]
        y0 = ft(self.entrance['horizon_y']) + dy; a = ft(self.entrance['width_slope'])
        d = y - y0
        den = ft(SAMPLE_Y) - y0
        h = k * a * np.maximum(d, ft(0)) / ft(2)
        key = (mask_rows.tobytes(), ys.tobytes(), self.entrance['horizon_y'])
        kc = getattr(self, '_grid_tables', None)   # the two refinement passes of a frame use the same rows: tables reused
        if kc is not None and kc[0] is dist and kc[1] is cums and kc[2] == key:
            ok, cf = kc[3], kc[4]
        else:
            tol = 2 + .02 * np.maximum(ys - self.entrance['horizon_y'], 0)
            ok = (dist[mask_rows] <= tol[:, None]).ravel()
            cf = cums[mask_rows].ravel()
            self._grid_tables = (dist, cums, key, ok, cf)
        off = np.arange(R, dtype=np.intp) * 640   # intp indices: np.take would convert int32 indices to intp anyway
        off2 = np.arange(R, dtype=np.intp) * 641
        per_vx = len(g1) * len(g2) * len(g3) * R
        n0 = len(g0); step = n0 if not COUNTS_CHUNK else max(1, min(n0, COUNTS_CHUNK // max(1, per_vx)))
        u8 = R < 255

        def count(b):
            return b.view(np.uint8).sum(-1, dtype=np.uint8) if u8 else b.sum(-1)

        def side(x):
            b = x >= 0; b &= x <= 639
            xi = np.round(x); np.clip(xi, 0, 639, out=xi)
            xi = xi.astype(np.intp); xi += off
            b &= np.take(ok, xi)
            return count(b)
        def one(s, e=None):
            vx = vx_all[s:s + step if e is None else e]
            c = (x216 - vx) * d
            c /= den
            c += vx
            xl = c - h
            xr = np.add(c, h, out=c) if c.shape == np.broadcast_shapes(c.shape, h.shape) else c + h
            nl = side(xl); nr = side(xr)
            m = xr - xl; m *= ft(.12); np.maximum(ft(6), m, out=m)
            lo = xl; lo += m; np.round(lo, out=lo); np.clip(lo, 0, 640, out=lo)
            hi = xr; hi -= m; np.round(hi, out=hi); np.clip(hi, 0, 640, out=hi)
            li = lo.astype(np.intp); li += off2; hj = hi.astype(np.intp); hj += off2
            ni = count(np.take(cf, hj) > np.take(cf, li))
            return nl.reshape(-1), nr.reshape(-1), ni.reshape(-1)
        starts = range(0, n0, step)
        rem = COUNTS_REMOTE
        if rem is not None and f32 and n0 >= 2:   # pathfast2-r1: vx [0, hs) here, vx [hs, n0) in the helper process(es)
            hs = rem.split(n0)
            tok = rem.send(self, g0[hs:], g1, g2, g3, mask_rows, ys)
            res = [one(s, min(s + step, hs)) for s in range(0, hs, step)]
            res += rem.recv(tok, lambda a, b: [one(s, min(s + step, b)) for s in range(a, b, step)], hs)
        else:
            res = list(_counts_pool().map(one, starts)) if COUNTS_THREADS > 1 and len(starts) > 1 else [one(s) for s in starts]
        cat = lambda i: (np.concatenate([r[i] for r in res]) if len(res) > 1 else res[0][i]).astype(_SUM_DTYPE)
        return cat(0), cat(1), cat(2)

    def _skip_ok(self, t, coasting):
        """pathfast2 C: True on the skipped frames (see COARSE_SKIP). _skip_prev is set at the end of _step."""
        pv = getattr(self, '_skip_prev', None)
        ok = (pv is not None and pv[0] == 'BOTH' and min(pv[1], pv[2]) >= SKIP_MIN_ROWS and self.last_seen == pv[3]
              and 0 < t - pv[3] <= SKIP_MAX_GAP_S and getattr(self, 'zone_hint', None) is None)
        n = (getattr(self, '_skip_n', 0) + 1) if ok else 0
        self._skip_n = n
        return ok and n % COARSE_SKIP != 0

    @staticmethod
    def _dist(mask, rows):
        """pathfast2-r1 (DT_CROP): the corridor reads the distance map only as 'dist <= tol' on the search rows (tol <= 2 + 0.02 * 420
        = 10.4 px). A mask pixel more than 10.4 / 0.955 < 11 rows above a search row cannot be within tol of it (the 3x3 chamfer
        steps cost >= 0.955 per row), and a shortest chamfer path between two pixels stays inside their bounding box. So on the
        band from DT_MARGIN (16) rows above the first search row down to the bottom, every value <= tol is the same as in the
        full transform and every value > tol stays > tol. Rows above the band are never read (left uninitialised)."""
        r0 = max(0, int(rows.min()) - DT_MARGIN) if len(rows) else 0
        if r0 == 0 or mask.ndim != 2: return cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 3)
        dist = np.empty(mask.shape, np.float32)
        dist[r0:] = cv2.distanceTransform((~mask[r0:]).astype(np.uint8), cv2.DIST_L2, 3)
        return dist

    def _search(self, dist, cums, rows, ys, center, spans):
        grids = [np.arange(c - sp, c + sp + 1e-6, st) if sp else np.array([c]) for c, (sp, st) in zip(center, spans)]
        params = np.stack(np.meshgrid(*grids, indexing='ij'), -1).reshape(-1, 4)
        params = params[(params[:, 2] >= self.K_RANGE[0]) & (params[:, 2] <= self.K_RANGE[1]) & (np.abs(params[:, 3]) <= self.DY_RANGE)]
        # float32 only for the coarse passes (thousands of candidates); the refinement passes (<= 25 candidates) keep float64
        g2 = grids[2][(grids[2] >= self.K_RANGE[0]) & (grids[2] <= self.K_RANGE[1])]; g3 = grids[3][np.abs(grids[3]) <= self.DY_RANGE]
        if COUNTS_GRID and len(params) and len(params) == len(grids[0]) * len(grids[1]) * len(g2) * len(g3):   # pathfast-r1
            nl, nr, ni = self._counts_grid(grids[0], grids[1], g2, g3, dist, cums, rows, ys, f32=len(params) > 200)
        else:
            nl, nr, ni = self._counts(params, dist, cums, rows, ys, f32=len(params) > 200)
        q = self.prediction
        motion = (np.abs(params[:, 0] - q[0]) / 25 + np.abs(params[:, 1] - q[1]) / 12 + np.abs(params[:, 2] - q[2]) * 20
                  + np.abs(params[:, 3] - q[3]) / 3 + np.abs(params[:, 3]) / 10 + np.abs(params[:, 2] - 1) * 5)
        if self.vx_ref is not None: motion = motion + np.abs(params[:, 0] - self.vx_ref) / self.VX_REF_W
        # Single-side support counts less than paired support; a lone line
        # with a hypothesised partner far off-image cannot dominate.
        # Row counts are normalised to the 2 px row grid so a sparser coarse
        # pass keeps the same balance against the motion prior (r1.1 had this
        # ratio inverted: coarse evidence weighed 1/4 of fine; found by Codex).
        scale = (ys[1] - ys[0]) / 2
        score = scale * (nl + nr + 2 * np.minimum(nl, nr) - 2.5 * ni) - motion
        best = int(np.argmax(score))
        return params[best], float(score[best])

    def step(self, mask, t):
        out = self._step(mask, t)
        if out['status'] == 'BOTH' and out['params']:
            vx = out['params']['vx']
            if self.vx_ref is None or self.vx_ref_t is None or not 0 < t - self.vx_ref_t <= .4: self.vx_ref = vx
            else: self.vx_ref += (t - self.vx_ref_t) / (self.VX_REF_TAU + t - self.vx_ref_t) * (vx - self.vx_ref)
            self.vx_ref_t = t
        elif out['status'] in ('CALIBRATING', 'LOST', 'OFF_CORRIDOR'):
            self.vx_ref = None; self.vx_ref_t = None
        out['diag'] = dict(out.get('diag') or {}, vx_ref=self.vx_ref)
        return out

    def _step(self, mask, t):
        if self.t is not None and not 0 < t - self.t <= .4:
            self.state = None; self.samples = []; self.entrance = None
        self.t = t
        if self.entrance is None:
            self._calibrate(mask, t)
            if self.entrance is None:
                self.calib_since = t if self.calib_since is None else self.calib_since
                if self.default is None or t - self.calib_since < self.DEFAULT_AFTER_S:
                    return self._result('CALIBRATING', None)
                self._cold_start(t)
        self.calib_since = None
        yh = self.entrance['horizon_y']
        ys = np.arange(max(int(np.ceil(yh)) + 8, 0), 358, 2).astype(float)
        rows = ys.astype(int)
        dist = self._dist(mask, rows) if DT_CROP else cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 3)
        if CUMS_ROWS and mask.shape == (360, 640) and len(rows):   # pathfast2-r1: only the rows that are read
            cums = np.zeros((360, 641), np.int32); cums[rows, 1:] = np.cumsum(mask[rows], 1, dtype=np.int32)
        else:
            cums = np.concatenate([np.zeros((360, 1), np.int32), np.cumsum(mask, 1, dtype=np.int32)], 1)
        coasting = t - self.last_seen
        self.prediction = self.state + np.r_[self.velocity * .5, 0, 0]
        widen = 3. if self.cold else 1 + min(2., coasting / .5)
        # Coarse heading/position/pitch jointly (a fixed pitch in the coarse
        # pass left the tracker on a local optimum at a junction mouth), then
        # refine and polish. Width scale stays at the entrance calibration:
        # freeing it let a flared mouth widen the corridor and keep it wide.
        if self.cold:
            # r1.8: the r1.6 cold start searched the 3x window at the normal 4 px
            # grid: one frame took 223 ms on the laptop, more on Orin, and live
            # cf6766f0 stopped with PID_VISION_STALE at 1.49 s. Coarse 12 px grid,
            # then a medium 4 px pass, then the usual refinement.
            p, _ = self._search(dist, cums, rows[::2], ys[::2], self.prediction, [(60 * widen, 12), (48 * widen, 12), (0, 0), (8, 8)])
            p, _ = self._search(dist, cums, rows[::2], ys[::2], p, [(12, 4), (12, 4), (0, 0), (4, 2)])
        else:
            # r1.9 (user 2026-11-04, live d903f981 / 1b619acb stopped with PID_VISION_STALE): the coarse window grows with the
            # coasting time (widen 1 -> 3) at a fixed 4 px grid, so the grid count grows ~widen^2 (3875 -> 33215 candidates):
            # consecutive COAST frames took 67 -> 89 -> 129 -> 160 ms in the path stage (laptop replay 34 -> 96 ms) and the
            # next frame came after the 0.4 s limit. After COARSE_COAST_S without support the grid step grows with the window
            # (count held at the COARSE_W level), then a medium 4 px pass around the best point. Normal tracking (frame gaps
            # up to ~0.26 s in these runs; a 0.25 s trigger changed tracked frames) keeps the old search. Experiment value (coast-latency-20261104).
            st = 4. * widen / self.COARSE_W if coasting > self.COARSE_COAST_S else 4.
            if COARSE_SKIP and self._skip_ok(t, coasting):   # pathfast2 C (off unless pathfast.json coarse_skip)
                p, _ = self._search(dist, cums, rows[::2], ys[::2], self.prediction, [(8, 4), (8, 4), (0, 0), (4, 4)])
            else:
                p, _ = self._search(dist, cums, rows[::2], ys[::2], self.prediction, [(60 * widen, st), (48 * widen, st), (0, 0), (8, 4)])
            if st > 4.: p, _ = self._search(dist, cums, rows[::2], ys[::2], p, [(st, 4), (st, 4), (0, 0), (4, 2)])
        p, _ = self._search(dist, cums, rows, ys, p, [(4, 2), (4, 2), (0, 0), (2, 1)])
        p, score = self._search(dist, cums, rows, ys, p, [(1, 1), (1, 1), (0, 0), (0, 0)])
        sl, sr, intrude, il, ir = self._score(p[None], dist, cums, rows, ys)
        sl, sr, intrude = sl[0], sr[0], intrude[0]
        nl, nr = int(sl.sum()), int(sr.sum())
        if COARSE_SKIP: self._skip_prev = ('BOTH' if min(nl, nr) >= self.MIN_SIDE_ROWS else None, nl, nr, t)   # pathfast2 C
        if min(nl, nr) >= self.MIN_SIDE_ROWS: status = 'BOTH'
        elif max(nl, nr) >= self.MIN_SIDE_ROWS: status = 'SINGLE'
        else: status = None
        if self.cold:
            self.cold = False
            _, xl0, xr0 = (g[0] for g in self._geometry(p[None], np.array([float(self.AXIS_Y)])))
            if status is None or max(nl, nr) < self.DEFAULT_MIN_ROWS or not xl0[0] <= self.AXIS_X <= xr0[0]:
                self.entrance = None; self.state = None; self.samples = []
                return self._result('CALIBRATING', None, dict(left_rows=nl, right_rows=nr, default_rejected=True))
        if status:
            self.velocity = .5 * self.velocity + .5 * (p[:2] - self.state[:2]); self.state = p; self.last_seen = t
        else:
            self.velocity *= .5
            if t - self.last_seen > self.COAST_S:
                return self._result('LOST', None, dict(left_rows=nl, right_rows=nr))
            status = 'COAST'
        diag = dict(left_rows=nl, right_rows=nr, intrude_rows=int(intrude.sum()), score=score,
                    calibration='default' if self.entrance.get('default') else 'live')
        # The vehicle must be inside the corridor it tracks. After a route
        # split (38bc8966 / d2a07709: car went left of the island, corridor
        # kept the straight exit) the corridor's near boundary lies on the
        # vehicle's side; then it is not a target for this vehicle.
        _, xl, xr = (g[0] for g in self._geometry(self.state[None], np.array([float(self.AXIS_Y)])))
        if not xl[0] <= self.AXIS_X <= xr[0]:
            self.off_since = t if self.off_since is None else self.off_since
            diag.update(axis_left=float(xl[0]), axis_right=float(xr[0]))
            if t - self.off_since >= self.OFF_S:
                self.state = None; self.samples = []; self.entrance = None; self.off_since = None
                self.velocity = np.zeros(2)
            return self._result('OFF_CORRIDOR', None, diag)
        self.off_since = None
        support = dict(ys=ys, left=sl, right=sr, intrude=intrude) if status != 'COAST' else None
        return self._result(status, self.state.copy(), diag, support)

    def _result(self, status, params, diag=None, support=None):
        out = dict(revision=REVISION, status=status, path=[], support=[], params=None, boundaries=None, diag=diag or {},
                   entrance=self.entrance, future_frames=False)
        if params is None: return out
        yh = self.entrance['horizon_y'] + params[3]
        ys = np.arange(340, int(np.ceil(yh)) + 11, -4).astype(float)
        c, xl, xr = (g[0] for g in self._geometry(params[None], ys))
        code = np.zeros(len(ys), int)
        if support is not None and len(ys):   # pathfast-r1: same per-row any() as a table (rows x support rows)
            near = np.abs(support['ys'][None, :] - ys[:, None]) <= 6
            code[:] = (near & support['left'][None, :]).any(1).astype(int) + (near & support['right'][None, :]).any(1).astype(int)
        keep = (c >= -320) & (c <= 960)
        out.update(params=dict(vx=float(params[0]), x216=float(params[1]), k=float(params[2]), dy=float(params[3])),
                   path=[[float(x * 2), int(y * 2)] for x, y in zip(c[keep], ys[keep])],
                   support=code[keep].tolist(),
                   boundaries=dict(left=[[float(x * 2), int(y * 2)] for x, y in zip(xl, ys)],
                                   right=[[float(x * 2), int(y * 2)] for x, y in zip(xr, ys)]))
        return out
