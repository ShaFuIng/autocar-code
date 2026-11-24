"""upper-edges-r1 (upper-edges-20261115, offline candidate): the two solid edge lines of the upper deck as a lateral + heading
reference after the dashed left turn.

Two parts, both pure (numpy only, no car I/O):
 detect(mask, H, cfg, hint_deg)  - WORKER side (live_model_worker / junction_live, next to the V4 mask). Row scan of the mask
     (rows row0..359 step row_step), run centres projected to the ground with the deck-level homography (image px -> ground cm,
     x right, z forward of the front tyre), kept in z_min..z_max / |x| <= x_max / ground run width <= w_max_cm. One direction search
     over the weighted points (weight = ground length per scanned row, so a far line is not out-voted by near pixels): for each
     angle th (deg, + = line points right of the car axis) the perpendicular offsets d = x cos th - z sin th are binned; the angle
     with the sharpest histogram (sum of squared bin weights) is the common direction of the parallel deck lines. Peaks of that
     histogram = lines; each is refitted (weighted least squares in the rotated frame) and described by d (cm, + right, at the car
     origin = the front tyre), ang (deg), along extent, length, coverage (fraction of 4 cm along-bins with points; dashes ~0.4-0.6,
     solid >= 0.7), rms (cm). Returns a small dict (json-able, a few hundred bytes).
 Estimator (CONTROL side, class UpperEdges): identity of the edge lines with the gyro-predicted deck direction and the previous
     lateral estimate; output per frame: deck heading in the gyro yaw frame (D = yaw - ang), lateral offset of the car from the
     dashed line (cm, + = car right of it), quality flags, and an 'aligned' flag.
Every number is a key (upper_edges in upper_single.json for the control side; the worker reads upper_edges_worker.json at model load).
Experiment values from upper-edges-20261115; not a verified safety limit."""
import math
import numpy as np
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'upper-edges-r1'

DET_DEFAULTS = dict(row0=100, row_step=2, z_min=25., z_max=260., x_max=200., w_min_px=2, w_max_cm=20., th_span_deg=40., th_step_deg=1.,
                    hint_span_deg=15., bin_cm=3., peak_min_cm=20., peak_sep_cm=15., peak_sep_deg=6., band_cm=7., max_lines=6, wcap=25.,
                    lines=True, max_pts=160, corr_c0=.9906, corr_c1=1.347e-3, corr_c2=7.06e-6, corr_zc=-12.)   # deck range correction (funnel.py / corr fit, 46 passes)


def _runs(sub):
    """Row-wise runs of a boolean array (R x W): (row index, x start, x end inclusive)."""
    R, Wd = sub.shape
    p = np.zeros((R, Wd + 2), np.int8); p[:, 1:-1] = sub
    df = np.diff(p, axis=1)
    rs, xs = np.nonzero(df == 1); re_, xe = np.nonzero(df == -1)
    # np.nonzero is row-major: starts and ends pair up in order
    return rs, xs, xe - 1


def _proj(H, x, y):
    w = H[2, 0] * x + H[2, 1] * y + H[2, 2]
    return (H[0, 0] * x + H[0, 1] * y + H[0, 2]) / w, (H[1, 0] * x + H[1, 1] * y + H[1, 2]) / w


def detect(mask, H, cfg=None, hint_deg=None):
    """mask: 2-D array (360 x 640, nonzero = line). H: 3x3 image->ground (cm). hint_deg: expected line angle (deg, + right) or None
    (full search). Returns dict(rev, n_pts, th, lines=[...]) or dict(rev, n_pts, th=None, lines=[]) ."""
    c = dict(DET_DEFAULTS)
    if cfg: c.update({k: v for k, v in cfg.items() if k in DET_DEFAULTS})
    m = np.asarray(mask)
    if m.shape != (360, 640):
        import cv2
        m = cv2.resize(m.astype(np.uint8), (640, 360), interpolation=cv2.INTER_NEAREST)
    rows = np.arange(int(c['row0']), 360, int(c['row_step']))
    sub = m[rows] > 0
    ri, x0, x1 = _runs(sub)
    out = dict(rev=REVISION, n_pts=0, th=None, lines=[])
    if len(ri) == 0: return out
    wpx = x1 - x0 + 1
    k = wpx >= int(c['w_min_px']); ri, x0, x1 = ri[k], x0[k].astype(float), x1[k].astype(float)
    y = rows[ri].astype(float); xc = .5 * (x0 + x1)
    gx, gz = _proj(H, xc, y)
    ax, az = _proj(H, x0 - .5, y); bx, bz = _proj(H, x1 + .5, y)
    gw = np.hypot(bx - ax, bz - az)
    _, gz2 = _proj(H, xc, y + float(c['row_step']))
    k = (gz >= c['z_min']) & (gz <= c['z_max']) & (np.abs(gx) <= c['x_max']) & (gw <= c['w_max_cm']) & (gz > 0)
    gx, gz = gx[k], gz[k]
    wt = np.clip(np.abs(gz - gz2[k]), .3, float(c['wcap']))
    if c['corr_c1'] or c['corr_c2'] or c['corr_c0'] != 1.:   # range-dependent scale on the deck: s(z) = c0 + c1 (z-100) + c2 (z-100)^2 about the camera foot zc
        dz = gz - 100.; sc = np.clip(c['corr_c0'] + c['corr_c1'] * dz + c['corr_c2'] * dz * dz, .8, 1.6)
        gx = gx / sc; gz = c['corr_zc'] + (gz - c['corr_zc']) / sc; wt = wt / sc
    out['n_pts'] = int(len(gx))
    if len(gx) < 6: return out
    ix = np.arange(len(gx)) if len(gx) <= int(c['max_pts']) else np.linspace(0, len(gx) - 1, int(c['max_pts'])).astype(int)
    out['pts'] = [[round(float(a), 1), round(float(b), 1), round(float(w), 1)] for a, b, w in zip(gx[ix], gz[ix], wt[ix] * len(gx) / len(ix))]
    if not c['lines']: return out   # points only (all the control side needs); the line search below is for logs / contact sheets
    if hint_deg is None: ths = np.arange(-c['th_span_deg'], c['th_span_deg'] + 1e-9, c['th_step_deg'])
    else: ths = np.arange(hint_deg - c['hint_span_deg'], hint_deg + c['hint_span_deg'] + 1e-9, c['th_step_deg'])
    tr = np.radians(ths)
    D = np.outer(gx, np.cos(tr)) - np.outer(gz, np.sin(tr))          # N x A
    bw = float(c['bin_cm']); lo = -c['x_max'] - 60.; nb = int((2 * c['x_max'] + 120.) / bw) + 1
    bi = np.clip(((D - lo) / bw).astype(int), 0, nb - 1)
    A = len(ths); hist = np.zeros((A, nb))
    np.add.at(hist, (np.broadcast_to(np.arange(A), bi.shape), bi), wt[:, None])
    # 2-D peaks in (angle, offset): the deck lines are NOT parallel (funnel toward the downhill), so every line gets its own angle
    hs = hist.copy(); hs[:, 1:-1] = .25 * hist[:, :-2] + .5 * hist[:, 1:-1] + .25 * hist[:, 2:]
    hs3 = hs.copy(); hs3[:, 1:-1] = hs[:, :-2] + hs[:, 1:-1] + hs[:, 2:]
    score = (hist ** 2).sum(axis=1); a = int(np.argmax(score)); out['th'] = round(float(ths[a]), 2)
    nsa = max(1, int(round(c['peak_sep_deg'] / c['th_step_deg']))); nsd = max(1, int(round(c['peak_sep_cm'] / bw)))
    flat = np.argsort(hs3, axis=None)[::-1]; pk = []
    for f in flat[:4000]:
        ia, ib = divmod(int(f), nb); v = hs3[ia, ib]
        if v < c['peak_min_cm']: break
        if any(abs(ia - qa) <= nsa and abs(ib - qb) <= nsd for qa, qb in pk): continue
        pk.append((ia, ib))
        if len(pk) >= int(c['max_lines']) + 2: break
    lines = []; used = np.zeros(len(gx), bool)
    for ia, ib in pk:
        th = float(ths[ia]); t = math.radians(th); ct, st = math.cos(t), math.sin(t)
        d = gx * ct - gz * st; u = gx * st + gz * ct; dc = lo + (ib + .5) * bw
        sel = (np.abs(d - dc) <= c['band_cm']) & ~used
        if sel.sum() < 3: continue
        dd, uu, ww = d[sel], u[sel], wt[sel]
        L = float(ww.sum())
        if L < c['peak_min_cm']: continue
        um = float(np.average(uu, weights=ww)); dm = float(np.average(dd, weights=ww))
        var = float(np.average((uu - um) ** 2, weights=ww))
        q = float(np.average((uu - um) * (dd - dm), weights=ww) / var) if var > 25. else 0.
        q = max(-.1, min(.1, q))
        res = dd - (dm + q * (uu - um)); rms = float(math.sqrt(np.average(res ** 2, weights=ww)))
        ang = th + math.degrees(math.atan(q))
        d0 = (dm - q * um) * math.cos(math.atan(q))
        ulo, uhi = float(uu.min()), float(uu.max())
        nbin = max(1, int((uhi - ulo) / 4.) + 1); occ = np.zeros(nbin, bool); occ[np.clip(((uu - ulo) / 4.).astype(int), 0, nbin - 1)] = True
        used |= sel
        lines.append(dict(d=round(d0, 1), ang=round(ang, 2), u0=round(ulo, 1), u1=round(uhi, 1), len=round(L, 1), cov=round(float(occ.mean()), 2),
                          rms=round(rms, 2), n=int(sel.sum())))
    lines.sort(key=lambda l: l['d'])
    out['lines'] = lines[:int(c['max_lines'])]
    return out


# ------------------------------------------------------------------------------------------------------------------------------------
# CONTROL side: funnel map matching (class UpperEdges, pure, vectorised).
# Funnel model of the upper deck (cm; x = right of the dashed line / its extension, y = along the deck relative to the front-tyre
# position at the lap_pos DOWN anchor; analysis/funnel.py over 46 passes 11-14 + 11-15, near-range points, deck range correction).
# Line CENTRES (solid lines ~5 cm wide). Left: straight -61 until y -50, chamfer to -38 at y -30, mouth -37. Right: 64 until y -220,
# narrowing (~ -11 deg) to 46 at y -100, 44 at -60, chamfer to 30 at -30, mouth 29. Mouth centre ~ -4, width ~66 (centres, ~60 inner).
FUNNEL_L = ((-400., -61.), (-50., -61.), (-30., -38.), (40., -37.))
FUNNEL_R = ((-400., 64.), (-220., 64.), (-180., 61.), (-100., 46.), (-60., 44.), (-50., 42.7), (-30., 30.), (40., 29.))
_FL = (np.array([p[0] for p in FUNNEL_L]), np.array([p[1] for p in FUNNEL_L]))
_FR = (np.array([p[0] for p in FUNNEL_R]), np.array([p[1] for p in FUNNEL_R]))
EST_DEFAULTS = dict(k_crest_m=3.58, y_valid_max_cm=-15., gate_cm=12., gate0_cm=25., huber_cm=4., min_pts=6, min_span_cm=30., phi_prior_sd_deg=8.,
                    x_prior_sd_cm=40., y_prior_sd_cm=15., d_alpha=.3, d_gate_deg=8., d_min_n=3, dash_end_cm=-100., rms_max_cm=6., jump_deg=12.,
                    late_y_cm=-90., use_y_max_cm=-60., x_alpha=.6, dr_max_m=1.0, aligned_head_deg=3., aligned_lat_cm=6., aligned_gz_dps=8., aligned_n=2,
                    mouth_x_cm=-4., mouth_from_cm=-110., mouth_to_cm=-40.)
NULLABLE_EST = ('y_prior_sd_cm', 'late_y_cm', 'dr_max_m', 'mouth_x_cm', 'use_y_max_cm')


def funnel_x(side, y):
    P = _FL if side == 'L' else _FR
    return np.interp(y, P[0], P[1])


def load_h(path=None):
    """(H, Hinv) from camera_ground_calibration.json next to this file (web/)."""
    import json
    from pathlib import Path
    d = json.loads(Path(path or Path(__file__).with_name('camera_ground_calibration.json')).read_text(encoding='utf8'))
    H = np.asarray(d['homography_image_to_ground_cm'], float)
    if H.shape != (3, 3) or not np.isfinite(H).all(): raise ValueError('bad homography')
    return H, np.linalg.inv(H)


def dash_x_cm(H, a, b):
    """car lateral (cm, + = car right of the line) from a dash-route image line x = a*y + b, using the calibrated range 30-140 cm."""
    ys = np.arange(150., 359., 8.); xs = a * ys + b; gx, gz = _proj(H, xs, ys); k = (gz > 30) & (gz < 140)
    if k.sum() < 3: return None
    s, x0 = np.polyfit(gz[k], gx[k], 1); return float(-x0 * math.cos(math.atan(s)))


class UpperEdges:
    """Per-frame pose of the car on the deck from the worker's points (detect()['pts']). step() returns a dict:
    ok (measured this frame), use (ok, or dead-reckoned within dr_max_m of the last fix with the deck direction established), phi (deg,
    heading relative to the deck, left +), x (cm, front tyre right of the dashed line), D (deck direction in the yaw frame), D_ok
    (>= d_min_n two-sided fits so far), y (cm to the crest, refined), xref, aligned.
    Inputs: det (detect() output or None), yaw (deg), y_rel_m (lap_pos since DASHe - k_crest; None = unknown -> no measurement),
    D_prior (deck direction: dash-direction median or target h; initialises D), odo_m (wheel distance, m), x_dash_cm (car lateral from
    this frame's dash fit: a fix of x), gz (deg/s, aligned flag only)."""

    def __init__(self, cfg=None):
        self.c = dict(EST_DEFAULTS); self.error = None
        try:
            for k, v in (cfg or {}).items():
                if k not in EST_DEFAULTS: continue
                if v is None and k in NULLABLE_EST: self.c[k] = None
                else: self.c[k] = float(v)
        except Exception as e: self.error = '%s:%s' % (type(e).__name__, e)
        self.reset()

    def reset(self):
        self.D = None; self.x = None; self.odo = None; self.n_al = 0; self.last = None; self.n_both = 0; self.fix_odo = None; self.y_off = 0.

    def ref_x(self, y_cm):
        c = self.c
        if y_cm is None or c['mouth_x_cm'] is None: return 0.
        f = min(1., max(0., (y_cm - c['mouth_from_cm']) / max(1., c['mouth_to_cm'] - c['mouth_from_cm'])))
        return f * c['mouth_x_cm']

    def _resid(self, x, ph, yv, GX, GZ, S):
        pr = math.radians(ph); cs, sn = math.cos(pr), math.sin(pr)
        wx = x + GX * cs - GZ * sn
        wy = (yv + GX * sn + GZ * cs) if yv is not None else np.full(len(GX), -150.)
        m = np.where(S, funnel_x('L', wy), funnel_x('R', wy))
        return wx - m, wy, cs, sn

    def step(self, det, yaw, y_rel_m=None, D_prior=None, odo_m=None, x_dash_cm=None, gz=None):
        c = self.c
        if self.D is None and D_prior is not None: self.D = float(D_prior)
        if yaw is None or self.D is None: return None
        phi_p = yaw - self.D
        if self.x is not None and odo_m is not None and self.odo is not None:
            self.x += -(odo_m - self.odo) * 100. * math.sin(math.radians(phi_p))
        if odo_m is not None: self.odo = odo_m
        if x_dash_cm is not None: self.x = float(x_dash_cm); self.fix_odo = odo_m
        x_p = 0. if self.x is None else self.x
        y_car = None if y_rel_m is None else 100. * float(y_rel_m)
        yv = None if y_car is None else y_car + self.y_off
        y_rep = yv
        pts = (det or {}).get('pts')
        if not pts: return self._out(None, phi_p, x_p, y_rep, gz, why='NO_POINTS')
        if yv is None: return self._out(None, phi_p, x_p, y_rep, gz, why='NO_Y')   # no along position (lap_pos not anchored at DASHe): the funnel cannot be placed
        A_ = np.asarray(pts, float).reshape(-1, 3); GX0, GZ0, W0 = A_[:, 0], A_[:, 1], A_[:, 2]
        x, ph = x_p, phi_p; sel = None; isL = None
        for it in range(2):   # identity at the current pose -> Gauss-Newton (x, phi[, y]); twice
            pr = math.radians(ph); cs, sn = math.cos(pr), math.sin(pr)
            wx = x + GX0 * cs - GZ0 * sn
            wy = (yv + GX0 * sn + GZ0 * cs) if yv is not None else np.full(len(GX0), -150.)
            keep = wy <= c['y_valid_max_cm']
            rL = np.abs(wx - funnel_x('L', wy)); rR = np.abs(wx - funnel_x('R', wy)) if yv is not None else np.full(len(wx), 1e9)
            rD = np.where(wy <= c['dash_end_cm'], np.abs(wx), 1e9)
            gate = c['gate_cm'] if it else max(c['gate_cm'], c['gate0_cm'])
            isL = keep & (rL < gate) & (rL < rR) & (rL < rD); isR = keep & (rR < gate) & (rR < rL) & (rR < rD)
            sel = isL | isR
            if sel.sum() < c['min_pts']: break
            S = isL[sel]; GX = GX0[sel]; GZ = GZ0[sel]; WW = W0[sel]
            if float(GZ.max() - GZ.min()) < c['min_span_cm']: break
            for _ in range(2):
                r, wy2, cs, sn = self._resid(x, ph, yv, GX, GZ, S)
                J = np.column_stack([np.ones(len(GX)), math.radians(1.) * (-GX * sn - GZ * cs)])
                fy = yv is not None and c['y_prior_sd_cm'] is not None
                if fy:   # along-position refinement from the funnel shape (narrowing / chamfers); weak prior on the lap_pos value
                    sl = (np.where(S, funnel_x('L', wy2 + 2.), funnel_x('R', wy2 + 2.)) - np.where(S, funnel_x('L', wy2 - 2.), funnel_x('R', wy2 - 2.))) / 4.
                    J = np.column_stack([J, -sl])
                w = WW * np.minimum(1., c['huber_cm'] / np.maximum(1e-6, np.abs(r))); w = w / max(1e-9, float(w.mean()))
                A = (J * w[:, None]).T @ J; g = (J * w[:, None]).T @ r
                A[0, 0] += 1. / c['x_prior_sd_cm'] ** 2; A[1, 1] += 1. / c['phi_prior_sd_deg'] ** 2
                g[0] += (x - x_p) / c['x_prior_sd_cm'] ** 2; g[1] += (ph - phi_p) / c['phi_prior_sd_deg'] ** 2
                if fy: A[2, 2] += 1. / c['y_prior_sd_cm'] ** 2; g[2] += (yv - y_car) / c['y_prior_sd_cm'] ** 2
                dl = np.linalg.solve(A, g); x -= float(dl[0]); ph -= float(dl[1])
                if fy: yv -= float(dl[2])
        if sel is None or sel.sum() < c['min_pts']: return self._out(None, phi_p, x_p, y_rep, gz, why='FEW_POINTS')
        S = isL[sel]; GX = GX0[sel]; GZ = GZ0[sel]; nL = int(S.sum()); nR = int(len(S) - nL); span = float(GZ.max() - GZ.min())
        if span < c['min_span_cm']: return self._out(None, phi_p, x_p, y_rep, gz, why='SHORT', nL=nL, nR=nR)
        if yv is not None and c['late_y_cm'] is not None and max(yv, y_car) > c['late_y_cm'] and (nL == 0 or nR == 0): return self._out(None, phi_p, x_p, y_rep, gz, why='ONE_SIDE_LATE', nL=nL, nR=nR)
        r, _, _, _ = self._resid(x, ph, yv, GX, GZ, S); rms = float(math.sqrt(np.mean(np.minimum(r ** 2, 400.))))
        if rms > c['rms_max_cm'] or abs(ph - phi_p) > c['jump_deg']: return self._out(None, phi_p, x_p, y_rep, gz, why='BAD_FIT', nL=nL, nR=nR, rms=rms)
        Dm = yaw - ph
        if nL and nR and abs(Dm - self.D) <= c['d_gate_deg']: self.D += _RI.alpha(c['d_alpha']) * (Dm - self.D); self.n_both += _RI.w()
        self.x = x_p + _RI.alpha(c['x_alpha']) * (x - x_p); self.fix_odo = odo_m
        if yv is not None and y_car is not None: self.y_off += _RI.alpha(.5) * ((yv - y_car) - self.y_off)
        return self._out(True, yaw - self.D, self.x, yv, gz, nL=nL, nR=nR, rms=rms, span=span, Dm=Dm, phi_meas=ph, x_meas=x)

    def _out(self, ok, ph, x, y_cm, gz, why=None, **kw):
        c = self.c
        D_ok = self.n_both >= _RI.thr(c['d_min_n'])
        dr_m = None if (self.fix_odo is None or self.odo is None) else self.odo - self.fix_odo
        use = bool(ok) or (D_ok and self.x is not None and dr_m is not None and c['dr_max_m'] is not None and dr_m <= c['dr_max_m'])
        if use and not ok and c['use_y_max_cm'] is not None and y_cm is not None and y_cm > c['use_y_max_cm']: use = False
        o = dict(rev=REVISION, ok=bool(ok), use=bool(use), why=why, phi=round(ph, 2), x=round(x, 1), D=None if self.D is None else round(self.D, 2),
                 D_ok=D_ok, y=None if y_cm is None else round(y_cm, 1), xref=round(self.ref_x(y_cm), 1), dr_m=None if dr_m is None else round(dr_m, 3))
        o.update({k: (round(v, 2) if isinstance(v, float) else v) for k, v in kw.items()})
        if use:
            al = abs(ph) <= c['aligned_head_deg'] and abs(x - o['xref']) <= c['aligned_lat_cm'] and (gz is None or abs(gz) <= c['aligned_gz_dps'])
            self.n_al = self.n_al + _RI.w() if al else 0
        else: self.n_al = 0
        o['aligned'] = self.n_al >= _RI.thr(c['aligned_n'])
        self.last = o
        return o


def virtual_line(Hinv, o, zs=(50., 90.)):
    """Image line x = a*y + b (640x360) of the lateral reference (o['xref'], along the deck) seen from the pose o (x, phi), in the same
    form as a dash-route fit, so upper_single's FOLLOW law and gains apply unchanged. zs: two ranges (cm, calibrated range)."""
    ph = math.radians(o['phi']); off = o['xref'] - o['x']; pts = []
    for z in zs:
        gx = off / math.cos(ph) + z * math.tan(ph)
        w = Hinv[2, 0] * gx + Hinv[2, 1] * z + Hinv[2, 2]
        pts.append(((Hinv[0, 0] * gx + Hinv[0, 1] * z + Hinv[0, 2]) / w, (Hinv[1, 0] * gx + Hinv[1, 1] * z + Hinv[1, 2]) / w))
    (x1, y1), (x2, y2) = pts
    if abs(y2 - y1) < 1e-6: return None
    a = (x2 - x1) / (y2 - y1); return a, x1 - a * y1
