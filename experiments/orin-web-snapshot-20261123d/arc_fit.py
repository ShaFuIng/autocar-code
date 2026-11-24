"""arc-fit core (offline analysis copy; the worker candidate module candidate/arc_fit.py is generated from this same code).

Per frame, from the v4 line mask (640x360, 0/255):
  1. connected components of the mask below row Y_TOP (z <= Z_MAX on flat ground),
  2. per component, on every ROW_STEP-th row: the run of that component -> both run ends projected to ground with the flat-ground
     homography (camera-ground-20261109-r2: x cm right, z cm forward of the front-tyre foremost edge) -> chord midpoint = line centre,
  3. PCA direction + parabola w = a + b u + c u^2 in the rotated frame (1 outlier-rejection pass) -> centreline curvature,
     direction and line width (median chord component perpendicular to the tangent),
  4. side of the line = sign of the car reference point P relative to the oriented line (oriented away from the car),
  5. extrapolation back to P with a circle (curvature of the fit) and with a straight line.
Signs: curvature and angles LEFT-POSITIVE (counter-clockwise, same sign as IMU gz); right curves are negative.
       d_*: distance from P to the line INNER edge (cm, >= 0 means P is inside, on the lane side of the line).
Nothing here uses the lane-width ratio or the worker's centre estimate.
"""
import math
import time
import numpy as np
import cv2

REVISION = 'arc-fit-r1'
H_DEFAULT = [[-0.15814323671781763, -0.0054270402019372475, 53.299599488926496],
             [-0.010394124797423627, -0.01821740515439466, -71.54463499234244],
             [-0.0003437155985005508, -0.01013832954490162, 1.0]]
P_REF = (0.0, -4.0)        # front-axle centre: x 0 (camera calibration lane centre), 4 cm behind the front-tyre foremost edge
Z_MIN, Z_MAX = 30.0, 160.0
ROW_STEP = 3
MIN_PTS = 6
MIN_SPAN = 25.0            # cm of visible line
HALF_LANE = 23.0           # half of the 46 cm inner lane width
DEG = 180.0 / math.pi


class ArcFit:
    def __init__(self, H=None, p_ref=P_REF, z_min=Z_MIN, z_max=Z_MAX, row_step=ROW_STEP, shape=(360, 640), dist=None):
        self.H = np.asarray(H if H is not None else H_DEFAULT, float)
        self.dist = dist                                  # None or (k1, k2, cx, cy, norm[, dy]): radial undistortion applied before H
        self.P = np.asarray(p_ref, float)
        self.h, self.w = shape
        # rows whose image-centre ground z lies in [z_min, z_max]
        ys = np.arange(self.h, dtype=float)
        _, zc = self._proj(np.full_like(ys, self.w / 2.0), ys)
        ok = (zc >= z_min) & (zc <= z_max) & np.isfinite(zc)
        self.y_top = int(ys[ok].min())
        self.rows = np.arange(self.h - 1, self.y_top - 1, -row_step)
        self.z_min, self.z_max = z_min, z_max

    def _undist(self, u, y):
        if not self.dist:
            return u, y
        k1, k2, cx, cy, nr = self.dist[:5]
        dy = self.dist[5] if len(self.dist) > 5 else 0.0      # camera pitch change since calibration ~ row shift (undistorted px)
        xn = (u - cx) / nr; yn = (y - cy) / nr; r2 = xn * xn + yn * yn
        f = 1.0 + k1 * r2 + k2 * r2 * r2
        return cx + (u - cx) * f, cy + (y - cy) * f - dy

    def _proj(self, u, y):
        u, y = self._undist(np.asarray(u, float), np.asarray(y, float))
        H = self.H
        d = H[2, 0] * u + H[2, 1] * y + H[2, 2]
        return (H[0, 0] * u + H[0, 1] * y + H[0, 2]) / d, (H[1, 0] * u + H[1, 1] * y + H[1, 2]) / d

    # ------------------------------------------------------------------ per component
    def _fit(self, pts, chords):
        n = len(pts)
        c0 = pts.mean(0)
        q = pts - c0
        cov = q.T @ q / n
        ev, evec = np.linalg.eigh(cov)
        d = evec[:, 1]
        nrm = np.array([-d[1], d[0]])
        keep = np.ones(n, bool)
        coef = None
        for _ in range(2):
            u = q[keep] @ d
            w = q[keep] @ nrm
            if keep.sum() < MIN_PTS:
                return None
            A = np.column_stack([np.ones_like(u), u, u * u])
            coef, *_ = np.linalg.lstsq(A, w, rcond=None)
            ua = q @ d
            res = q @ nrm - (coef[0] + coef[1] * ua + coef[2] * ua * ua)
            sd = max(1.0, 2.5 * float(np.sqrt(np.mean(res[keep] ** 2))))
            nk = np.abs(res) <= sd
            if nk.sum() == keep.sum() and np.all(nk == keep):
                break
            keep = nk
        ua = q @ d
        res = q @ nrm - (coef[0] + coef[1] * ua + coef[2] * ua * ua)
        rms = float(np.sqrt(np.mean(res[keep] ** 2)))
        uk = ua[keep]
        # orient d from the end nearer to P toward the far end
        e_lo = c0 + uk.min() * d + (coef[0] + coef[1] * uk.min() + coef[2] * uk.min() ** 2) * nrm
        e_hi = c0 + uk.max() * d + (coef[0] + coef[1] * uk.max() + coef[2] * uk.max() ** 2) * nrm
        flip = np.hypot(*(e_lo - self.P)) > np.hypot(*(e_hi - self.P))
        u_near = uk.max() if flip else uk.min()
        u_mid = 0.5 * (uk.min() + uk.max())
        span = float(uk.max() - uk.min())
        s_or = -1.0 if flip else 1.0   # sign to orient tangents away from the car

        def tangent(uu):
            t = d + (coef[1] + 2 * coef[2] * uu) * nrm
            return s_or * t / np.hypot(*t)

        crossdn = d[0] * nrm[1] - d[1] * nrm[0]          # +1 or -1
        # (x right, z forward) seen from above is a standard right-handed plane: CCW == left turn.
        # k = cross(r', r'') / |r'|^3 with r' = d + w' n, r'' = 2c n -> 2c*crossdn/(1+w'^2)^1.5 for travel along +u;
        # travelling the other way (s_or = -1) flips the sign.
        def curv(uu):
            wp = coef[1] + 2 * coef[2] * uu
            return 2 * coef[2] * crossdn / (1 + wp * wp) ** 1.5 * s_or   # left-positive (1/cm)
        Q0 = c0 + u_near * d + (coef[0] + coef[1] * u_near + coef[2] * u_near ** 2) * nrm
        t0 = tangent(u_near)
        k_mid = curv(u_mid)
        # circle (Pratt algebraic fit, handles straight lines) on the kept points: replaces the parabola curvature/near point/tangent
        cf = self._pratt(q[keep] / 100.0)
        if cf is not None:
            A_, B_, C_, D_ = cf
            pk = q[keep]
            if abs(A_) > 1e-4:                             # radius < 50 m
                cen = np.array([-B_ / (2 * A_), -C_ / (2 * A_)]) * 100.0
                R = 0.5 / abs(A_) * 100.0
                dd = np.hypot(pk[:, 0] - cen[0], pk[:, 1] - cen[1])
                rms = float(np.sqrt(np.mean((dd - R) ** 2)))
                pn = pk[np.argmin(np.hypot(*(pk + c0 - self.P).T))]
                rv = pn - cen; rv = rv / np.hypot(*rv)
                Q0 = c0 + cen + rv * R
                tt = np.array([-rv[1], rv[0]])
                if tt @ (d * s_or) < 0:
                    tt = -tt
                t0 = tt
                k_mid = (1.0 / R) * (1.0 if (t0[0] * (cen[1] + c0[1] - Q0[1]) - t0[1] * (cen[0] + c0[0] - Q0[0])) > 0 else -1.0)
            else:                                          # straight: B u + C w + D = 0 (scaled)
                nv = np.array([B_, C_]); nn = np.hypot(*nv)
                if nn > 0:
                    nv = nv / nn
                    tt = np.array([-nv[1], nv[0]])
                    if tt @ (d * s_or) < 0:
                        tt = -tt
                    dist = (pk @ nv) + D_ * 100.0 / nn
                    rms = float(np.sqrt(np.mean(dist ** 2)))
                    pn = pk[np.argmin(np.hypot(*(pk + c0 - self.P).T))]
                    Q0 = c0 + pn - nv * (pn @ nv + D_ * 100.0 / nn)
                    t0 = tt; k_mid = 0.0
        # line width from chords: component perpendicular to the local tangent
        ck = chords[keep]
        wp = coef[1] + 2 * coef[2] * uk
        tl = d[None, :] + wp[:, None] * nrm[None, :]
        tl = tl / np.hypot(tl[:, 0], tl[:, 1])[:, None]          # orientation sign irrelevant for |cross|
        wid = np.abs(tl[:, 0] * ck[:, 1] - tl[:, 1] * ck[:, 0])
        lw = float(np.median(wid))
        return dict(n=int(keep.sum()), rms=rms, span=span, k=float(k_mid), Q0=Q0, t0=t0, lw=lw, pk=pts[keep], uk=uk,   # arc-lat-r1: kept centre points (+ position along the fit axis) for the optional 'pts' output
                    zmin=float(pts[keep, 1].min()), zmax=float(pts[keep, 1].max()),
                    xnear=float(Q0[0]), znear=float(Q0[1]))

    @staticmethod
    def _pratt(p):
        """Pratt circle fit; returns (A, B, C, D) of A(x^2+y^2) + Bx + Cy + D = 0 with B^2 + C^2 - 4AD = 1, or None."""
        x = p[:, 0]; y = p[:, 1]; z = x * x + y * y
        Z = np.column_stack([z, x, y, np.ones_like(x)])
        M = Z.T @ Z / len(x)
        Binv = np.array([[0, 0, 0, -0.5], [0, 1, 0, 0], [0, 0, 1, 0], [-0.5, 0, 0, 0]])
        try:
            ev, evec = np.linalg.eig(Binv @ M)
        except np.linalg.LinAlgError:
            return None
        ev = ev.real; evec = evec.real
        best = None
        for i in np.argsort(ev):
            a = evec[:, i]
            c = a[1] ** 2 + a[2] ** 2 - 4 * a[0] * a[3]
            if c > 1e-12 and ev[i] > -1e-9:
                best = a / math.sqrt(c); break
        return None if best is None else tuple(float(v) for v in best)

    def _signed(self, Q0, t0, k):
        """signed lateral offset of P from the oriented centreline (+ = P left of the line), circle and straight extrapolation."""
        v = self.P - Q0
        s_lin = float(t0[0] * v[1] - t0[1] * v[0])        # cross(t, v) > 0: P is left of the oriented line
        psi_lin = math.degrees(math.atan2(-t0[0], t0[1]))
        if abs(k) < 1e-5:                                 # R > 1000 m... practically straight
            return s_lin, s_lin, psi_lin
        R = 1.0 / abs(k)
        nl = np.array([-t0[1], t0[0]])                    # left normal (CCW rotation)
        C = Q0 + nl * (1.0 / k)                           # k>0 (left curve): centre to the left
        r = self.P - C
        dist = float(np.hypot(*r))
        s_arc = math.copysign(1.0, k) * (R - dist)
        t = np.array([-r[1], r[0]]) if k > 0 else np.array([r[1], -r[0]])
        psi_arc = math.degrees(math.atan2(-t[0], t[1]))
        return float(s_arc), s_lin, psi_arc

    # ------------------------------------------------------------------ per frame
    def runs(self, mask):
        """per sampled row, the runs of every connected component: array (label, row_index, first_col, last_col) and the row y."""
        if mask.ndim == 3:
            mask = mask[..., -1]
        if mask.shape != (self.h, self.w):
            mask = cv2.resize(mask, (self.w, self.h), interpolation=cv2.INTER_NEAREST)
        sub = (mask[self.y_top:] > 0).astype(np.uint8)
        nlab, lab = cv2.connectedComponents(sub, connectivity=8)
        if nlab <= 1:
            return None, None
        R = self.rows - self.y_top
        L = lab[R]                                        # (nrows, w)
        pad = np.zeros((L.shape[0], 1), L.dtype)
        Lp = np.concatenate([pad, L, pad], 1)
        ri, ci = np.nonzero(Lp[:, 1:] != Lp[:, :-1])      # change positions, sorted by row then column
        if len(ri) == 0:
            return None, None
        # consecutive change points in the same row delimit runs; keep the non-zero ones
        same = ri[1:] == ri[:-1]
        a = ci[:-1][same]; b = ci[1:][same]; r = ri[:-1][same]
        lb = Lp[r, a + 1]
        # a run cut by the left/right image border has a wrong midpoint (the line continues outside the image): drop it
        k = (lb > 0) & (a > 0) & (b < self.w)
        if not k.any():
            return None, None
        S = np.column_stack([lb[k], r[k], a[k], b[k] - 1]).astype(float)   # run = mask columns [a, b-1]
        return S, self.rows[S[:, 1].astype(int)].astype(float)

    def measure(self, mask):
        if mask is None:
            return dict(ok=False, why='NO_MASK')
        S, y = self.runs(mask)
        if S is None:
            return dict(ok=False, why='EMPTY', lines=[])
        xa, za = self._proj(S[:, 2] - 0.5, y)             # pixel edges (pixel centres at integer columns)
        xb, zb = self._proj(S[:, 3] + 0.5, y)
        A = np.column_stack([xa, za]); B = np.column_stack([xb, zb])
        M = 0.5 * (A + B); C = B - A
        lines = []
        for lb in np.unique(S[:, 0]):
            k = (S[:, 0] == lb) & (M[:, 1] >= self.z_min - 5) & (M[:, 1] <= self.z_max + 10)
            if k.sum() < MIN_PTS:
                continue
            f = self._fit(M[k], C[k])
            if f is None or f['span'] < MIN_SPAN:
                continue
            s_arc, s_lin, psi_car = self._signed(f['Q0'], f['t0'], f['k'])
            side = 'L' if s_lin < 0 else 'R'               # P right of the line -> it is a left line
            hw = 0.5 * min(max(f['lw'], 1.0), 8.0)
            f.update(label=int(lb), side=side, s_arc=s_arc, s_lin=s_lin,
                     d_arc=abs(s_arc) - hw, d_lin=abs(s_lin) - hw, psi_car=float(psi_car),
                     psi=float(math.degrees(math.atan2(-f['t0'][0], f['t0'][1]))))
            lines.append(f)
        out = dict(ok=False, why='NO_LINE', lines=lines)
        best = {}
        for f in lines:
            # lane boundary on each side = the qualifying line nearest to P whose near end is close enough
            if f['znear'] > 110 or f['rms'] > 2.5:
                continue
            o = best.get(f['side'])
            if o is None or abs(f['s_lin']) < abs(o['s_lin']):
                best[f['side']] = f
        if not best:
            return out
        kc = []; wts = []
        for sd, f in best.items():
            # lane-centre curvature: centre is (HALF_LANE + lw/2) to the right of a left line / left of a right line
            off = (HALF_LANE + 0.5 * f['lw']) * (1.0 if sd == 'L' else -1.0)   # + = to the right
            kcen = f['k'] / (1.0 + f['k'] * off)
            f['k_centre'] = float(kcen)
            kc.append(kcen); wts.append(f['span'] ** 2 / max(f['rms'], 0.3))
        kappa = float(np.average(kc, weights=wts))
        src = ''.join(sorted(best))
        out.update(ok=True, why='OK', src=src, kappa=kappa * 100 * DEG, best=best)
        y_l = (HALF_LANE - best['L']['d_arc']) if 'L' in best else None      # car lateral position, + = left of lane centre
        y_r = (best['R']['d_arc'] - HALF_LANE) if 'R' in best else None
        out.update(y_l=y_l, y_r=y_r)
        return out


def sub_pts(f, n):
    """arc-lat-r1: at most n kept centre points of a line, evenly spaced along the fit axis, near end first, [x, z] cm rounded to 0.1."""
    pk = f.get('pk'); uk = f.get('uk')
    if pk is None or len(pk) == 0 or n <= 0: return []
    o = np.argsort(uk); pk = pk[o]
    if np.hypot(*(pk[0] - P_REF)) > np.hypot(*(pk[-1] - P_REF)): pk = pk[::-1]
    if len(pk) > n: pk = pk[np.unique(np.round(np.linspace(0, len(pk) - 1, n)).astype(int))]
    return [[round(float(a), 1), round(float(b), 1)] for a, b in pk]


def compact(res, pts_n=0):
    """flat, JSON-friendly summary (what the worker writes into path_diagnostic.arc_fit).
    arc-lat-r1: pts_n > 0 adds L / R 'pts' (line centre points on the ground, camera ground frame) and 'rev2'; pts_n 0 = arc-fit-r1 output unchanged."""
    if not res.get('ok'):
        return dict(revision=REVISION, ok=False, why=res.get('why'), n_lines=len(res.get('lines') or []))
    o = dict(revision=REVISION, ok=True, src=res['src'], kappa_dpm=round(res['kappa'], 2), n_lines=len(res['lines']),
             y_l_cm=None if res.get('y_l') is None else round(res['y_l'], 1), y_r_cm=None if res.get('y_r') is None else round(res['y_r'], 1))
    for sd in ('L', 'R'):
        f = res['best'].get(sd)
        if f is None:
            continue
        o[sd] = dict(k_dpm=round(f['k'] * 100 * DEG, 2), kc_dpm=round(f['k_centre'] * 100 * DEG, 2), d_arc_cm=round(f['d_arc'], 1),
                     d_lin_cm=round(f['d_lin'], 1), psi_car_deg=round(f['psi_car'], 1), psi_near_deg=round(f['psi'], 1), rms_cm=round(f['rms'], 2),
                     span_cm=round(f['span'], 1), z_cm=[round(f['zmin'], 1), round(f['zmax'], 1)], lw_cm=round(f['lw'], 1), n=f['n'])
        if pts_n > 0: o[sd]['pts'] = sub_pts(f, pts_n)
    if pts_n > 0: o['rev2'] = 'arc-lat-r1'
    return o


class Runner:
    """worker wrapper: config dict from crossings_worker.json key 'arc_fit' (absent = the worker never builds this).
    keys: enabled (default true), row_step (default 3), z_max (cm, default 160), every (compute every n-th frame, default 1), H (3x3, optional)."""
    def __init__(self, cfg):
        cfg = dict(cfg) if isinstance(cfg, dict) else {}
        self.on = cfg.get('enabled', True) is not False
        self.every = max(1, int(cfg.get('every') or 1)); self.n = 0
        self.af = ArcFit(H=cfg.get('H'), row_step=int(cfg.get('row_step') or ROW_STEP), z_max=float(cfg.get('z_max') or Z_MAX))
        self.pts_n = int(cfg.get('pts') or 0) if cfg.get('pts') is not True else 16   # arc-lat-r1: key 'pts' (true = 16, n = n points per line); absent / 0 = r1 output

    def run(self, mask):
        self.n += 1
        if (self.n - 1) % self.every:
            return dict(revision=REVISION, ok=False, why='SKIPPED')
        t0 = time.perf_counter()
        try:
            out = compact(self.af.measure(mask), self.pts_n)
        except Exception as e:                     # diagnostics only: never let it break the path stage
            out = dict(revision=REVISION, ok=False, why='ERROR:' + type(e).__name__)
        out['ms'] = round((time.perf_counter() - t0) * 1e3, 2)
        return out


def from_config(cfg):
    if cfg is None or cfg is False:
        return None
    r = Runner(cfg if isinstance(cfg, dict) else {})
    return r if r.on else None
