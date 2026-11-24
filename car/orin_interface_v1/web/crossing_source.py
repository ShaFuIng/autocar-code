"""crossing-source-r1: ground-plane boundary sanity for the camera-pair-v2-junction estimator (worker side).
Pure numpy helpers; the estimator hook is in ground_identity.py (GroundIdentityEstimate._pair).
Two rules, both on the white-line tracks BEFORE pairing / single-side selection:
 1. ground kink cut: a lane edge is smooth on the ground; where a track's ground direction turns by more than
    kink_deg within one short segment (the '<' where the lane edge meets a crossing / V marking) the track is
    cut there and only the near part is kept;
 2. direction continuity: a track whose ground heading differs from the lane heading carried over from the
    previous accepted boundaries (rotated by the gyro yaw since) by more than dir_deg is not used as a boundary.
    Only while the car is not turning hard (|gz| <= gz_max_dps) and the carried heading is <= ref_max_s old.
Ground = camera-ground-20261109-r2 homography (flat floor, calibrated camera pose). Thresholds are experiment
values from the 11-10..11-13 logs, not validated limits. enabled false = the estimator is not touched.
r2 (zone gate): the two rules act only on frames whose worker request carries zone in `zones` (live_pid sends
zone "xp2" in the circle-crossing stretch after curve 2). Outside the zone the estimator output is unchanged; only the
lane-direction reference is kept warm from the accepted edges (read-only on the tracks).
"""
import math
import numpy as np

REVISION = 'crossing-source-r2'
DEFAULTS = dict(enabled=False, kink_deg=15.0, base_cm=12.0, seg_cm=12.0, dir_deg=15.0, gz_max_dps=30.0,
                ref_max_s=1.5, min_span_cm=12.0, z_min_cm=30.0, z_max_cm=140.0, dir_rule=True, kink_rule=True,
                zones=('xp2',), zone_start_m=0.4, zone_end_m=4.5, zone_min_turned_deg=100.0)


_MAXR, _MINR, _ADDR = np.maximum.reduce, np.minimum.reduce, np.add.reduce   # pathfast-r1


def _eigh_probe():
    """pathfast2-r1: np.linalg.eigh on a real 2x2 float64 matrix is one call of the gufunc _umath_linalg.eigh_lo (signature
    'd->dd', LAPACK ?syevd) plus Python argument checks and an error-state context. Calling the gufunc directly gives the same
    arrays; used only if that holds on this numpy for a set of test matrices (bitwise), else np.linalg.eigh. A non-finite
    result (where np.linalg.eigh would raise LinAlgError) is recomputed with np.linalg.eigh, so errors are unchanged."""
    try:
        gu = np.linalg._umath_linalg.eigh_lo
        rng = np.random.default_rng(20261116)
        tests = [np.array([[4., 1.], [1., 3.]]), np.eye(2), np.zeros((2, 2)), np.array([[1e-12, 0.], [0., 1e12]])]
        for _ in range(64):
            x = rng.normal(0, 30, (12, 2)); x -= x.mean(0); tests.append(x.T @ x)
        for a in tests:
            w0, v0 = np.linalg.eigh(a); w1, v1 = gu(a, signature='d->dd')
            if w0.tobytes() != w1.tobytes() or v0.tobytes() != v1.tobytes() or v1.dtype != v0.dtype: return None
        return gu
    except Exception:
        return None


_EIGH_GU = _eigh_probe()
EIGH_DIRECT = True


def _eigh(a):
    gu = _EIGH_GU if EIGH_DIRECT else None
    if gu is None or a.dtype != np.float64 or a.shape != (2, 2): return np.linalg.eigh(a)
    w, v = gu(a, signature='d->dd')
    if not (np.isfinite(w).all() and np.isfinite(v).all()): return np.linalg.eigh(a)
    return w, v


def to_ground(H, rows, xs, dy=0.0):
    p = np.stack([np.asarray(xs, float), np.asarray(rows, float) + dy, np.ones(len(rows))], 1) @ np.asarray(H).T
    return p[:, :2] / p[:, 2:3]


def track_ground(H, pts, dy, zr):
    """pts {row: x} -> (rows near->far, g Nx2 [x_cm, z_cm]) inside the z range."""
    if not pts: return np.zeros(0, int), np.zeros((0, 2))
    rows = np.array(sorted(pts, reverse=True), int)          # near (large row) first
    g = to_ground(H, rows, [pts[int(r)] for r in rows], dy)
    ok = (g[:, 1] >= zr[0]) & (g[:, 1] <= zr[1]) & np.isfinite(g).all(1)
    return rows[ok], g[ok]


def angle_deg(g):
    """heading of a point set (principal direction), degrees in (-90, 90]: 0 = along the car axis,
    + = the line runs to the right as it goes away from the car. None if shorter than 3 cm."""
    g = np.asarray(g, float)
    if len(g) < 3: return None
    # pathfast-r1: np.ptp(g, 0) = max.reduce - min.reduce and g.mean(0) = add.reduce / n, called directly (same reductions)
    if np.hypot(*(_MAXR(g, 0) - _MINR(g, 0))) < 3: return None
    c = g - _ADDR(g, 0) / len(g)
    w, v = _eigh(c.T @ c); d = v[:, -1]   # pathfast2-r1 (same arrays as np.linalg.eigh)
    if d[1] < 0 or (d[1] == 0 and d[0] < 0): d = -d
    return math.degrees(math.atan2(d[0], d[1]))


def kink_index(g, kink_deg, base_cm, seg_cm):
    """index (into g, near->far) of the first point beyond a ground kink, or None.
    Break test: the straight fit of the near part (>= base_cm, >= 4 points) against the straight fit of the
    next seg_cm (>= 3 points). A lane edge in the measured curves (centre radius >= 1.175 m, inner edge
    >= ~0.95 m) turns ~0.6 deg/cm, i.e. well under kink_deg between the two fits; the '<' where the lane
    edge meets the crossing marking turns ~20-25 deg within ~9 cm (b44b / ffc2 / ed96 / 4879 logs)."""
    n = len(g)
    i = 3
    while i < n and g[i, 1] - g[0, 1] < base_cm: i += 1
    while i < n - 2:
        j = i + 2
        while j < n - 1 and g[j, 1] - g[i, 1] < seg_cm: j += 1
        if g[j, 1] - g[i, 1] < seg_cm * 0.75: return None
        a = angle_deg(g[max(0, i - 11):i + 1]); b = angle_deg(g[i:j + 1])
        if a is not None and b is not None and adiff(a, b) > kink_deg: return i + 1
        i += 1
    return None


def track_heading(g, min_span):
    if len(g) < 4 or np.hypot(*np.ptp(g, 0)) < min_span: return None
    return angle_deg(g)


def adiff(a, b):
    """smallest difference of two line directions (lines, not vectors: 180 deg periodic)."""
    return abs((a - b + 90.0) % 180.0 - 90.0)
