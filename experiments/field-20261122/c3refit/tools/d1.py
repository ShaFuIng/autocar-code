"""diagnosis 1 + 6: per pass - where the lane features are in the controller's distance (C3s scatter), the state at the hand-back and before the ramp foot.
lane heading T = car heading (gyro since C3s) + arc_lat psi.  Features (continuous distance xc):
  xB1 = where T crosses 60 deg (middle of bend 1), xB2 = where T crosses 105 deg (middle of bend 2), xK = where the car heading crosses 150 deg (corner), xG = yaw-rate peak in 4.2-5.8 m."""
import sys, numpy as np, p3
RUNS = sys.argv[1:] or ['d60e', '84f2', '6f61', 'ccf1', 'c6f3']


def cross(x, y, lev, lo, hi):
    m = (x >= lo) & (x <= hi) & np.isfinite(y)
    xs, ys = x[m], y[m]
    for i in range(1, len(xs)):
        if ys[i - 1] < lev <= ys[i]: return xs[i - 1] + (xs[i] - xs[i - 1]) * (lev - ys[i - 1]) / (ys[i] - ys[i - 1])
    return np.nan


def at(A, k, x0, h=0.15):
    m = (A['xc'] >= x0 - h) & (A['xc'] < x0 + h); v = A[k][m].astype(float)
    return np.nanmedian(v) if np.isfinite(v).any() else np.nan


rows = []
print('run  lap | xB1(T=60) xB2(T=105) xK(h=150) xG(gz peak) | h at 0.6  1.2  2.4  4.2  5.7 | y at 1.2  2.4  3.3  4.2  5.7 | psi 4.2 | corner 4.2-5.6: err max, cmd max, cout min | tail 5.6-7.2: err max, cmd max | front cin min')
for q in p3.passes(RUNS):
    A = q['A']; x = A['xc']; T = A['h'] + A['psi']
    # smooth T a little (psi is noisy): running median of 5
    Ts = np.array([np.nanmedian(T[max(0, i - 2):i + 3]) if np.isfinite(T[max(0, i - 2):i + 3]).any() else np.nan for i in range(len(T))])
    xB1 = cross(x, Ts, 60., 0.9, 3.0); xB2 = cross(x, Ts, 105., 2.6, 4.6); xK = cross(x, A['h'], 150., 3.5, 6.2)
    m = (x >= 4.2) & (x <= 5.8); xG = x[m][np.nanargmin(A['gz'][m])]
    f = lambda lo, hi, k, fn: fn(A[k][(x >= lo) & (x < hi)].astype(float)) if np.isfinite(A[k][(x >= lo) & (x < hi)].astype(float)).any() else np.nan
    r = (q['run'], q['lap'], xB1, xB2, xK, xG, at(A, 'h', 0.6), at(A, 'h', 1.2), at(A, 'h', 2.4), at(A, 'h', 4.2), at(A, 'h', 5.7), at(A, 'y', 1.2), at(A, 'y', 2.4), at(A, 'y', 3.3), at(A, 'y', 4.2), at(A, 'y', 5.7), at(A, 'psi', 4.2),
         f(4.2, 5.6, 'err', np.nanmax), f(4.2, 5.6, 'steer', np.nanmax), f(4.2, 5.6, 'cout', np.nanmin), f(5.6, 7.2, 'err', np.nanmax), f(5.6, 7.2, 'steer', np.nanmax), f(0.6, 4.2, 'cin', np.nanmin))
    rows.append(r)
    print('%s %d | %5.2f %5.2f %5.2f %5.2f | %5.1f %5.1f %5.1f %5.1f %5.1f | %+5.1f %+5.1f %+5.1f %+5.1f %+5.1f | %+5.1f | %.2f %+4.0f %+5.1f | %.2f %+4.0f | %+5.1f' % r)
R = np.array([r[2:] for r in rows], float); runs = np.array([r[0] for r in rows])
for name, m in (('no x-band (d60e 84f2 6f61)', np.isin(runs, ['d60e', '84f2', '6f61'])), ('c6f3 (x-band owns)', runs == 'c6f3')):
    if not m.any(): continue
    print('%s: n %d | xB1 mean %.2f sd %.2f | xB2 mean %.2f sd %.2f | xK mean %.2f sd %.2f | xG mean %.2f sd %.2f | xB2-xB1 sd %.2f  xK-xB2 sd %.2f' % (name, m.sum(), np.nanmean(R[m, 0]), np.nanstd(R[m, 0]), np.nanmean(R[m, 1]), np.nanstd(R[m, 1]),
          np.nanmean(R[m, 2]), np.nanstd(R[m, 2]), np.nanmean(R[m, 3]), np.nanstd(R[m, 3]), np.nanstd(R[m, 1] - R[m, 0]), np.nanstd(R[m, 2] - R[m, 1])))
m = np.isin(runs, ['d60e', '84f2', '6f61', 'ccf1', 'c6f3'])
ok = m & np.isfinite(R).all(axis=1)
c = lambda i, j: np.corrcoef(R[ok, i], R[ok, j])[0, 1]
print('all %d passes: tail err max vs h(5.7) r %+.2f | vs y(5.7) r %+.2f | tail cmd max vs h(5.7) r %+.2f | corner err max vs h(4.2) r %+.2f | vs y(4.2) r %+.2f | corner cout min vs h(4.2) r %+.2f vs y(4.2) r %+.2f | xB1 vs xB2 r %+.2f | xB2 vs xK r %+.2f' % (
    ok.sum(), c(18, 8), c(18, 13), c(19, 8), c(15, 7), c(15, 12), c(17, 7), c(17, 12), c(0, 1), c(1, 2)))
