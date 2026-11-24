"""steering gain check for the models: curvature (-gz / v, deg/m) at t + lag against the command (before the backlash stage, us from the centre),
least squares per zone. c3 front = C3s+0.5..4.2 m (20 passes), c4 = lap_ff4 x 1.5..7.2 m (26 passes)."""
import numpy as np, lib
from c3pass import passes as p3
from c4pass import passes as p4


def fit(P, xk, lo, hi, label):
    for lag in (0.15, 0.20, 0.25, 0.30, 0.35):
        U = []; K = []; per = []
        for q in P:
            A = q['A']; t = A['t']; k = -A['gz'] / np.where(A['v'] > 0.3, A['v'], np.nan); kk = np.interp(t + lag, t, np.nan_to_num(k, nan=0.))
            m = (A[xk] >= lo) & (A[xk] <= hi) & np.isfinite(A['steer']) & np.isfinite(kk) & np.isfinite(k)
            U.extend(A['steer'][m]); K.extend(kk[m])
            if m.sum() > 30: per.append(np.polyfit(A['steer'][m], kk[m], 1))
        U = np.array(U); K = np.array(K); a, b = np.polyfit(U, K, 1); r = np.corrcoef(U, K)[0, 1]; per = np.array(per)
        print('%s lag %.2f s: curvature = %.3f * (cmd %+.0f)  r %.2f  n %d | per pass gain median %.3f (p10 %.3f p90 %.3f), mean cmd %+.0f mean curvature %.1f' % (label, lag, a, b / a, r, len(U), np.median(per[:, 0]), np.percentile(per[:, 0], 10), np.percentile(per[:, 0], 90), U.mean(), K.mean()))


fit(p3(lib.BASE_RUNS), 'x', 0.5, 4.2, 'c3 front'); fit(p4(), 'x4', 1.5, 7.2, 'c4      ')
