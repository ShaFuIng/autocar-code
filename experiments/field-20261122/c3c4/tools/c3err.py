"""curve 3: camera near error = f(x bin) + b*y_cm + c*psi_deg (least squares, 20 passes without the curve-3 trim, x 0..4.2 m, error < 0.98),
and how the final command reacts to the error inside an x bin (K = us per unit error, pass-to-pass regression) -> c3err.json for sim_c3.py."""
import json, numpy as np, lib
from c3pass import passes
P = passes(lib.BASE_RUNS); edges = np.arange(0.0, 4.21, 0.3); nb = len(edges) - 1
X = []; Y = []; M = []
for q in P:
    A = q['A']; m = np.isfinite(A['x']) & np.isfinite(A['y']) & np.isfinite(A['psi']) & np.isfinite(A['err']) & (A['x'] >= 0) & (A['x'] < 4.2) & (A['err'] < 0.98) & (A['valid'] > 0)
    for i in np.where(m)[0]:
        k = int(A['x'][i] // 0.3); r = np.zeros(nb + 2); r[k] = 1; r[nb] = A['y'][i]; r[nb + 1] = A['psi'][i]; X.append(r); Y.append(A['err'][i]); M.append((A['x'][i], A['y'][i], A['psi'][i], A['steer'][i], A['err'][i]))
X = np.array(X); Y = np.array(Y); M = np.array(M); co = np.linalg.lstsq(X, Y, rcond=None)[0]; res = Y - X @ co
print('frames %d  b %.4f per cm (+ = outside)  c %.4f per deg (+ = pointing out)  residual sd %.3f  R2 %.2f' % (len(Y), co[nb], co[nb + 1], res.std(), 1 - res.var() / Y.var()))
print('  x bin      f (centred, tangent)  driving err med   y med   psi med |  K = d(final cmd)/d(err) in the bin (us per unit)  n')
Ks = []
for k in range(nb):
    m = (M[:, 0] >= edges[k]) & (M[:, 0] < edges[k + 1])
    kk = np.polyfit(M[m, 4], M[m, 3], 1)[0] if m.sum() > 20 else np.nan; Ks.append(kk)
    print('  %.1f-%.1f   %+.2f                 %+.2f            %+5.1f   %+5.1f   | %5.0f   %d' % (edges[k], edges[k + 1], co[k], np.median(Y[m]), np.median(M[m, 1]), np.median(M[m, 2]), kk, m.sum()))
print('K median %.0f' % np.nanmedian(Ks))
json.dump(dict(b=float(co[nb]), c=float(co[nb + 1]), K=float(np.nanmedian(Ks)), Kx=[[float(edges[k] + .15), float(Ks[k])] for k in range(nb)]), open(lib.HERE / 'c3err.json', 'w'))
