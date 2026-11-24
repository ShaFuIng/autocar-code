"""curve 4: what the camera near error reads as a function of the turned angle, the lateral position and the relative heading.
model  err = f(tb bin) + b * y_cm + c * psi_deg   (least squares over all curve-4 frames with a valid arc_lat fix, tb 40..190)
-> the 'dynamic centred' error f(tb) (y = 0, psi = 0) to compare with the static hand-placed measurement (+0.06 for the 18 px preview offset)."""
import json, sys, numpy as np, lib
from c4pass import passes
P = passes()
edges = np.arange(40, 191, 10); nb = len(edges) - 1
X = []; Y = []; meta = []
for q in P:
    A = q['A']
    m = np.isfinite(A['tb']) & np.isfinite(A['y']) & np.isfinite(A['psi']) & np.isfinite(A['err']) & (A['tb'] >= 40) & (A['tb'] < 190) & (A['err'] < 0.98) & (A['valid'] > 0)
    for i in np.where(m)[0]:
        k = int((A['tb'][i] - 40) // 10); r = np.zeros(nb + 2); r[k] = 1; r[nb] = A['y'][i]; r[nb + 1] = A['psi'][i]; X.append(r); Y.append(A['err'][i]); meta.append((A['tb'][i], A['y'][i], A['psi'][i]))
X = np.array(X); Y = np.array(Y); co, res, rk, sv = np.linalg.lstsq(X, Y, rcond=None)
pred = X @ co; print('frames', len(Y), 'b (per cm of y, + = outside) %.4f' % co[nb], 'c (per deg of psi, + = pointing out) %.4f' % co[nb + 1], 'residual sd %.3f' % np.std(Y - pred), 'R2 %.2f' % (1 - np.var(Y - pred) / np.var(Y)))
M = np.array(meta)
STATIC = [(16, .39), (30, .56), (44, .44), (67, .53), (83, .65), (106, .13), (122, .36), (142, .22), (168, .18), (185, .29), (197, .03)]
print(' tb bin   f(tb)=centred,tangent | driving median err  y   psi | static(+0.06) nearest')
out = []
for k in range(nb):
    m = (M[:, 0] >= edges[k]) & (M[:, 0] < edges[k + 1])
    st = min(STATIC, key=lambda s: abs(s[0] - (edges[k] + 5)))
    print('  %3d-%3d   %+.3f              | %+.2f  %+5.1f %+5.1f | %+.2f @%d' % (edges[k], edges[k + 1], co[k], np.median(Y[m]), np.median(M[m, 1]), np.median(M[m, 2]), st[1] + .06, st[0]))
    out.append([float(edges[k] + 5), round(float(co[k]), 3)])
# per-bin sensitivity check (separate fits)
print(' per-bin separate fit  b  c')
for k in range(nb):
    m = (M[:, 0] >= edges[k]) & (M[:, 0] < edges[k + 1])
    A_ = np.c_[np.ones(m.sum()), M[m, 1], M[m, 2]]; c2 = np.linalg.lstsq(A_, Y[m], rcond=None)[0]
    print('  %3d-%3d  a %+.3f b %+.4f c %+.4f n %d' % (edges[k], edges[k + 1], c2[0], c2[1], c2[2], m.sum()))
json.dump(dict(b=float(co[nb]), c=float(co[nb + 1]), f=out), open(lib.HERE / 'c4err.json', 'w'))
