"""model refinement (adds to model.json):
 (c2) static steering map: median curvature per bin of the lagged command (is the gain constant over the command range? where does it saturate?)
 (d2) camera with zone-wise slopes: near error = F(x) + B_z*y + C_z*psi, zones along x; residual of c6f3"""
import json, numpy as np, p3
ALL = ['d60e', '84f2', '6f61', 'ccf1', 'c6f3']
P = {r: p3.passes([r]) for r in ALL}
M = json.load(open(p3.lib.HERE / 'model.json'))
XG = np.array([a for a, b in M['T']]); Tm = np.array([b for a, b in M['T']]); Tf = lambda x: np.interp(x, XG, Tm)
tau, tl = M['plant']['tau'], M['plant']['tl']


def lagged(t, u):
    us = np.interp(t - tau, t, u); o = np.empty(len(t)); o[0] = us[0]
    for i in range(1, len(t)): o[i] = o[i - 1] + (us[i] - o[i - 1]) * (1 - np.exp(-(t[i] - t[i - 1]) / tl))
    return o


U = []; K = []; X = []; R = []
for r in ALL:
    for q in P[r]:
        A = q['A']; ul = lagged(A['t'], np.nan_to_num(A['steer'])); m = (A['xc'] >= 0.3) & (A['xc'] <= 6.0) & np.isfinite(A['curv']) & np.isfinite(A['steer'])
        U.extend(ul[m]); K.extend(A['curv'][m]); X.extend(A['xc'][m]); R.extend([r] * int(m.sum()))
U = np.array(U); K = np.array(K); X = np.array(X); R = np.array(R)
print('(c2) curvature against the lagged command (dead time %.2f s, lag %.2f s), C3s + 0.3 .. 6.0 m, all runs' % (tau, tl))
ed = [-150, -50, 0, 40, 80, 120, 160, 200, 240, 280, 320, 360, 420]; mp = []
for a, b in zip(ed[:-1], ed[1:]):
    m = (U >= a) & (U < b)
    if m.sum() < 15: continue
    mc = (R == 'c6f3') & m
    mp.append([round(float(np.median(U[m])), 1), round(float(np.median(K[m])), 2)])
    print('   cmd %+4d..%+4d  n %4d  cmd median %+5.0f  curvature median %5.1f (p25 %5.1f p75 %5.1f) = %.3f per us | c6f3 only: %s' % (a, b, m.sum(), np.median(U[m]), np.median(K[m]), np.percentile(K[m], 25), np.percentile(K[m], 75),
          np.median(K[m]) / max(1., np.median(U[m])), ('%5.1f (n %d)' % (np.median(K[mc]), mc.sum())) if mc.sum() > 8 else '-'))
M['plant']['map'] = mp
# ---------- camera, zone slopes
edges = np.arange(0.0, 6.61, 0.3); nb = len(edges) - 1; Z = [0.0, 1.2, 2.4, 3.3, 4.5, 5.4, 6.6]; nz = len(Z) - 1


def rows(runs):
    Xm = []; Y = []; Mx = []
    for r in runs:
        for q in P[r]:
            A = q['A']; psi = Tf(A['xc']) - A['h']
            m = np.isfinite(A['y']) & np.isfinite(A['err']) & (A['xc'] >= 0) & (A['xc'] < 6.6) & (A['err'] < 0.98) & (A['valid'] > 0)
            for i in np.where(m)[0]:
                k = int(A['xc'][i] // 0.3); z = max(0, int(np.searchsorted(Z, A['xc'][i], side='right')) - 1); z = min(z, nz - 1)
                row = np.zeros(nb + 2 * nz); row[k] = 1; row[nb + z] = A['y'][i]; row[nb + nz + z] = psi[i]; Xm.append(row); Y.append(A['err'][i]); Mx.append((A['xc'][i], A['y'][i], psi[i]))
    return np.array(Xm), np.array(Y), np.array(Mx)


Xa, Ya, Ma = rows(ALL); co = np.linalg.lstsq(Xa, Ya, rcond=None)[0]; res = Ya - Xa @ co
print('(d2) camera near error = F(x) + B_z*y + C_z*psi, all five runs: residual sd %.3f (one B / C for the whole curve: %.3f)' % (res.std(), M['cam']['sd']))
for z in range(nz): print('   zone %.1f-%.1f m: B %.4f per cm   C %.4f per deg' % (Z[z], Z[z + 1], co[nb + z], co[nb + nz + z]))
Xc, Yc, Mc = rows(['c6f3']); rc = Yc - Xc @ co
print('   c6f3 residual by 0.3 m bin (median): ' + ' '.join('%.1f:%+.2f' % (edges[k], np.median(rc[(Mc[:, 0] >= edges[k]) & (Mc[:, 0] < edges[k + 1])])) for k in range(nb) if ((Mc[:, 0] >= edges[k]) & (Mc[:, 0] < edges[k + 1])).sum() > 3))
M['cam2'] = dict(Z=Z, B=[float(co[nb + z]) for z in range(nz)], C=[float(co[nb + nz + z]) for z in range(nz)], F=[[round(float(edges[k] + 0.15), 2), round(float(co[k]), 3)] for k in range(nb)], sd=float(res.std()))
M['plant'].update(G=0.215, U0=0., tau=0.10, tl=0.08)   # chosen with sweep.py: closed-loop calibration on 6f61 (chain) and c6f3 (band); the regression above gives 0.19-0.21 with 0.10 / 0.15
json.dump(M, open(p3.lib.HERE / 'model.json', 'w'), indent=0)
