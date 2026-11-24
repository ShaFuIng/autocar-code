"""steering gain / delay / play and camera-error model on the curve-1 passes (-3.0 .. +2.5 s around C1e).  python ident.py [runs]
writes c1err.json (camera model) next to this file."""
import sys, json, numpy as np, lib, passes
runs = sys.argv[1:] or lib.MAIN + lib.MORE
X = []
for run in runs:
    Q, A = passes.run_arrays(run)
    for i in passes.exits(A, 'C1e'):
        t = A['t']; ref = passes.straight_ref(A, i); tb = ref - A['yaw']
        m = (t >= t[i] - 3.0) & (t <= t[i] + 2.5)
        X.append(dict(run=run, lap=A['lap'][i], t=t[m] - t[i], gz=A['gz'][m], v=A['v'][m], pre=A['pre'][m], sent=A['sent'][m], tb=tb[m], e=A['e'][m], y=A['y'][m], psi=A['psi'][m], odo=A['odo'][m]))
print('passes', len(X))


def lagged(t, u, lag): return np.interp(t - lag, t, u)


def play(u, h):
    w = np.empty_like(u); w[0] = u[0]
    for k in range(1, len(u)): w[k] = min(max(w[k - 1], u[k] - h / 2), u[k] + h / 2)
    return w


print('\nsteering identification (frames with v > 0.6):  curvature[deg/m] = -gz/v = G * (u(t - lag) - zero)')
for name, key, h in (('pre-play command', 'pre', 0), ('sent, no play', 'sent', 0), ('sent, play 110', 'sent', 110), ('sent, play 130', 'sent', 130), ('sent, play 155', 'sent', 155)):
    for lag in (0.10, 0.15, 0.20, 0.25, 0.30):
        U = []; K = []
        for p in X:
            u = p[key].copy(); ok = np.isfinite(u) & np.isfinite(p['gz']) & np.isfinite(p['v']) & (p['v'] > 0.6)
            if ok.sum() < 30: continue
            u = np.interp(p['t'], p['t'][ok], u[ok]); w = play(u, h) if h else u
            U.append(lagged(p['t'], w, lag)[ok]); K.append((-p['gz'] / p['v'])[ok])
        U = np.concatenate(U); K = np.concatenate(K); a, b = np.polyfit(U, K, 1); r = np.corrcoef(U, K)[0, 1]
        print('  %-18s lag %.2f: G %.3f deg/m/us  zero at %+5.0f us  r %.3f  resid sd %.1f deg/m  n %d' % (name, lag, a, -b / a, r, np.std(K - (a * U + b)), len(U)))
tb = np.concatenate([p['tb'] for p in X]); e = np.concatenate([p['e'] for p in X]); y = np.concatenate([p['y'] for p in X]); psi = np.concatenate([p['psi'] for p in X])
ok = np.isfinite(tb) & np.isfinite(e) & np.isfinite(y) & np.isfinite(psi) & (tb > -5) & (tb < 165) & (e < 0.98)
edges = np.arange(-5, 170, 10); nb = len(edges) - 1; bi = np.clip(np.digitize(tb[ok], edges) - 1, 0, nb - 1)
M = np.zeros((ok.sum(), nb + 2)); M[np.arange(ok.sum()), bi] = 1; M[:, nb] = y[ok]; M[:, nb + 1] = psi[ok]
co, *_ = np.linalg.lstsq(M, e[ok], rcond=None); res = e[ok] - M @ co
print('\ncamera error model A (free)  e = f(tb) + b*y + c*psi   n %d  b %.4f /cm (+ = outside)  c %.4f /deg (+ = pointing out)  resid sd %.3f  R2 %.2f' % (ok.sum(), co[nb], co[nb + 1], res.std(), 1 - res.var() / e[ok].var()))
print('   (arc_lat psi is noisy frame to frame -> the free c is biased low; the 11-17 geometry model gives 0.025 /cm and 0.037 /deg = look-ahead ~85 cm)')
# model B: geometry-tied heading term  c = b * L * pi/180, L = 85 cm (rear axle -> sampled row), fitted b
LA = 85.
M2 = np.zeros((ok.sum(), nb + 1)); M2[np.arange(ok.sum()), bi] = 1; M2[:, nb] = y[ok] + LA * np.radians(psi[ok])
c2, *_ = np.linalg.lstsq(M2, e[ok], rcond=None); r2 = e[ok] - M2 @ c2
print('camera error model B (tied)  e = f(tb) + b*(y + %.0f cm * psi[rad])   b %.4f /cm -> c %.4f /deg   resid sd %.3f  R2 %.2f' % (LA, c2[nb], c2[nb] * LA * np.pi / 180, r2.std(), 1 - r2.var() / e[ok].var()))
print('  tb bin    f_A(tb)  f_B(tb) | median e   y    psi   n')
for k in range(nb):
    mk = bi == k
    if mk.sum() > 10: print('  %4d-%4d  %+.3f   %+.3f  | %+.2f  %+5.1f %+5.1f  %d' % (edges[k], edges[k + 1], co[k], c2[k], np.median(e[ok][mk]), np.median(y[ok][mk]), np.median(psi[ok][mk]), mk.sum()))
json.dump(dict(edges=edges.tolist(), f=c2[:nb].tolist(), b=float(c2[nb]), c=float(c2[nb] * LA * np.pi / 180), sd=float(r2.std()), free=dict(f=co[:nb].tolist(), b=float(co[nb]), c=float(co[nb + 1]), sd=float(res.std()))), open(lib.HERE / 'c1err.json', 'w'))
