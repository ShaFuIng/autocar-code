"""model identification for curve 3 (five runs), written to model.json:
 (a) lane heading T(x) (deg since the encoder step that carries the C3s crossing) = median over the 15 passes without x-band / trim of [gyro heading + arc_lat psi], 0.1 m grid, lightly smoothed
 (b) kinematic check: y(x) re-integrated from T(x) - h_pass(x) against arc_lat y, per run (does the geometry hold on a different path = c6f3?)
 (c) steering plant: curvature(t) = G * (u(t - tau) lagged by tl + U0), u = command before / after the backlash stage, least squares over all runs and per run
 (d) camera: near error = F(x) + B*y + C*psi, fitted on all runs; residual of c6f3 against a fit made without it
 (e) chain stand-in after the hand-back: final command as a function of the near error (per 0.3 m bin: a + K*e), all runs"""
import json, sys, numpy as np, p3
BASE = ['d60e', '84f2', '6f61']; ALL = ['d60e', '84f2', '6f61', 'ccf1', 'c6f3']
P = {r: p3.passes([r]) for r in ALL}
out = {}
# ---------- (a)
XG = np.arange(-0.6, 6.61, 0.1); Ts = []; Hs = []; Ys = []
for r in BASE:
    for q in P[r]:
        A = q['A']; T = A['h'] + A['psi']; row = []; rh = []; ry = []
        for x0 in XG:
            m = (A['xc'] >= x0 - 0.1) & (A['xc'] < x0 + 0.1)
            row.append(np.nanmedian(T[m]) if np.isfinite(T[m]).any() else np.nan); rh.append(np.nanmedian(A['h'][m]) if m.any() else np.nan); ry.append(np.nanmedian(A['y'][m]) if np.isfinite(A['y'][m]).any() else np.nan)
        Ts.append(row); Hs.append(rh); Ys.append(ry)
Tm = np.nanmedian(np.array(Ts), axis=0); Tsd = np.nanstd(np.array(Ts), axis=0)
ok = np.isfinite(Tm); Tm = np.interp(XG, XG[ok], Tm[ok])
Tm = np.convolve(np.pad(Tm, 1, mode='edge'), [0.25, 0.5, 0.25], mode='valid')
out['T'] = [[round(float(a), 2), round(float(b), 2)] for a, b in zip(XG, Tm)]
print('(a) lane heading T(x) (deg), its slope kT (deg/m over +-0.2 m), spread sd over 15 passes, base car heading / y medians')
Hm = np.nanmedian(np.array(Hs), axis=0); Ym = np.nanmedian(np.array(Ys), axis=0)
for i in range(2, len(XG) - 2, 3): print('  x %+4.1f  T %6.1f  kT %5.1f  sd %4.1f | car h %6.1f  y %+5.1f' % (XG[i], Tm[i], (Tm[i + 2] - Tm[i - 2]) / 0.4, Tsd[i], Hm[i], Ym[i]))
Tf = lambda x: np.interp(x, XG, Tm)
# ---------- (b)
print('(b) kinematic check: y re-integrated from x = 0.6 m with psi = T(x) - h(x), minus arc_lat y (cm) at 1.5 / 2.4 / 3.3 / 4.2 / 5.1 m; and psi_model - psi_arc_lat (deg) median')
for r in ALL:
    D = []; Dp = []
    for q in P[r]:
        A = q['A']; x = A['xc']; i0 = int(np.argmin(np.abs(x - 0.6))); y0 = np.nanmedian(A['y'][max(0, i0 - 3):i0 + 4])
        psi = Tf(x) - A['h']; yy = np.full(len(x), np.nan); yy[i0] = y0
        for i in range(i0 + 1, len(x)): yy[i] = yy[i - 1] + 100. * np.sin(np.radians((psi[i] + psi[i - 1]) / 2)) * (x[i] - x[i - 1])
        D.append([np.nanmedian((yy - A['y'])[(x >= a - 0.15) & (x < a + 0.15)]) for a in (1.5, 2.4, 3.3, 4.2, 5.1)]); Dp.append(np.nanmedian((psi - A['psi'])[(x > 0.6) & (x < 4.4)]))
    D = np.array(D); print('  %s  n %d  dy median %s | sd %s | dpsi %+.1f' % (r, len(D), ' '.join('%+5.1f' % v for v in np.nanmedian(D, axis=0)), ' '.join('%4.1f' % v for v in np.nanstd(D, axis=0)), np.nanmedian(Dp)))
# ---------- (c)
print('(c) steering plant, frames with C3s + 0.3 .. 4.4 m: curvature = G * (u_lagged + U0)')


def lagged(t, u, tau, tl):
    us = np.interp(t - tau, t, u); o = np.empty(len(t)); o[0] = us[0]
    for i in range(1, len(t)): o[i] = o[i - 1] + (us[i] - o[i - 1]) * (1 - np.exp(-(t[i] - t[i - 1]) / max(1e-3, tl)))
    return o


def play(u, h):
    o = np.empty(len(u)); o[0] = u[0]
    for i in range(1, len(u)): o[i] = min(max(o[i - 1], u[i] - h / 2), u[i] + h / 2)
    return o


def fitset(runs, key, tau, tl, h=0., lo=0.3, hi=4.4):
    U = []; K = []
    for r in runs:
        for q in P[r]:
            A = q['A']; u = np.nan_to_num(A[key], nan=0.)
            if h: u = play(u, h)
            ul = lagged(A['t'], u, tau, tl); m = (A['xc'] >= lo) & (A['xc'] <= hi) & np.isfinite(A['curv']) & np.isfinite(A[key])
            U.extend(ul[m]); K.extend(A['curv'][m])
    U = np.array(U); K = np.array(K); a, b = np.polyfit(U, K, 1); res = K - (a * U + b)
    return a, b / a, res.std(), 1 - res.var() / K.var(), len(U)


best = None
for key, h in (('steer', 0.), ('sent', 0.), ('sent', 110.), ('sent', 130.), ('sent', 155.)):
    for tau in (0.05, 0.10, 0.15, 0.20):
        for tl in (0.05, 0.10, 0.15, 0.20):
            g_, u0, sd, r2, n = fitset(ALL, key, tau, tl, h)
            if best is None or sd < best[0]: best = (sd, key, h, tau, tl, g_, u0, r2)
            if (tau, tl) in ((0.10, 0.10), (0.15, 0.10), (0.10, 0.15), (0.15, 0.15)): print('  all runs  u=%-5s play %3.0f  dead time %.2f s lag %.2f s: G %.3f U0 %+4.0f  residual sd %.1f deg/m  R2 %.2f  n %d' % (key, h, tau, tl, g_, u0, sd, r2, n))
sd, key, h, tau, tl, g_, u0, r2 = best
print('  best: u=%s play %.0f dead time %.2f lag %.2f: G %.3f U0 %+.0f sd %.1f R2 %.2f' % (key, h, tau, tl, g_, u0, sd, r2))
BT, BL = (tau, tl) if key == 'steer' else (0.10, 0.15)
for r in ALL:
    a = fitset([r], 'steer', BT, BL); print('  %s alone (u=steer, %.2f / %.2f): G %.3f U0 %+4.0f sd %.1f R2 %.2f' % ((r, BT, BL) + a[:4]))
for lo, hi in ((0.3, 1.2), (1.2, 2.4), (2.4, 3.3), (3.3, 4.4), (4.4, 5.6)):
    a = fitset(ALL, 'steer', BT, BL, lo=lo, hi=hi); print('  x %.1f-%.1f (all runs, u=steer): G %.3f U0 %+4.0f sd %.1f R2 %.2f n %d' % ((lo, hi) + a))
gt = np.concatenate([q['A']['gtab'][np.isfinite(q['A']['gtab'])] for r in ALL for q in P[r]]); vv = np.concatenate([q['A']['v'][np.isfinite(q['A']['gtab'])] for r in ALL for q in P[r]])
for vlo, vhi in ((0.7, 0.95), (0.95, 1.08), (1.08, 1.3)):
    U = []; K = []
    for r in ALL:
        for q in P[r]:
            A = q['A']; ul = lagged(A['t'], np.nan_to_num(A['steer']), BT, BL); m = (A['xc'] >= 0.3) & (A['xc'] <= 4.4) & np.isfinite(A['curv']) & (A['v'] >= vlo) & (A['v'] < vhi)
            U.extend(ul[m]); K.extend(A['curv'][m])
    a, b = np.polyfit(U, K, 1); m = (vv >= vlo) & (vv < vhi)
    print('  speed %.2f-%.2f m/s: G %.3f U0 %+4.0f n %d | lap_ff gain table g used by the band at these speeds: median %.3f' % (vlo, vhi, a, b / a, len(U), np.median(gt[m]) if m.any() else np.nan))
gA = fitset(ALL, 'steer', BT, BL)
out['plant'] = dict(G=float(gA[0]), U0=float(gA[1]), tau=BT, tl=BL)
# ---------- (d)
edges = np.arange(0.0, 6.31, 0.3); nb = len(edges) - 1


def cam_rows(runs, hi=6.3):
    X = []; Y = []; M = []
    for r in runs:
        for q in P[r]:
            A = q['A']; psi = Tf(A['xc']) - A['h']
            m = np.isfinite(A['y']) & np.isfinite(A['err']) & (A['xc'] >= 0) & (A['xc'] < hi) & (A['err'] < 0.98) & (A['valid'] > 0)
            for i in np.where(m)[0]:
                k = int(A['xc'][i] // 0.3); row = np.zeros(nb + 2); row[k] = 1; row[nb] = A['y'][i]; row[nb + 1] = psi[i]; X.append(row); Y.append(A['err'][i]); M.append((A['xc'][i], A['y'][i], psi[i]))
    return np.array(X), np.array(Y), np.array(M)


print('(d) camera near error = F(x) + B*y + C*psi  (psi = T(x) - gyro heading; 0.3 m bins)')
Xb, Yb, Mb = cam_rows(BASE + ['ccf1']); cb = np.linalg.lstsq(Xb, Yb, rcond=None)[0]
Xa, Ya, Ma = cam_rows(ALL); ca = np.linalg.lstsq(Xa, Ya, rcond=None)[0]
print('  fit without c6f3: B %.4f /cm  C %.4f /deg  residual sd %.3f | fit with all five runs: B %.4f  C %.4f  residual sd %.3f' % (cb[nb], cb[nb + 1], (Yb - Xb @ cb).std(), ca[nb], ca[nb + 1], (Ya - Xa @ ca).std()))
Xc, Yc, Mc = cam_rows(['c6f3'])
print('  c6f3 frames against the fit WITHOUT c6f3 (how well a table made the old way predicts a new path): residual by x bin (median; + = camera reads more than the model)')
rc = Yc - Xc @ cb; ra = Yc - Xc @ ca
for k in range(nb):
    m = (Mc[:, 0] >= edges[k]) & (Mc[:, 0] < edges[k + 1])
    if m.sum() > 3: print('    %.1f-%.1f  F %+.2f (all-run fit %+.2f) | c6f3 err med %+.2f  y %+5.1f  psi %+5.1f | residual old-kind fit %+.2f  all-run fit %+.2f  n %d' % (edges[k], edges[k + 1], cb[k], ca[k], np.median(Yc[m]), np.median(Mc[m, 1]), np.median(Mc[m, 2]), np.median(rc[m]), np.median(ra[m]), m.sum()))
out['cam'] = dict(B=float(ca[nb]), C=float(ca[nb + 1]), F=[[round(float(edges[k] + 0.15), 2), round(float(ca[k]), 3)] for k in range(nb)], sd=float((Ya - Xa @ ca).std()))
# ---------- (e)
print('(e) chain stand-in: final command = a + K * near error per 0.3 m bin (all runs; frames where the x-band share is 0 or absent)')
ch = []
for k in range(nb + 2):
    lo = k * 0.3; U = []; E = []
    for r in ALL:
        for q in P[r]:
            A = q['A']; own = np.nan_to_num(A['xb_own'], nan=0.) if r == 'c6f3' else np.zeros(len(A['x']))
            m = (A['xc'] >= lo) & (A['xc'] < lo + 0.3) & np.isfinite(A['err']) & np.isfinite(A['steer']) & (own <= 0.)
            U.extend(A['steer'][m]); E.extend(A['err'][m])
    if len(U) > 25:
        U = np.array(U); E = np.array(E); a, b = np.polyfit(E, U, 1); ch.append([round(lo + 0.15, 2), round(float(b), 1), round(float(a), 1)])
        print('    %.1f-%.1f  cmd = %+5.0f %+5.0f * e   residual sd %4.0f us  (err range %+.2f .. %+.2f, n %d)' % (lo, lo + 0.3, b, a, (U - (a * E + b)).std(), E.min(), E.max(), len(U)))
out['chain'] = ch
out['base_med'] = dict(x=[round(float(v), 2) for v in XG], h=[None if not np.isfinite(v) else round(float(v), 2) for v in Hm], y=[None if not np.isfinite(v) else round(float(v), 2) for v in Ym])
json.dump(out, open(p3.lib.HERE / 'model.json', 'w'), indent=0)
