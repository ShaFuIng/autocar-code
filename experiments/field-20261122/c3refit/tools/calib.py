"""calibration of the simple model against the car (must pass before the model is used on a new table):
 1) open-loop plant: each pass's LOGGED command through the plant -> heading at 1.8 / 2.4 / 3.3 / 4.2 / 5.4 m against the gyro (per run medians)
 2) closed loop, chain only  -> should look like run 6f61 / d60e (x-band not in control)
 3) closed loop, the car's x-band table (arc2.json 2b81e9b2) -> should look like run c6f3: deeper inside at 1.2-2.4 m, the +330 spike at 3.0-3.6 m, a milder corner,
    and the saturated error before the ramp foot
usage: python calib.py <arc2.json with the car's xband>"""
import json, sys, numpy as np, p3, sim
E = np.arange(0.3, 6.61, 0.3)
P = {r: p3.passes([r]) for r in ('6f61', 'd60e', 'c6f3')}
med = lambda r, k: p3.binmed(P[r], k, E)
vprof = {r: med(r, 'v') for r in P}


def vfun(r):
    xs = (E[:-1] + E[1:]) / 2; v = np.where(np.isfinite(vprof[r]), vprof[r], 0.9); v = np.clip(v, 0.7, 1.25)   # the 5 Hz wheel speed has outliers at the anchor change
    return lambda x: float(np.interp(x, xs, v))


print('1) open-loop plant check (G %.3f, U0 %+.0f, dead time %.2f s, lag %.2f s): heading error = model - gyro (deg) at x; started at 0.6 m from the gyro heading' % (sim.M['plant']['G'], sim.M['plant']['U0'], sim.M['plant']['tau'], sim.M['plant']['tl']))
Pm = sim.M['plant']
for r in P:
    D = []
    for q in P[r]:
        A = q['A']; t = A['t']; u = np.nan_to_num(A['steer']); us = np.interp(t - Pm['tau'], t, u); ul = np.empty(len(t)); ul[0] = us[0]
        for i in range(1, len(t)): ul[i] = ul[i - 1] + (us[i] - ul[i - 1]) * (1 - np.exp(-(t[i] - t[i - 1]) / Pm['tl']))
        k = Pm['G'] * (ul + Pm['U0']); vv = np.nan_to_num(A['v'], nan=0.95); i0 = int(np.argmin(np.abs(A['xc'] - 0.6))); hm = np.full(len(t), np.nan); hm[i0] = A['h'][i0]
        for i in range(i0 + 1, len(t)): hm[i] = hm[i - 1] + k[i] * (A['xc'][i] - A['xc'][i - 1])
        D.append([np.nanmedian((hm - A['h'])[(A['xc'] >= a - 0.1) & (A['xc'] < a + 0.1)]) for a in (1.8, 2.4, 3.3, 4.2, 5.4)])
    D = np.array(D); print('   %s n %d  median %s | sd %s' % (r, len(D), ' '.join('%+5.1f' % v for v in np.nanmedian(D, axis=0)), ' '.join('%4.1f' % v for v in np.nanstd(D, axis=0))))


def show(title, r, R):
    print(title); print('    x      | car: cmd   err    y    h    curv | model: cmd   err    y    h    curv  share')
    sm = lambda c: sim.binned(R, E, c)
    for k in range(len(E) - 1):
        print('   %.1f-%.1f |     %+5.0f %+.2f %+5.1f %6.1f %5.1f |       %+5.0f %+.2f %+5.1f %6.1f %5.1f  %.2f' % (E[k], E[k + 1], med(r, 'steer')[k], med(r, 'err')[k], med(r, 'y')[k], med(r, 'h')[k], med(r, 'curv')[k],
              sm(1)[k], sm(4)[k], sm(2)[k], sm(5)[k], sm(6)[k], sm(7)[k]))
    c = lambda a, b: np.sqrt(np.nanmean((a - b) ** 2)); f = (E[:-1] >= 0.6) & (E[:-1] < 4.4)
    print('   rms model - car, 0.6-4.5 m: cmd %.0f us, y %.1f cm, heading %.1f deg | 4.5-6.6 m: cmd %.0f us, heading %.1f deg' % (c(sm(1)[f], med(r, 'steer')[f]), c(sm(2)[f], med(r, 'y')[f]), c(sm(5)[f], med(r, 'h')[f]), c(sm(1)[~f], med(r, 'steer')[~f]), c(sm(5)[~f], med(r, 'h')[~f])))
    return R


def start(r):
    return dict(x0=0.3, h0=float(med(r, 'h')[0]), y0=float(med(r, 'y')[0]), v_of_x=vfun(r))


if __name__ == '__main__':
    xb = json.load(open(sys.argv[1], encoding='utf8'))['c3']['xband']
    Rs = [sim.run(None, seed=s, noise=0.05, **start('6f61')) for s in range(9)]
    show('2) chain only, started from the 6f61 medians at 0.3 m (median of 9 noisy runs shown as one concatenated set)', '6f61', np.concatenate(Rs))
    Rs = [sim.run(xb, seed=s, noise=0.05, **start('c6f3')) for s in range(9)]
    R = show("3) the car's x-band table (2b81e9b2), started from the c6f3 medians at 0.3 m", 'c6f3', np.concatenate(Rs))
    f = lambda Rr, lo, hi, c, fn: fn(Rr[(Rr[:, 0] >= lo) & (Rr[:, 0] < hi), c])
    print('   per model run: cmd max 3.0-3.6 m %s | err max 4.2-5.6 m %s | err max 5.6-6.6 m %s | y min 1.2-2.4 m %s' % (' '.join('%+.0f' % f(q, 3.0, 3.6, 1, np.max) for q in Rs), ' '.join('%.2f' % f(q, 4.2, 5.6, 4, np.max) for q in Rs),
          ' '.join('%.2f' % f(q, 5.6, 6.6, 4, np.max) for q in Rs), ' '.join('%+.0f' % f(q, 1.2, 2.4, 2, np.min) for q in Rs)))
    Rs0 = [sim.run(None, seed=s, noise=0.05, **start('6f61')) for s in range(9)]
    print('   chain only for comparison: cmd max 3.0-3.6 m %s | err max 4.2-5.6 m %s | err max 5.6-6.6 m %s | y min 1.2-2.4 m %s' % (' '.join('%+.0f' % f(q, 3.0, 3.6, 1, np.max) for q in Rs0), ' '.join('%.2f' % f(q, 4.2, 5.6, 4, np.max) for q in Rs0),
          ' '.join('%.2f' % f(q, 5.6, 6.6, 4, np.max) for q in Rs0), ' '.join('%+.0f' % f(q, 1.2, 2.4, 2, np.min) for q in Rs0)))
