"""curve 3: lane shape from the car: lane heading = car heading (gyro integral since x = -0.9 m, right +) + arc_lat psi (car left of the lane +).
kT = d(lane heading)/dx, kCar = d(car heading)/dx (line fits over +-0.3 m per pass, medians of the 20 passes without the curve-3 trim)."""
import numpy as np, lib
from c3pass import passes
P = passes(lib.BASE_RUNS)


def slope(x, y, x0, h):
    m = (x >= x0 - h) & (x <= x0 + h) & np.isfinite(x) & np.isfinite(y)
    if m.sum() < 6 or np.ptp(x[m]) < h: return np.nan
    return np.polyfit(x[m], y[m], 1)[0]


H = []
for q in P:
    A = q['A']; t = A['t']; gz = np.nan_to_num(A['gz']); h = -np.concatenate([[0], np.cumsum((gz[1:] + gz[:-1]) / 2 * np.diff(t))])
    i0 = np.argmin(np.abs(A['x'] - 0.0)); h = h - h[i0]; q['h'] = h; q['tt'] = h + A['psi']
print('  x     car heading  lane heading  psi(med p25 p75)   kCar   kT(med p25 p75)   y(med)  err   steer')
for x0 in np.arange(-0.6, 6.7, 0.3):
    r = {k: [] for k in 'h tt psi kc kt y err steer'.split()}
    for q in P:
        A = q['A']; m = (A['x'] >= x0 - 0.15) & (A['x'] < x0 + 0.15)
        if not m.any(): continue
        r['h'].append(np.nanmedian(q['h'][m])); r['tt'].append(np.nanmedian(q['tt'][m])); r['psi'].append(np.nanmedian(A['psi'][m])); r['y'].append(np.nanmedian(A['y'][m])); r['err'].append(np.nanmedian(A['err'][m])); r['steer'].append(np.nanmedian(A['steer'][m]))
        r['kc'].append(slope(A['x'], q['h'], x0, 0.3)); r['kt'].append(slope(A['x'], q['tt'], x0, 0.3))
    f = np.nanmedian; pc = np.nanpercentile
    print(' %+4.1f   %6.1f      %6.1f      %+5.1f %+5.1f %+5.1f   %5.1f   %5.1f %5.1f %5.1f   %+5.1f  %+.2f  %+4.0f' % (x0, f(r['h']), f(r['tt']), f(r['psi']), pc(r['psi'], 25), pc(r['psi'], 75), f(r['kc']), f(r['kt']), pc(r['kt'], 25), pc(r['kt'], 75), f(r['y']), f(r['err']), f(r['steer'])))
