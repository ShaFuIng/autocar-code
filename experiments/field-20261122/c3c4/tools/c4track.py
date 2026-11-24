"""curve 4: lane shape seen from the car. track heading TT = band turned angle (car, right +) + arc_lat psi (car left of the lane +).
kappa_track = d TT / d x (local line fit per pass, then median). Also the same for the car (d tb / d x) as a check of -gz/v."""
import numpy as np, lib
from c4pass import passes
P = passes()


def slope(x, y, x0, h):
    m = (x >= x0 - h) & (x <= x0 + h) & np.isfinite(x) & np.isfinite(y)
    if m.sum() < 6 or np.ptp(x[m]) < h: return np.nan
    return np.polyfit(x[m], y[m], 1)[0]


print(' x4    n  TT(med)  tb(med)  psi(med p25 p75)  kT(med p25 p75)  kCar(med)   y    | by track angle')
rows = []
for x0 in np.arange(0.9, 8.4, 0.3):
    TT = []; TB = []; PS = []; KT = []; KC = []; Y = []
    for q in P:
        A = q['A']; m = (A['x4'] >= x0 - 0.15) & (A['x4'] < x0 + 0.15)
        if not m.any(): continue
        tt = A['tb'] + A['psi']
        TT.append(np.nanmedian(tt[m])); TB.append(np.nanmedian(A['tb'][m])); PS.append(np.nanmedian(A['psi'][m])); Y.append(np.nanmedian(A['y'][m]))
        KT.append(slope(A['x4'], tt, x0, 0.45)); KC.append(slope(A['x4'], A['tb'], x0, 0.45))
    f = np.nanmedian; pc = np.nanpercentile
    rows.append((x0, f(TT), f(TB), f(KT), f(KC)))
    print('%5.2f %3d  %6.1f  %6.1f   %+5.1f %+5.1f %+5.1f   %5.1f %5.1f %5.1f   %5.1f   %+5.1f' % (x0, np.isfinite(KT).sum(), f(TT), f(TB), f(PS), pc(PS, 25), pc(PS, 75), f(KT), pc(KT, 25), pc(KT, 75), f(KC), f(Y)))
print('\n by car turned angle tb (10 deg bins): kT = dTT/dx, kCar')
for a in np.arange(30, 200, 10):
    KT = []; KC = []; PS = []
    for q in P:
        A = q['A']; m = (A['tb'] >= a) & (A['tb'] < a + 10) & np.isfinite(A['x4'])
        if m.sum() < 3: continue
        xc = np.nanmedian(A['x4'][m]); tt = A['tb'] + A['psi']
        KT.append(slope(A['x4'], tt, xc, 0.45)); KC.append(slope(A['x4'], A['tb'], xc, 0.45)); PS.append(np.nanmedian(A['psi'][m]))
    print('  tb %3d-%3d  kT %5.1f (%5.1f..%5.1f)  kCar %5.1f  psi %+5.1f  n %d' % (a, a + 10, np.nanmedian(KT), np.nanpercentile(KT, 25), np.nanpercentile(KT, 75), np.nanmedian(KC), np.nanmedian(PS), np.isfinite(KT).sum()))
