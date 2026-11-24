"""c3front: is the two-step turning of the curve-3 front part in the track (seen by the camera on every lap at the same place) or in the control?
Per lap, in 0.25 m bins of x (m after C3s): car curvature gz/v (deg/m), camera line curvature arc_fit.kappa_dpm placed at the middle of
its fitted span (x + z_mid), arc_lat map curvature at the car, heading (integral of gz) and its residual to a constant-radius arc."""
import sys
import numpy as np
import c3data
A = c3data.all_laps(c3data.RUNS + (['run-9284-20261121'] if '--more' in sys.argv else []))
B = np.arange(-1.0, 7.01, 0.25)
def binned(xs, ys):
    xs = np.array(xs, float); ys = np.array(ys, float); o = []
    for a, b in zip(B[:-1], B[1:]):
        m = (xs >= a) & (xs < b) & ~np.isnan(ys); o.append(np.mean(ys[m]) if m.sum() else np.nan)
    return np.array(o)
car = {}; fit = {}; mapk = {}; head = {}; spd = {}; res = {}
for k, rows in A.items():
    v = c3data.speed(rows); x = np.array([r['x'] for r in rows]); gz = np.array([r['gz'] if r['gz'] is not None else np.nan for r in rows])
    kc = np.where(v > 0.3, gz / np.maximum(v, 0.3), np.nan)
    car[k] = binned(x, kc); spd[k] = binned(x, v)
    fx = []; fy = []
    for r in rows:
        if r['af_ok'] and r['af_k'] is not None and r['af_z'][0] is not None:
            fx.append(r['x'] + (r['af_z'][0] + r['af_z'][1]) / 200.); fy.append(r['af_k'])
    fit[k] = binned(fx, fy)
    mapk[k] = binned([r['x'] for r in rows if r['al_kap'] is not None], [r['al_kap'] for r in rows if r['al_kap'] is not None])
    t = np.array([r['t'] for r in rows]); dt = np.diff(t, prepend=t[0]); dt[dt > .3] = 0
    h = np.cumsum(np.nan_to_num(gz) * dt); i0 = np.searchsorted(x, 0.); h = h - h[min(i0, len(h) - 1)]
    head[k] = binned(x, h)
    m = (x >= 0) & (x <= 4.25)
    a = np.polyfit(x[m], h[m], 1)   # constant-radius arc = heading linear in distance
    res[k] = (a[0], np.sqrt(np.mean((h[m] - np.polyval(a, x[m])) ** 2)), np.max(np.abs(h[m] - np.polyval(a, x[m]))))
keys = list(A)
def table(title, D, fmt='%6.0f'):
    print(title); print('  x m      ' + ' '.join('%6s' % ('%s%d' % (k[0][4:6], k[1])) for k in keys) + '    median   sd')
    for i, a in enumerate(B[:-1]):
        vals = np.array([D[k][i] for k in keys]); print('  %+.2f    ' % a + ' '.join((fmt % v) if not np.isnan(v) else '     -' for v in vals) + '    ' + (fmt % np.nanmedian(vals)) + ' ' + ('%5.1f' % np.nanstd(vals)))
table('A. car curvature gz/v, deg per metre (negative = right)', car)
table('B. camera line curvature (arc_fit.kappa_dpm at the middle of the fitted span), deg per metre', fit)
table('C. arc_lat map curvature at the car (kap_dpm), deg per metre', mapk)
table('D. speed m/s', spd, '%6.2f')
table('E. heading turned since C3s, deg', head)
print('F. heading vs distance 0..4.25 m, straight-line fit (= constant-radius arc): slope deg/m (radius m), rms residual deg, max residual deg')
for k in keys: print('   %s L%d  %+.1f deg/m (R %.2f m)  rms %.1f  max %.1f' % (k[0][4:8], k[1], res[k][0], 57.3 / abs(res[k][0]), res[k][1], res[k][2]))
# correlation between laps: is the dip at the same x on every lap?
M = np.array([car[k] for k in keys]); i0 = list(B).index(0.); i1 = list(B).index(4.25)
c = np.corrcoef(M[:, i0:i1]); print('G. lap-to-lap correlation of the car-curvature profile over x 0..4.25 m: median off-diagonal r = %.2f (min %.2f)' % (np.median(c[np.triu_indices(len(keys), 1)]), c[np.triu_indices(len(keys), 1)].min()))
Fm = np.array([fit[k] for k in keys]); cf = np.corrcoef(np.nan_to_num(Fm[:, i0:i1], nan=np.nanmean(Fm))); print('   same for the camera line curvature: median r = %.2f' % np.median(cf[np.triu_indices(len(keys), 1)]))
mc = np.nanmedian(M, 0); mf = np.nanmedian(Fm, 0); ok = ~np.isnan(mc[i0:i1]) & ~np.isnan(mf[i0:i1])
print('H. median car curvature vs median camera curvature over x 0..4.25 m: r = %.2f ; camera min/max %.0f/%.0f deg/m, car min/max %.0f/%.0f deg/m' % (np.corrcoef(mc[i0:i1][ok], mf[i0:i1][ok])[0, 1], np.nanmin(mf[i0:i1]), np.nanmax(mf[i0:i1]), np.nanmin(mc[i0:i1]), np.nanmax(mc[i0:i1])))
