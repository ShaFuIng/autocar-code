"""c3front: which control layer makes the command fall back in the curve-3 front part? Median over the laps per 0.25 m bin of x."""
import collections, sys
import numpy as np
import c3data
A = c3data.all_laps()
B = np.arange(-1.0, 7.01, 0.25)
cols = ['e', 'P', 'I', 'D', 'desired', 'steer', 'sent', 'yi_u', 'yi_gz_lp', 'gz', 'pwm', 'af_k', 'af_psi', 'al_y', 'al_clr_in']
acc = {c: collections.defaultdict(list) for c in cols}; own = collections.defaultdict(collections.Counter); rea = collections.defaultdict(collections.Counter); cen = []
for k, rows in A.items():
    for r in rows:
        b = int(np.floor((r['x'] + 1.0) / 0.25))
        if not 0 <= b < len(B) - 1: continue
        for c in cols:
            if isinstance(r[c], (int, float)): acc[c][b].append(r[c])
        own[b][r['owner']] += 1; rea[b][(r['sa_reason'], r['bz_cap'])] += 1
        if r['center']: cen.append(r['center'])
C = np.median(cen) if cen else 1475.
print('centre %.0f us. Median over %d laps (runs e1b4, b93e, 9934). us columns are relative to the centre except P/I/D (PID terms, us).' % (C, len(A)))
print('  x m    err    P    I    D  desired steer  sent | yaw_inner u  gz_lp   gz | pwm | cam line curv, psi_car | y_cm clr_in | owner (top 2) | speed_adapt reason, brake cap (top 2)')
med = lambda c, b: np.median(acc[c][b]) if acc[c][b] else float('nan')
for b, a in enumerate(B[:-1]):
    print('  %+.2f  %+.2f %4.0f %4.0f %4.0f   %+5.0f %+5.0f %+5.0f |   %+5.0f    %+5.0f %+5.0f | %3.0f |  %+5.0f  %+5.0f | %+5.0f %+5.0f | %s | %s' % (
        a, med('e', b), med('P', b), med('I', b), med('D', b), med('desired', b) - C, med('steer', b) - C, med('sent', b) - C, med('yi_u', b), med('yi_gz_lp', b), med('gz', b), med('pwm', b),
        med('af_k', b), med('af_psi', b), med('al_y', b), med('al_clr_in', b), ' '.join('%s %d%%' % (o, 100 * n / sum(own[b].values())) for o, n in own[b].most_common(2)),
        ' '.join('%s/%s %d%%' % (str(o[0])[:11], o[1], 100 * n / sum(rea[b].values())) for o, n in rea[b].most_common(2))))
# lateral deviation of the car's own path from its best constant-radius arc over x 0..4.25 m
print('\nLateral deviation of the driven path from the best constant-radius arc (x 0..4.25 m), per lap: max |dev| cm ; and the same for the camera line (integrating arc_fit curvature placed at x + z_mid)')
for k, rows in A.items():
    x = np.array([r['x'] for r in rows]); t = np.array([r['t'] for r in rows]); gz = np.array([r['gz'] or 0. for r in rows]); dt = np.diff(t, prepend=t[0]); dt[dt > .3] = 0
    h = np.cumsum(gz * dt); m = (x >= 0) & (x <= 4.25); xs = x[m]; hs = h[m]
    a = np.polyfit(xs, hs, 1); dh = np.radians(hs - np.polyval(a, xs)); ds = np.diff(xs, prepend=xs[0]); lat = np.cumsum(np.sin(dh) * ds) * 100
    lat = lat - np.polyval(np.polyfit(xs, lat, 1), xs)   # remove the free offset / heading of the reference arc
    fx = np.array([r['x'] + (r['af_z'][0] + r['af_z'][1]) / 200. for r in rows if r['af_ok'] and r['af_k'] is not None and r['af_z'][0] is not None]); fk = np.array([r['af_k'] for r in rows if r['af_ok'] and r['af_k'] is not None and r['af_z'][0] is not None])
    o = np.argsort(fx); fx = fx[o]; fk = fk[o]; g = np.arange(0., 4.26, 0.05); kk = np.interp(g, fx, fk); th = np.cumsum(kk) * 0.05
    a2 = np.polyfit(g, th, 1); d2 = np.radians(th - np.polyval(a2, g)); l2 = np.cumsum(np.sin(d2)) * 0.05 * 100; l2 = l2 - np.polyval(np.polyfit(g, l2, 1), g)
    print('   %s L%d  car path: %.1f cm (heading residual max %.1f deg) | camera line: mean curvature %.1f deg/m (R %.2f m), max |dev| %.1f cm, heading residual max %.1f deg' % (k[0][4:8], k[1], np.max(np.abs(lat)), np.degrees(np.max(np.abs(dh))), a2[0], 57.3 / abs(a2[0]), np.max(np.abs(l2)), np.degrees(np.max(np.abs(d2)))))
