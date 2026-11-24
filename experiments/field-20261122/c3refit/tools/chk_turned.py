"""which heading reference is the cleanest? lane heading = car heading + arc_lat psi at fixed distances, spread over the passes, for three car-heading references:
 h      = gyro integral zeroed where the continuous distance crosses C3s (offline only)
 turned = the c3 layer's turned_deg (heading at the C3s crossing looked up in the lap_pos history; the car has this today)
 H      = heading relative to the lap_pos base plateau (circle straight, C2e + 3.5 .. 6.5 m; candidate reference, C3s is defined as H = 10 deg)"""
import numpy as np, p3
P = p3.passes(['d60e', '84f2', '6f61'])
print('per pass: H - h, turned - h (deg, medians over 0.6-5.5 m)')
for q in P:
    A = q['A']; m = (A['xc'] > 0.6) & (A['xc'] < 5.5); mt = m & np.isfinite(A['turned'])
    print('  %s %d  H-h %+5.1f  turned-h %s  H at the first C3s frame %.1f' % (q['run'], q['lap'], np.nanmedian((A['H'] - A['h'])[m]), ('%+5.1f' % np.median((A['turned'] - A['h'])[mt])) if mt.sum() > 5 else '  -  ', A['H'][int(np.argmin(np.abs(A['x'] - 0.0)))]))
print('spread (sd, deg) over the passes of [car heading + arc_lat psi] at x:')
for x0 in (0.9, 1.2, 2.6, 2.9, 3.2, 4.3):
    r = {'h': [], 'turned': [], 'H': []}
    for q in P:
        A = q['A']; m = (A['xc'] >= x0 - 0.15) & (A['xc'] < x0 + 0.15)
        for k in r:
            v = (A[k] + A['psi'])[m]; r[k].append(np.nanmedian(v) if np.isfinite(v).any() else np.nan)
    print('  x %.1f  h %.1f  turned %.1f  H %.1f   (n %d / %d / %d)' % (x0, np.nanstd(r['h']), np.nanstd(r['turned']), np.nanstd(r['H']), np.isfinite(r['h']).sum(), np.isfinite(r['turned']).sum(), np.isfinite(r['H']).sum()))
print('spread (sd, deg) of the CAR heading itself at x (driving scatter + reference scatter):')
for x0 in (0.6, 1.2, 2.4, 3.0, 4.2):
    r = {'h': [], 'turned': [], 'H': []}
    for q in P:
        A = q['A']; m = (A['xc'] >= x0 - 0.1) & (A['xc'] < x0 + 0.1)
        for k in r: r[k].append(np.nanmedian(A[k][m]) if np.isfinite(A[k][m]).any() else np.nan)
    print('  x %.1f  h %.1f  turned %.1f  H %.1f  | medians h %.1f turned %.1f H %.1f' % (x0, np.nanstd(r['h']), np.nanstd(r['turned']), np.nanstd(r['H']), np.nanmedian(r['h']), np.nanmedian(r['turned']), np.nanmedian(r['H'])))
