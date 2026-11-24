"""c3front: where are the track features of curve 3 in odometer terms, and which landmark (C2e or C3s) places them with less scatter?
Features per lap: the left-bend peak (max positive gz), the first right bend (camera line curvature peak near x~1.6), the straight piece
(camera curvature minimum near x~2.9), the corner (max |gz| at x 4..5.5). Also the steady relation command -> curvature in the front part."""
import json, sys
import numpy as np
import c3data
EXP = c3data.EXP
rowsall = []
print('lap        C3s at C2e+ | corner peak: x(C3s) / C2e+ | straight piece (cam curv min): x / C2e+ | bend 1 (cam curv max 1..2.3): x / C2e+ | bend 2 (cam max 3..4.2): x / C2e+ | right turn starts (gz_lp < -12 first after the left bend): x / C2e+')
S = {k: [] for k in ('c3s', 'corner_x', 'corner_c', 'str_x', 'str_c', 'b1_x', 'b1_c', 'b2_x', 'b2_c', 'st_x', 'st_c')}
for run in c3data.RUNS:
    F = c3data.load(run); T = np.array([p['t'] for p, _, _ in F])
    odo = np.array([(p.get('lap_pos') or {}).get('odo_m') if (p.get('lap_pos') or {}).get('odo_m') is not None else np.nan for p, _, _ in F], float)
    k = [i for i in range(1, len(F)) if not np.isnan(odo[i]) and odo[i] != odo[i - 1]]; od = np.interp(T, T[k], odo[k])
    gz = np.array([(p.get('inputs') or {}).get('gz') or 0. for p, _, _ in F])
    last = max(((p.get('lap_pos') or {}).get('lap') or 0) for p, _, _ in F); c2e = {}; c3s = {}
    for i, (p, af, v) in enumerate(F):
        lp = p.get('lap_pos') or {}
        if lp.get('anchor') == 'C2e' and lp.get('lap') not in c2e and lp.get('since_anchor_m') is not None: c2e[lp['lap']] = lp['odo_m'] - lp['since_anchor_m']
        if lp.get('anchor') == 'C3s' and lp.get('lap') not in c3s and lp.get('since_anchor_m') is not None: c3s[lp['lap']] = lp['odo_m'] - lp['since_anchor_m']
    for lap in sorted(c3s):
        if lap < 1 or lap >= last or lap not in c2e: continue
        d3 = c3s[lap]; d2 = c2e[lap]; x = od - d3
        m = (x > 3.8) & (x < 5.8); ic = np.where(m)[0][np.argmin(gz[m])]
        fx = []; fk = []
        for i, (p, af, v) in enumerate(F):
            L = af.get('L') or {}
            if af.get('ok') and af.get('kappa_dpm') is not None and L.get('z_cm') and -3 < x[i] < 6:
                fx.append(x[i] + sum(L['z_cm']) / 200.); fk.append(af['kappa_dpm'])
        fx = np.array(fx); fk = np.array(fk)
        def ext(a, b, fn):
            mm = (fx >= a) & (fx <= b); return fx[mm][fn(fk[mm])] if mm.sum() else np.nan
        xs = ext(2.3, 3.4, np.argmax); xb1 = ext(1.0, 2.3, np.argmin); xb2 = ext(3.0, 4.2, np.argmin)
        # start of the right turn after the left bend: low-passed gz (0.3 s) first below -12 after the last frame with gz_lp > +5
        lpz = np.zeros(len(gz)); a = 0.
        for i in range(1, len(gz)):
            dt = min(.2, max(0., T[i] - T[i - 1])); a += (gz[i] - a) * (1 - np.exp(-dt / .3)); lpz[i] = a
        mm = np.where((x > -4.5) & (x < 1.0))[0]; pos = [i for i in mm if lpz[i] > 5.]
        st = np.nan
        if pos:
            for i in range(pos[-1], mm[-1]):
                if lpz[i] < -12.: st = x[i]; break
        vals = dict(c3s=d3 - d2, corner_x=x[ic], corner_c=od[ic] - d2, str_x=xs, str_c=xs + d3 - d2, b1_x=xb1, b1_c=xb1 + d3 - d2, b2_x=xb2, b2_c=xb2 + d3 - d2, st_x=st, st_c=st + d3 - d2)
        for kk in S: S[kk].append(vals[kk])
        print('%s L%d   %6.2f      | %5.2f / %5.2f           | %5.2f / %5.2f                         | %5.2f / %5.2f                         | %5.2f / %5.2f                  | %5.2f / %5.2f' % (
            run[4:8], lap, vals['c3s'], vals['corner_x'], vals['corner_c'], vals['str_x'], vals['str_c'], vals['b1_x'], vals['b1_c'], vals['b2_x'], vals['b2_c'], vals['st_x'], vals['st_c']))
print('scatter (sd, m) of each feature measured from C3s vs from C2e:')
for a, b, name in (('corner_x', 'corner_c', 'corner peak'), ('str_x', 'str_c', 'straight piece'), ('b1_x', 'b1_c', 'bend 1'), ('b2_x', 'b2_c', 'bend 2'), ('st_x', 'st_c', 'right-turn start')):
    print('   %-16s from C3s: mean %+.2f sd %.2f (min %+.2f max %+.2f) | from C2e: mean %.2f sd %.2f (min %.2f max %.2f)' % (name, np.nanmean(S[a]), np.nanstd(S[a]), np.nanmin(S[a]), np.nanmax(S[a]), np.nanmean(S[b]), np.nanstd(S[b]), np.nanmin(S[b]), np.nanmax(S[b])))
print('   C3s landmark at C2e + mean %.2f sd %.2f (min %.2f max %.2f)' % (np.mean(S['c3s']), np.std(S['c3s']), np.min(S['c3s']), np.max(S['c3s'])))
# command -> curvature in the front part (x 0.5..4.0): curvature(t) against the command sent 0.25 s earlier
A = c3data.all_laps(); X = []; Y = []; XS = []
for k, rows in A.items():
    v = c3data.speed(rows); t = np.array([r['t'] for r in rows]); xx = np.array([r['x'] for r in rows])
    st = np.array([(r['steer'] or np.nan) for r in rows], float); se = np.array([(r['sent'] or np.nan) for r in rows], float); gz = np.array([r['gz'] or 0. for r in rows])
    for i in range(len(rows)):
        if 0.5 <= xx[i] <= 4.0 and v[i] > .5:
            j = np.searchsorted(t, t[i] - .25)
            if not np.isnan(st[j]) and not np.isnan(se[j]): X.append(st[j] - 1475.); XS.append(se[j] - 1475.); Y.append(gz[i] / v[i])
X = np.array(X); XS = np.array(XS); Y = np.array(Y)
for nm, xx in (('proposal steer_us', X), ('sent steer_cmd', XS)):
    a = np.polyfit(xx, Y, 1); r = np.corrcoef(xx, Y)[0, 1]
    print('front part x 0.5..4.0 m, n=%d: curvature (deg/m) = %.3f * (%s - centre 1475) %+.1f   r=%.2f ; mean command %+.0f us, mean curvature %.1f deg/m -> command for -26 deg/m: %+.0f us' % (len(xx), a[0], nm, a[1], r, xx.mean(), Y.mean(), (-26. - a[1]) / a[0]))
