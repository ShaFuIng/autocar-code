"""diagnosis 2 (run c6f3 = x-band r1 owning, against 6f61 = shadow):
 (a) the encoder staircase inside the band: how much the band target moves on the frames where the controller's distance steps, against the other frames
 (b) where the turning happens: centre of the feed-forward of the car's table, of the car's curvature, and of the lane bends
 (c) the +330 us spike at 3.0-3.6 m, pass by pass: its parts and the car's state
 (d) arc_lat's +-20 cm range: share of frames without a value because the car was beyond it (the logged medians are cut there)
 (e) the tail: state after the corner"""
import json, numpy as np, p3
M = json.load(open(p3.lib.HERE / 'model.json')); Tf = lambda x: np.interp(x, [a for a, b in M['T']], [b for a, b in M['T']])
P6 = p3.passes(['6f61']); Pc = p3.passes(['c6f3']); Pb = p3.passes(['d60e', '84f2', '6f61'])
CAR = json.load(open(p3.lib.EXP / 'field-20261122' / 'c3c4' / 'on' / 'c3_own_step1_in10' / 'arc2.json', encoding='utf8'))['c3']['xband']

print('(a) frames with C3s + 0.9 .. 4.05 m (band share 1). "step frame" = the distance the band uses changed on this frame (5 Hz encoder; the band sees the step one frame after lap_pos logs it).')
for name, P, key in (('c6f3 final command (band owns)', Pc, 'steer'), ('6f61 band target (shadow)', P6, 'xb_tgt'), ('6f61 final command (chain)', P6, 'steer')):
    ds = []; dn = []; dk = []; de = []
    for q in P:
        A = q['A']; m = (A['x'] >= 0.9) & (A['x'] < 4.05); ii = np.where(m)[0]
        for a, b in zip(ii[:-1], ii[1:]):
            if b != a + 1 or not np.isfinite(A[key][a]) or not np.isfinite(A[key][b]): continue
            d = abs(A[key][b] - A[key][a])
            xk = A['xb_x'] if (np.isfinite(A['xb_x'][a]) and np.isfinite(A['xb_x'][b])) else None   # the band's own distance (lap_pos of the previous update)
            if (xk[b] - xk[a] > 0.02) if xk is not None else (A['x'][a] - A['x'][a - 1] > 0.02):
                ds.append(d)
                if np.isfinite(A['xb_prof'][a]) and np.isfinite(A['xb_prof'][b]): dk.append(abs(A['xb_prof'][b] - A['xb_prof'][a])); de.append(abs(300. * (A['xb_ec'][b] - A['xb_ec'][a])))
            else: dn.append(d)
    ds = np.array(ds); dn = np.array(dn)
    print('   %-32s step frames n %3d: |change| median %3.0f P90 %3.0f max %3.0f us | other frames n %4d: median %3.0f P90 %3.0f max %3.0f us%s' % (name, len(ds), np.median(ds), np.percentile(ds, 90), ds.max(), len(dn), np.median(dn), np.percentile(dn, 90), dn.max(),
          (' | on step frames the feed-forward alone moves median %.0f max %.0f us, the error centre (x K 300) median %.0f max %.0f us' % (np.median(dk), np.max(dk), np.median(de), np.max(de))) if dk else ''))
st = np.concatenate([np.diff(q['A']['x'][(q['A']['x'] >= 0.9) & (q['A']['x'] < 4.05)]) for q in Pc]); st = st[st > 0.02]
print('   size of a distance step: median %.2f m, P90 %.2f m, max %.2f m; slope of the car table: feed-forward up to %.0f us per m, error centre up to %.2f per m' % (np.median(st), np.percentile(st, 90), st.max(),
      max(abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(CAR['kappa_x'], CAR['kappa_x'][1:])) / 0.16, max(abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(CAR['e_c_x'], CAR['e_c_x'][1:]))))

print('(b) centre (weighted mean position) of the turning in bend 1 (0.9-2.7 m) and bend 2 (2.7-4.3 m)')
cen = lambda xs, w: float(np.sum(xs * w) / np.sum(w))
xs = np.arange(0.9, 4.31, 0.05); kT = np.gradient(Tf(xs), xs); kx = np.interp(xs, [a for a, b in CAR['kappa_x']], [b for a, b in CAR['kappa_x']])
for lo, hi in ((0.9, 2.7), (2.7, 4.3)):
    m = (xs >= lo) & (xs < hi); line = '   %.1f-%.1f m: lane bend centre %.2f m (%.0f deg) | car table feed-forward centre %.2f m' % (lo, hi, cen(xs[m], np.clip(kT[m], 0, None)), np.trapezoid(np.clip(kT[m], 0, None), xs[m]), cen(xs[m], kx[m]))
    for name, P in (('6f61 (chain)', P6), ('c6f3 (band)', Pc)):
        c = []
        for q in P:
            A = q['A']; mm = (A['xc'] >= lo) & (A['xc'] < hi) & np.isfinite(A['curv'])
            c.append(cen(A['xc'][mm], np.clip(A['curv'][mm], 0, None)))
        line += ' | %s car curvature centre %.2f m (sd %.2f)' % (name, np.mean(c), np.std(c))
    print(line)
print('   plant dead time + lag %.2f s = %.2f m at 1.05 m/s: a feed-forward centred that far before the lane bend puts the turning on the bend.' % (M['plant']['tau'] + M['plant']['tl'], (M['plant']['tau'] + M['plant']['tl']) * 1.05))

print('(c) c6f3, 2.7-3.6 m, per pass (medians per 0.3 m of the continuous distance): final cmd | band parts: feed-forward, error term, gyro term -> target (cap +330) | band share | near error vs table e_c | y, psi (= lane heading - car heading) | curve_ff applied %')
for q in Pc:
    A = q['A']; psi = Tf(A['xc']) - A['h']
    for a in (2.7, 3.0, 3.3):
        m = (A['xc'] >= a) & (A['xc'] < a + 0.3); md = lambda k: np.nanmedian(A[k][m]) if np.isfinite(A[k][m].astype(float)).any() else np.nan
        print('   lap %d %.1f-%.1f: cmd %+4.0f (max %+4.0f) | %+4.0f %+4.0f %+3.0f -> %+4.0f | %.2f | err %+.2f vs e_c %.2f | y %+5.1f psi %+5.1f | %3.0f' % (q['lap'], a, a + 0.3, md('steer'), np.nanmax(A['steer'][m]), md('xb_prof'), md('xb_e'), md('xb_gz'), md('xb_tgt'),
              np.nanmean(A['xb_own'][m]), md('err'), md('xb_ec'), md('y'), np.nanmedian(psi[m]), 100 * np.nanmean(A['cff_ap'][m])))

print('(d) share of frames where arc_lat gave no value (in curve 3 almost always Y_RANGE = beyond +-20 cm), per 0.3 m: the logged y medians and the -12..-13 cm inner-clearance minima are cut at that range')
E = np.arange(0.6, 4.81, 0.3)
for name, P in (('6f61', P6), ('c6f3', Pc)):
    print('   %s: ' % name + ' '.join('%.1f:%2.0f%%' % (a, 100 * np.mean(~np.isfinite(np.concatenate([q['A']['y'][(q['A']['xc'] >= a) & (q['A']['xc'] < a + 0.3)] for q in P])))) for a in E[:-1]))

print('(e) after the corner (medians): heading since C3s at 5.1 / 5.4 / 5.7 / 6.0 m | y at 5.7 / 6.0 | curvature 4.8-5.4 m | near error 5.4-6.0 m | cmd max 6.0-6.6 m')
for name, P in (('no x-band (15 passes)', Pb), ('c6f3 (4 passes)', Pc)):
    f = lambda k, a, b, fn=np.nanmedian: fn(np.concatenate([q['A'][k][(q['A']['xc'] >= a) & (q['A']['xc'] < b)] for q in P]).astype(float))
    print('   %-22s h %5.1f %5.1f %5.1f %5.1f | y %+5.1f %+5.1f | curvature %4.1f | err %+.2f | cmd max per pass %s' % (name, f('h', 5.0, 5.2), f('h', 5.3, 5.5), f('h', 5.6, 5.8), f('h', 5.9, 6.1), f('y', 5.6, 5.8), f('y', 5.9, 6.1), f('curv', 4.8, 5.4), f('err', 5.4, 6.0),
          ' '.join('%+.0f' % np.nanmax(q['A']['steer'][(q['A']['xc'] >= 6.0) & (q['A']['xc'] < 6.6)]) for q in P)))
print('   lane heading (model.json) at 5.3 / 5.6 / 5.9 / 6.2 m: %.0f %.0f %.0f %.0f' % (Tf(5.3), Tf(5.6), Tf(5.9), Tf(6.2)))
