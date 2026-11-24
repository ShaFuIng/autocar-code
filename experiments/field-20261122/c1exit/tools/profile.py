"""curve profile by the heading turned since the straight before the curve (tb, deg, right +; integrated gyro, reference = mean heading on the straight).
  python profile.py C1e|C2e [runs...]      per 10-deg bin: driven curvature (-gz/v), pre-play command, error, lateral position, speed; C1e: split by the late slam
The median driven curvature per tb bin over many passes ~ the lane curvature at that heading (every pass drives the same lane)."""
import sys, numpy as np, lib, passes
an = sys.argv[1] if len(sys.argv) > 1 else 'C1e'
runs = sys.argv[2:] or (lib.MAIN + lib.MORE if an == 'C1e' else lib.MAIN)
back = (6.0, 3.3) if an == 'C1e' else (8.5, 5.6)
P = []
for run in runs:
    Q, A = passes.run_arrays(run)
    for i in passes.exits(A, an):
        t = A['t']; ref = passes.straight_ref(A, i, back); tb = ref - A['yaw']
        m = (t >= t[i] - (3.6 if an == 'C1e' else 5.8)) & (t <= t[i] + 2.2)
        A12 = (t >= t[i] - 1.2) & (t < t[i])
        P.append(dict(run=run, lap=int(A['lap'][i]), slam=float(np.nanmax(A['sent'][A12])), tb=tb[m], k=-A['gz'][m] / np.where(A['v'][m] > 0.5, A['v'][m], np.nan), pre=A['pre'][m], e=A['e'][m], y=A['y'][m], psi=A['psi'][m],
                      v=A['v'][m], gz=A['gz'][m], t=t[m] - t[i], x=A['x'][m], anc=A['anc'][m], tb_exit=tb[i]))
print('%s: %d passes; tb at the first %s frame: median %.1f (p10 %.1f p90 %.1f); final heading (mean 1.2..2.2 s after): median %.1f (p10 %.1f p90 %.1f)' % (
    an, len(P), an, np.median([p['tb_exit'] for p in P]), np.percentile([p['tb_exit'] for p in P], 10), np.percentile([p['tb_exit'] for p in P], 90),
    *np.percentile([np.nanmean(p['tb'][(p['t'] > 1.2) & (p['t'] < 2.2)]) for p in P], [50, 10, 90])))
groups = [('all', P)]
if an == 'C1e': groups += [('late slam (sent >= 430 in the last 1.2 s)', [p for p in P if p['slam'] >= 430]), ('no late slam (< 300)', [p for p in P if p['slam'] < 300])]
hi = 160 if an == 'C1e' else 220
for name, G in groups:
    print('\n%s: %d passes' % (name, len(G)))
    print('  tb bin   n  | curvature med (p25 p75) | pre med (p10 p90) | err med (p10 p90) |  y med  psi med |  v   | gz med')
    for a in range(-10, hi, 10):
        c = lambda k: np.concatenate([p[k][(p['tb'] >= a) & (p['tb'] < a + 10) & (p['t'] < 0.6)] for p in G])
        k = c('k'); k = k[np.isfinite(k)]
        if len(k) < 8: continue
        pre = c('pre'); e = c('e'); y = c('y'); ps = c('psi'); v = c('v'); gz = c('gz')
        print('  %4d-%4d %4d | %5.1f (%5.1f %5.1f)     | %+5.0f (%+5.0f %+5.0f) | %+.2f (%+.2f %+.2f) | %+5.1f  %+5.1f   | %.2f | %+5.1f' % (
            a, a + 10, len(k), np.median(k), np.percentile(k, 25), np.percentile(k, 75), np.nanmedian(pre), np.nanpercentile(pre, 10), np.nanpercentile(pre, 90), np.nanmedian(e), np.nanpercentile(e, 10), np.nanpercentile(e, 90),
            np.nanmedian(y) if np.isfinite(y).any() else np.nan, np.nanmedian(ps) if np.isfinite(ps).any() else np.nan, np.nanmedian(v), np.nanmedian(gz)))
