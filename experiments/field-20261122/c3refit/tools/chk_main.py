"""re-computation of the main session's numbers (0.6 m bins of the controller's staircase distance, medians) for runs c6f3 and 6f61."""
import numpy as np, p3
E = np.arange(0.6, 7.21, 0.6)
for r in ('c6f3', '6f61'):
    P = p3.passes([r]); B = lambda k, fn=np.nanmedian: p3.binmed(P, k, E, xk='x', fn=fn)
    print(r, 'x bins from 0.6 m, 0.6 m wide:')
    print('   y (cm)        ' + ' '.join('%+6.1f' % v for v in B('y')))
    print('   cmd - centre  ' + ' '.join('%+6.0f' % v for v in B('steer')))
    print('   cmd max       ' + ' '.join('%+6.0f' % v for v in B('steer', np.nanmax)))
    print('   near error    ' + ' '.join('%+6.2f' % v for v in B('err')))
    print('   curve_ff %    ' + ' '.join('%6.0f' % (100 * v) for v in B('cff_ap', np.nanmean)))
    print('   no arc_lat y %' + ' '.join('%6.0f' % (100 * np.mean(~np.isfinite(np.concatenate([q['A']['y'][(q['A']['x'] >= a) & (q['A']['x'] < a + 0.6)] for q in P])))) for a in E[:-1]))
    if r == '6f61':
        d = lambda lo, hi: np.nanmedian(np.concatenate([(q['A']['xb_tgt'] - q['A']['steer'])[(q['A']['x'] >= lo) & (q['A']['x'] < hi)] for q in P]))
        print('   r1 target - actual: ' + ' '.join('%.2f-%.2f %+4.0f' % (a, a + 0.45, d(a, a + 0.45)) for a in np.arange(0.9, 4.5, 0.45)))
    for q in P:
        A = q['A']; f = lambda lo, hi, k, fn: fn(A[k][(A['x'] >= lo) & (A['x'] < hi)].astype(float))
        print('   lap %d: corner 4.2-5.6 err max %.2f cmd max %+4.0f gz min %+4.0f cout min %+5.1f | tail 6.0-7.2 err max %.2f cmd max %+4.0f' % (q['lap'], f(4.2, 5.6, 'err', np.nanmax), f(4.2, 5.6, 'steer', np.nanmax), f(4.2, 5.6, 'gz', np.nanmin), f(4.2, 5.6, 'cout', np.nanmin), f(6.0, 7.2, 'err', np.nanmax), f(6.0, 7.2, 'steer', np.nanmax)))
