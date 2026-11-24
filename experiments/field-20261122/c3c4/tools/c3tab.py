"""curve 3 by distance after C3s (0.3 m bins): what each layer does. usage: python c3tab.py [base|trim]"""
import sys, numpy as np, lib
from c3pass import passes

which = sys.argv[1] if len(sys.argv) > 1 else 'base'
P = passes(lib.BASE_RUNS if which == 'base' else ['ccf1'])
print(which, 'passes', len(P))
print('  x          n   v   pwm | err med/p90/max  cerr |  P    I    D  | des  yawin cff_in | cff% cff_floor | a3% a3_floor a3_a | trim | steer med/p90/max sent | curv  kpath | y    cin med/min  cout med/min | owner')
for a in np.arange(-0.9, 7.5, 0.3):
    R = {}
    for q in P:
        A = q['A']; m = (A['x'] >= a) & (A['x'] < a + 0.3)
        for k in A: R.setdefault(k, []).extend(A[k][m].tolist())
        R.setdefault('curv', []).extend((-A['gz'][m] / np.where(A['v'][m] > 0.3, A['v'][m], np.nan)).tolist())
    if not R['x']: continue
    F = lambda k: np.array(R[k], float)
    md = lambda k: np.nanmedian(F(k)) if np.isfinite(F(k)).any() else float('nan')
    pc = lambda k, p: np.nanpercentile(F(k), p) if np.isfinite(F(k)).any() else float('nan')
    mn = lambda k: np.nanmin(F(k)) if np.isfinite(F(k)).any() else float('nan')
    ow = {}
    for o in R['owner']: ow[o] = ow.get(o, 0) + 1
    ows = ' '.join('%s:%d' % (k, 100 * v / len(R['owner'])) for k, v in sorted(ow.items(), key=lambda kv: -kv[1])[:3])
    print('  %+4.1f..%+4.1f %4d %.2f %3.0f | %+.2f/%+.2f/%+.2f %+.2f | %+4.0f %+4.0f %+4.0f | %+4.0f %+4.0f %+4.0f | %3.0f %+5.0f | %3.0f %+5.0f %.2f | %2.0f | %+4.0f/%+4.0f/%+4.0f %+4.0f | %5.1f %5.1f | %+5.1f %+5.1f/%+5.1f %+5.1f/%+5.1f | %s' % (
        a, a + 0.3, len(R['x']), md('v'), md('pwm'), md('err'), pc('err', 90), np.nanmax(F('err')), md('cerr'), md('P'), md('I'), md('D'), md('des'), md('yi_out'), md('cff_in'),
        100 * np.nanmean(F('cff_ap')), md('cff_floor'), 100 * np.nanmean(F('a3_ap')), md('a3_floor'), md('a3_a'), md('trim'), md('steer'), pc('steer', 90), np.nanmax(F('steer')), md('sent'), md('curv'), -md('kpath'),
        md('y'), md('cin'), mn('cin'), md('cout'), mn('cout'), ows))
