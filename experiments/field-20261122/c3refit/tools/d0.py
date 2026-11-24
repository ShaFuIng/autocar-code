"""diagnosis 0: profiles by distance (0.3 m bins of the continuous distance xc), per run; and the staircase lag of the controller's x."""
import sys, numpy as np, p3
E = np.arange(-0.6, 7.21, 0.3)
for r in sys.argv[1:] or ['6f61', 'c6f3', 'd60e']:
    P = p3.passes([r]); print('====', r, len(P), 'passes')
    lag = np.concatenate([(q['A']['xc'] - q['A']['x'])[(q['A']['xc'] > 0.6) & (q['A']['xc'] < 4.4)] for q in P])
    print('controller x behind the real distance (xc - x), 0.6-4.4 m: median %.3f mean %.3f p90 %.3f max %.3f m' % (np.median(lag), lag.mean(), np.percentile(lag, 90), lag.max()))
    B = lambda k, fn=np.nanmedian: p3.binmed(P, k, E, fn=fn)
    cols = [('v', B('v'), '%.2f'), ('err', B('err'), '%+.2f'), ('errmx', B('err', np.nanmax), '%+.2f'), ('steer', B('steer'), '%+4.0f'), ('stmax', B('steer', np.nanmax), '%+4.0f'), ('sent', B('sent'), '%+4.0f'), ('curv', B('curv'), '%5.1f'), ('gz', B('gz'), '%+5.1f'),
            ('h', B('h'), '%6.1f'), ('y', B('y'), '%+5.1f'), ('psi', B('psi'), '%+5.1f'), ('cin', B('cin'), '%+5.1f'), ('cinmn', B('cin', np.nanmin), '%+5.1f'), ('coutmn', B('cout', np.nanmin), '%+5.1f'), ('cff%', 100 * B('cff_ap', np.nanmean), '%3.0f'),
            ('I', B('I'), '%+3.0f'), ('a3in', B('a3_in'), '%+4.0f'), ('xbtgt', B('xb_tgt'), '%+4.0f'), ('xbprof', B('xb_prof'), '%+4.0f'), ('xb_e', B('xb_e'), '%+4.0f'), ('xbgz', B('xb_gz'), '%+3.0f'), ('own', B('xb_own', np.nanmean), '%.2f'), ('ec', B('xb_ec'), '%.2f')]
    print('  x      ' + ' '.join('%6s' % c[0] for c in cols))
    for k in range(len(E) - 1): print('  %+4.1f  ' % E[k] + ' '.join('%6s' % (c[2] % c[1][k]) for c in cols))
