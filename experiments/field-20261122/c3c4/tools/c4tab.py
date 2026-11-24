"""curve 4: profiles over x (lap_ff4 metres since the predicted LAND8 point), over xe (main-dialogue x: LAND8 + encoder) and over the band's turned angle tb."""
import sys, numpy as np, lib
from c4pass import passes

P = passes()


def good(q):
    A = q['A']; m = (A['xe'] > 1.5) & (A['xe'] < 9)
    return np.nanmin(A['cin'][m]) >= -7. and np.nanmin(A['cout'][m]) >= 3. and np.nanmax(A['err'][m]) <= 0.62
G = [q for q in P if good(q)]
print('passes', len(P), 'good', len(G), [(q['run'], q['lap']) for q in G])


def tab(S, key, edges, label):
    print('--', label, 'n passes', len(S))
    print('  bin        n  kappa_act(med/p25/p75) kpath  err(med p25 p75)  steer   pid   prof  btgt  btop   ec   e_us   kapB   y    cin(med/min) cout(med/min)  v   pwm  I')
    for a, b in zip(edges, edges[1:]):
        R = {k: [] for k in ('kap', 'kpath', 'err', 'steer', 'pid', 'prof', 'btgt', 'btop', 'ec', 'eus', 'kapB', 'y', 'cin', 'cout', 'v', 'pwm', 'I')}; pk = []
        for q in S:
            A = q['A']; m = (A[key] >= a) & (A[key] < b)
            if not m.any(): continue
            vv = np.where(A['v'] > 0.3, A['v'], np.nan)
            ka = -A['gz'][m] / vv[m]; pk.append(np.nanmedian(ka))
            for k, arr in (('kap', ka), ('kpath', -A['kpath'][m]), ('err', A['err'][m]), ('steer', A['steer'][m]), ('pid', A['pid'][m]), ('prof', A['prof'][m]), ('btgt', A['btgt'][m]), ('btop', A['btop'][m]),
                           ('ec', A['ec'][m]), ('eus', A['eus'][m]), ('kapB', A['kap'][m]), ('y', A['y'][m]), ('cin', A['cin'][m]), ('cout', A['cout'][m]), ('v', vv[m]), ('pwm', A['pwm'][m]), ('I', A['I'][m])): R[k].extend(arr.tolist())
        f = lambda k, fn=np.nanmedian: (fn(np.array(R[k], float)) if np.isfinite(np.array(R[k], float)).any() else float('nan'))
        pc = lambda k, p: np.nanpercentile(np.array(R[k], float), p) if np.isfinite(np.array(R[k], float)).any() else float('nan')
        if not R['err']: continue
        print('  %5.2f-%5.2f %4d  %5.1f/%5.1f/%5.1f  %5.1f  %+.2f %+.2f %+.2f  %+5.0f %+5.0f %+5.0f %+5.0f %+5.0f  %.2f %+5.0f  %5.1f %+5.1f  %+5.1f/%+5.1f %+5.1f/%+5.1f  %.2f %3.0f %+4.0f' % (
            a, b, len(R['err']), np.nanmedian(pk), np.nanpercentile(pk, 25), np.nanpercentile(pk, 75), f('kpath'), f('err'), pc('err', 25), pc('err', 75), f('steer'), f('pid'), f('prof'), f('btgt'), f('btop'), f('ec'), f('eus'), f('kapB'),
            f('y'), f('cin'), f('cin', np.nanmin), f('cout'), f('cout', np.nanmin), f('v'), f('pwm'), f('I')))


tab(P, 'xe', np.arange(0, 9.76, 0.75), 'ALL by xe 0.75 m (main-dialogue x)')
tab(P, 'x4', np.arange(0, 9.6, 0.3), 'ALL by lap_ff4 x 0.3 m')
tab(G, 'x4', np.arange(0, 9.6, 0.3), 'GOOD by lap_ff4 x 0.3 m')
tab(P, 'tb', np.arange(0, 211, 10), 'ALL by band turned angle 10 deg')
tab(G, 'tb', np.arange(0, 211, 10), 'GOOD by band turned angle 10 deg')
