"""landing3-20261121: profile by distance after LAND8 (13 landings, mouth cap 34): PWM and its source, error, yaw rate, clearance.
usage: python profile.py"""
import sys, collections, numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from data import load, landings, M

RUNS = ('run-e1b4-20261121', 'run-b93e-20261121', 'run-9934-20261121')
B = collections.defaultdict(list)
for run in RUNS:
    Q = load(run)
    for k in landings(Q):
        enc0 = Q[k]['enc']; s0 = Q[k]['s'] or 0.
        end = next((i for i in range(k, len(Q)) if Q[i]['anc'] != 'LAND8'), len(Q) - 1)
        if end - k < 60: continue
        for i in range(k, end):
            if Q[i]['enc'] is None: continue
            x = s0 + (Q[i]['enc'] - enc0) * M
            if x > 5.: break
            src = 'brake31' if Q[i]['bz_cap'] == 31.0 and (Q[i]['pwm'] or 0) <= 31.05 else 'landing' if Q[i]['landing'] else 'c4 by-curve' if Q[i]['curve_by'] == 'c4' else 'other'
            B[int(x / .5)].append((Q[i]['pwm'] or 0, src, Q[i]['e'] or 0, Q[i]['gz'] or 0, Q[i]['clr_out'], Q[i]['clr_in'], Q[i]['sa_reason'], Q[i]['sa_level'], Q[i]['raw'] or 0, Q[i]['bo_phase'], Q[i]['owner']))
print('x after LAND8 | frames | PWM mean (min..max) | PWM source | speed_adapt level | |e| mean / P90 | gz mean / sd | cmd - 1475 mean / max | outer clr median / min | inner clr median / min | bottom_owner phase | owner')
for b in sorted(B):
    X = B[b]; pw = np.array([x[0] for x in X]); e = np.array([abs(x[2]) for x in X]); gz = np.array([x[3] for x in X]); cmd = np.array([x[8] for x in X]) - 1475.
    co = np.array([x[4] for x in X if x[4] is not None]); ci = np.array([x[5] for x in X if x[5] is not None])
    pc = lambda j: ' '.join('%s %.0f%%' % (k, 100. * n / len(X)) for k, n in collections.Counter(x[j] for x in X).most_common(3))
    print(' %.1f-%.1f | %3d | %.0f (%.0f..%.0f) | %s | %s | %.2f / %.2f | %+.0f / %.0f | %+.0f / %+.0f | %s | %s | %s | %s' % (
        b * .5, b * .5 + .5, len(X), pw.mean(), pw.min(), pw.max(), pc(1), pc(7), e.mean(), np.percentile(e, 90), gz.mean(), gz.std(), cmd.mean(), cmd.max(),
        ('%+.1f / %+.1f (n %d)' % (np.median(co), co.min(), len(co))) if len(co) else '-', ('%+.1f / %+.1f (n %d)' % (np.median(ci), ci.min(), len(ci))) if len(ci) else '-', pc(9), pc(10)))
