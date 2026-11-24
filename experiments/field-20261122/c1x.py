"""short-start summary: start drift, curve 1, after the exit. usage (from experiments/): python field-20261122/c1x.py "label=run run ..." ..."""
import sys, numpy as np
sys.path.insert(0, 'field-20261118')
from ld import load


def one(r):
    Q = load('run-%s-20261122' % r); t0 = Q[0]['t']; t = np.array([p['t'] - t0 for p in Q]); gz = np.array([(p.get('inputs') or {}).get('gz') or 0 for p in Q], float)
    yaw = np.concatenate([[0], np.cumsum((gz[1:] + gz[:-1]) / 2 * np.diff(t))])
    c = [i for i, p in enumerate(Q) if p['_lap'] == 1 and p['_s'] == 'curve1']; s = c[0]; e = c[-1]
    w = [i for i in range(len(Q)) if 0.3 <= t[i] <= 3.0]; ew = [Q[i].get('image_error') or 0 for i in w]
    sb = [Q[i]['start_bias'].get('bias_us') for i in range(len(Q)) if isinstance(Q[i].get('start_bias'), dict) and Q[i]['start_bias'].get('bias_us') is not None]
    al = lambda i, k: (Q[i].get('arc_lat') or {}).get(k) if (Q[i].get('arc_lat') or {}).get('ok') else np.nan
    A = np.array([(-(yaw[i] - yaw[s]), Q[i].get('image_error') or 0, al(i, 'y_cm'), al(i, 'clr_in_cm'), (Q[i].get('steer_us') or 0) - 1475, gz[i]) for i in c], float)
    b = lambda lo, hi: np.nanmedian(A[(A[:, 0] >= lo) & (A[:, 0] < hi), 2]) if ((A[:, 0] >= lo) & (A[:, 0] < hi)).sum() > 2 else np.nan
    ex = []
    for lo, hi in ((0, .4), (.4, .8), (.8, 1.2), (1.2, 1.6), (1.6, 2.2), (2.2, 3.0)):
        m = [i for i in range(e + 1, len(Q)) if lo <= t[i] - t[e] < hi]
        ex.append((np.median([Q[i].get('image_error') or 0 for i in m]), np.median([(Q[i].get('steer_us') or 0) - 1475 for i in m]), np.median([Q[i].get('i') or 0 for i in m]), yaw[m[-1]] - yaw[e]) if m else None)
    pr = np.array([(p.get('imu_boost') or {}).get('pitch_rel_deg') or 0 for p in Q[45:]])
    v = np.nan
    en = [(t[i], (Q[i].get('inputs') or {}).get('enc_b')) for i in c if (Q[i].get('inputs') or {}).get('enc_b')]
    if len(en) > 3: v = (en[-1][1] - en[0][1]) * 0.000222 / (en[-1][0] - en[0][0])
    print(' %s 起步: 左舵 %s 車頭 最右 %+5.1f 最左 %+5.1f 誤差 最低 %+.2f 最高 %+.2f | 彎1 y %s emax %.2f 舵max %+4.0f 轉速min %+4.0f 內淨空 %+5.1f 車速 %.2f | 俯仰中位 %+.2f' % (
        r, min(sb) if sb else None, min(yaw[w]), max(yaw[w]), min(ew), max(ew), ' '.join('%+5.1f' % b(lo, lo + 30) for lo in (0, 30, 60, 90, 120)), A[:, 1].max(), A[:, 4].max(), A[:, 5].min(), np.nanmin(A[:, 3]), v, np.median(pr)))
    print('      出彎後 誤差: %s | 舵: %s | 積分: %s | 車頭: %s' % (' '.join('%+.2f' % x[0] if x else ' -- ' for x in ex), ' '.join('%+4.0f' % x[1] if x else ' -- ' for x in ex),
                                                    ' '.join('%+3.0f' % x[2] if x else ' -- ' for x in ex), ' '.join('%+5.1f' % x[3] if x else ' -- ' for x in ex)))


for a in sys.argv[1:]:
    g, runs = a.split('=', 1); print(g)
    for r in runs.split(): one(r)
