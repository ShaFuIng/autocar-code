"""curve 2 by turned angle + curve 4 by distance after LAND8. usage (from experiments/): python field-20261122/c2c4.py "label=run-a run-b" ..."""
import sys, collections, numpy as np
sys.path.insert(0, 'field-20261118')
from ld import load


def c2(runs, label):
    rows = collections.OrderedDict(); per = []
    for r in runs:
        Q = load(r); t = np.array([p['t'] for p in Q]); gz = np.array([(p.get('inputs') or {}).get('gz') or 0 for p in Q], float)
        yaw = np.concatenate([[0], np.cumsum((gz[1:] + gz[:-1]) / 2 * np.diff(t))])
        for lap in range(1, max(p['_lap'] for p in Q) + 1):
            c = [i for i, p in enumerate(Q) if p['_lap'] == lap and p['_s'] == 'curve2']
            if len(c) < 40: continue
            s = c[0]; al = lambda i, k: (Q[i].get('arc_lat') or {}).get(k) if (Q[i].get('arc_lat') or {}).get('ok') else np.nan
            for i in c:
                p = Q[i]; tb = -(yaw[i] - yaw[s]); k = int(tb // 30) * 30; lf = p.get('lap_ff') or {}; cf = p.get('curve_ff') or {}
                rows.setdefault(k, []).append((p.get('image_error') or 0, (lf.get('target_us') or np.nan) - 1475 if lf.get('target_us') else np.nan, (p.get('steer_us') or 1475) - 1475, gz[i],
                                               al(i, 'y_cm'), al(i, 'clr_in_cm'), al(i, 'clr_out_cm'), 1 if cf.get('applied') else 0))
            E = [Q[i].get('image_error') or 0 for i in c]; S = [(Q[i].get('steer_us') or 1475) - 1475 for i in c]
            ex = [Q[i].get('image_error') or 0 for i in range(c[-1] + 1, min(len(Q), c[-1] + 60))]
            per.append((max(E), max(S), min(gz[c]), np.nanmin([al(i, 'clr_in_cm') for i in c]), np.nanmin([al(i, 'clr_out_cm') for i in c]), t[c[-1]] - t[s], min(ex), max(ex)))
    P = np.array(per)
    print(label, '%d圈' % len(P)); print('   逐圈 誤差峰', ' '.join('%.2f' % x for x in P[:, 0]), '| 舵峰', ' '.join('%+.0f' % x for x in P[:, 1]), '| 轉速min', ' '.join('%+.0f' % x for x in P[:, 2]))
    print('   逐圈 內淨空min', ' '.join('%+.1f' % x for x in P[:, 3]), '| 外淨空min', ' '.join('%+.1f' % x for x in P[:, 4]), '| 時間中位 %.2f | 出彎後誤差 min' % np.median(P[:, 5]), ' '.join('%+.2f' % x for x in P[:, 6]), 'max', ' '.join('%+.2f' % x for x in P[:, 7]))
    for k in sorted(rows):
        v = np.array(rows[k], float)
        if len(v) < 8 or k < 0: continue
        f = lambda j: np.nanmedian(v[:, j]) if np.isfinite(v[:, j]).any() else float('nan')
        print('   %3d-%3d° 誤差 %+.2f 帶狀目標 %+5.0f 實送 %+5.0f 轉速 %+4.0f 位置 %+5.1f 內 %+5.1f 外 %+5.1f 前饋下限 %3.0f%%' % (k, k + 30, f(0), f(1), f(2), f(3), f(4), f(5), f(6), 100 * np.nanmean(v[:, 7])))


def c4(runs, label):
    rows = collections.OrderedDict(); T = []
    for r in runs:
        Q = load(r); gz = np.array([(p.get('inputs') or {}).get('gz') or 0 for p in Q], float)
        for lap in range(1, max(p['_lap'] for p in Q) + 1):
            c = [p for p in Q if p['_lap'] == lap and p['_s'] == 'curve4']
            if len(c) > 40: T.append(c[-1]['t'] - c[0]['t'])
            i0 = next((i for i, p in enumerate(Q) if p['_lap'] == lap and (p.get('lap_pos') or {}).get('anchor') == 'LAND8'), None)
            if i0 is None: continue
            e0 = None
            for i in range(i0, min(len(Q), i0 + 320)):
                p = Q[i]; en = (p.get('inputs') or {}).get('enc_b')
                if en is None: continue
                if e0 is None: e0 = en - (p['lap_pos'].get('since_anchor_m') or 0) / 0.000222
                x = (en - e0) * 0.000222
                if x > 9: break
                a = p.get('arc_lat') or {}
                rows.setdefault(int(x / 0.75), []).append((a.get('v') if a.get('ok') else np.nan, p.get('pwm') or 0, p.get('image_error') or 0, (p.get('steer_us') or 1475) - 1475, gz[i],
                                                           a.get('clr_in_cm') if a.get('ok') else np.nan, a.get('clr_out_cm') if a.get('ok') else np.nan))
    print(label, '彎4 時間', ' '.join('%.2f' % x for x in T), '中位 %.2f' % np.median(T))
    for k, v in rows.items():
        v = np.array(v, float); f = lambda j: np.nanmedian(v[:, j]) if np.isfinite(v[:, j]).any() else float('nan'); mn = lambda j: np.nanmin(v[:, j]) if np.isfinite(v[:, j]).any() else float('nan')
        print('   x %4.2f-%4.2f 車速 %.2f 油門 %2.0f 誤差 %+.2f(max %+.2f) 實送 %+5.0f 轉速 %+4.0f 內 %+5.1f(min %+5.1f) 外 %+5.1f(min %+5.1f)' % (k * 0.75, k * 0.75 + 0.75, f(0), f(1), f(2), v[:, 2].max(), f(3), f(4), f(5), mn(5), f(6), mn(6)))


for a in sys.argv[1:]:
    label, runs = a.split('=', 1); runs = runs.split(); c2(runs, label); c4(runs, label)
