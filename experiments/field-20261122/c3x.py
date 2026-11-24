"""curve 3 by distance after C3s. usage (from experiments/): python field-20261122/c3x.py "label=run-a run-b" ..."""
import sys, collections, numpy as np
sys.path.insert(0, 'field-20261118')
from ld import load

for a in sys.argv[1:]:
    label, runs = a.split('=', 1); rows = collections.OrderedDict(); T = []; per = []
    for r in runs.split():
        Q = load(r); gz = np.array([(p.get('inputs') or {}).get('gz') or 0 for p in Q], float)
        for lap in range(1, max(p['_lap'] for p in Q) + 1):
            c = [p for p in Q if p['_lap'] == lap and p['_s'] == 'curve3']
            if len(c) > 60: T.append(c[-1]['t'] - c[0]['t'])
            pl = []
            for i, p in enumerate(Q):
                lp = p.get('lap_pos') or {}
                if p['_lap'] != lap or lp.get('anchor') != 'C3s' or lp.get('since_anchor_m') is None or lp['since_anchor_m'] > 7.4: continue
                x = lp['since_anchor_m']; al = p.get('arc_lat') or {}; cf = p.get('curve_ff') or {}; ct = p.get('curve_trim') or {}
                row = (al.get('v') if al.get('ok') else np.nan, p.get('pwm') or 0, p.get('image_error') or 0, (p.get('steer_us') or 1475) - 1475, gz[i], al.get('y_cm') if al.get('ok') else np.nan,
                       al.get('clr_in_cm') if al.get('ok') else np.nan, al.get('clr_out_cm') if al.get('ok') else np.nan, 1 if cf.get('applied') else 0, ct.get('trim_us') or 0, x)
                rows.setdefault(int(x / 0.6), []).append(row); pl.append(row)
            if len(pl) > 60:
                P = np.array(pl, float); m = lambda lo, hi, j, f: (f(P[(P[:, 10] >= lo) & (P[:, 10] < hi), j]) if ((P[:, 10] >= lo) & (P[:, 10] < hi)).any() else np.nan)
                per.append((r[4:8], lap, m(0.6, 4.2, 6, np.nanmin), m(4.2, 5.6, 7, np.nanmin), m(4.2, 5.6, 2, np.max), m(4.2, 5.6, 3, np.max), m(4.2, 5.6, 4, np.min), m(5.6, 7.2, 2, np.max), m(5.6, 7.2, 3, np.max), m(5.6, 7.2, 6, np.nanmin)))
    print(label, '彎3 時間', ' '.join('%.2f' % x for x in T), '中位 %.2f' % np.median(T))
    print('  逐圈: 前段內淨空min | 拐角 外淨空min 誤差max 舵max 轉速min | 尾段 誤差max 舵max 內淨空min')
    for x in per: print('   %s L%d  %+5.1f | %+5.1f %.2f %+4.0f %+4.0f | %.2f %+4.0f %+5.1f' % x)
    print('  C3s後 | 車速 油門 | 誤差(中位/max) | 實送(中位/max) | 轉速 | 位置 | 內淨空(中位/min) | 外淨空(中位/min) | 前饋下限% | 減舵量')
    for k, v in rows.items():
        M = np.array(v, float); f = lambda j: np.nanmedian(M[:, j]) if np.isfinite(M[:, j]).any() else float('nan'); mn = lambda j: np.nanmin(M[:, j]) if np.isfinite(M[:, j]).any() else float('nan')
        print('   %.1f-%.1f  %.2f %2.0f | %+.2f/%+.2f | %+5.0f/%+5.0f | %+4.0f | %+5.1f | %+5.1f/%+5.1f | %+5.1f/%+5.1f | %3.0f%% | %2.0f' % (
            k * 0.6, k * 0.6 + 0.6, f(0), f(1), f(2), M[:, 2].max(), f(3), M[:, 3].max(), f(4), f(5), f(6), mn(6), f(7), mn(7), 100 * M[:, 8].mean(), f(9)))
