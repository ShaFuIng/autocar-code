"""land4 per-descent summary. usage (from experiments/): python field-20261122/l4x.py run-xxxx-20261122 [...]"""
import sys, json, numpy as np
sys.path.insert(0, 'field-20261118')
from ld import load

for r in sys.argv[1:]:
    Q = load(r); t0 = Q[0]['t']; t = np.array([p['t'] for p in Q]); gz = np.array([(p.get('inputs') or {}).get('gz') or 0 for p in Q], float)
    yaw = np.concatenate([[0], np.cumsum((gz[1:] + gz[:-1]) / 2 * np.diff(t))])
    st = [p['t'] for i, p in enumerate(Q) if p['_s'] == 'curve1' and (i == 0 or Q[i - 1]['_s'] != 'curve1')]
    print(r, 'dur %.1f laps' % (Q[-1]['t'] - t0), [round(b - a, 1) for a, b in zip(st, st[1:])], 'last', Q[-1]['_s'], Q[-1]['_lap'])
    err = [p['land4'] for p in Q if isinstance(p.get('land4'), dict) and p['land4'].get('reason') in ('ERROR', 'NO_IMU')]; print('  land4 ERROR/NO_IMU', len(err), err[:1])
    for lap in range(1, max(p['_lap'] for p in Q) + 1):
        idx = [i for i, p in enumerate(Q) if p['_lap'] == lap]
        S = [i for i in idx if (Q[i].get('land4') or {}).get('state') == 'SLOPE']; L = [i for i in idx if (Q[i].get('land4') or {}).get('state') == 'LAND']
        dn = [i for i in idx if Q[i]['_s'] == 'downhill']
        if not S or not dn: continue
        l4 = lambda i: Q[i]['land4']
        sent = np.array([(Q[i].get('steer_us') or 1475) - 1475 for i in S]); ap = sum(1 for i in S if l4(i).get('applied')); e = np.array([Q[i].get('image_error') or 0 for i in S])
        val = np.mean([Q[i].get('reason') == 'PROPOSAL_ONLY' for i in S]); d = [l4(i)['psi'] - l4(i)['th'] for i in S if l4(i).get('psi') is not None]
        mouth = Q[max(0, S[0] - 3):S[0] + 8]; ms = [(p.get('steer_us') or 1475) - 1475 for p in mouth]
        psi_land = l4(S[-1]).get('psi'); psi_end = [l4(i).get('psi') for i in S if l4(i).get('u_m') is not None and -0.8 <= l4(i)['u_m'] < -0.6]
        print('  L%d 坡上 %d格 有效 %.0f%% 套用 %d格 ref_n %s glitch %s | 實送−中心 %+4.0f..%+4.0f 最大一步 %3.0f | 坡口±0.3s 實送 %+4.0f..%+4.0f 一步 %3.0f | 誤差 %+.2f..%+.2f | 車頭−目標 %+.1f..%+.1f | 車頭 坡末0.7m前 %s 落地 %s' % (
            lap, len(S), 100 * val, ap, l4(S[-1]).get('ref_n'), l4(S[-1]).get('glitch_deg'), sent.min(), sent.max(), np.abs(np.diff(sent)).max(), min(ms), max(ms), np.abs(np.diff(ms)).max(),
            e.min(), e.max(), min(d) if d else 0, max(d) if d else 0, ('%+.1f' % np.median(psi_end)) if psi_end else '--', ('%+.1f' % psi_land) if psi_land is not None else '--'))
        if L:
            al = lambda i, k: (Q[i].get('arc_lat') or {}).get(k) if (Q[i].get('arc_lat') or {}).get('ok') else np.nan
            ls = np.array([(Q[i].get('steer_us') or 1475) - 1475 for i in L]); join = abs(((Q[L[0]].get('steer_us') or 1475) - (Q[S[-1]].get('steer_us') or 1475)))
            i0 = L[0]; aft = [i for i in idx if i >= i0 and Q[i]['t'] - Q[i0]['t'] <= 3.2]
            g = gz[aft]; ee = [Q[i].get('image_error') or 0 for i in aft]
            co = [al(i, 'clr_out_cm') for i in aft]; ci = [al(i, 'clr_in_cm') for i in aft]
            y45 = [al(i, 'clr_out_cm') for i in aft if -(yaw[i] - yaw[i0]) < 45]
            enc = lambda i: (Q[i].get('inputs') or {}).get('enc_b'); v = np.nan
            if enc(i0) and enc(i0 - 8): v = (enc(i0) - enc(i0 - 8)) * 0.000222 / (t[i0] - t[i0 - 8])
            print('      落地 車速 %.2f 交接跳 %3.0f | 落地後0～1.5m 實送 %+4.0f..%+4.0f 一步 %3.0f | 3.2s內 誤差max %.2f 轉速min %+4.0f max %+4.0f | 外淨空min %+5.1f（前45° %+5.1f）內淨空min %+5.1f' % (
                v, join, ls.min(), ls.max(), np.abs(np.diff(ls)).max(), max(ee), g.min(), g.max(), np.nanmin(co) if np.isfinite(co).any() else np.nan, np.nanmin(y45) if np.isfinite(y45).any() else np.nan, np.nanmin(ci) if np.isfinite(ci).any() else np.nan))
