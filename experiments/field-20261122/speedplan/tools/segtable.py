# -*- coding: utf-8 -*-
"""全圈路段盤點：每輪每圈每路段的長度、時間、車速（MCU 重算）、油門與決定層、餘裕指標。
用法：python segtable.py <主輪> [對照輪 ...]  → 印出表（重導到 proof/T1_segments.txt）。"""
import sys, collections
import numpy as np
from common import *


def src(r):
    if r['reason'] != 'PROPOSAL_ONLY': return 'HOLD:%s' % r['reason']
    if r['sg_dh']: return 'speed_guard'
    if r['tpc'] is not None: return 'upper_cap(turn %s)' % r['tpc']
    if r['bz_bef'] is not None: return '%s(cap %s)' % (r['bz'], r['bz_cap'])
    if r['llp'] is not None and r['pwm'] is not None and abs(float(r['pwm']) - float(r['llp'])) < 0.01: return 'lane_cap %s' % r['llp']
    if r['ls_bef'] is not None: return '%s%s' % (r['ls'], '(taper)' if r['ls_tp'] else '')
    if r['uw']: return 'upper_wide:%s' % r['uw']
    s = 'SA:%s/%s' % (r['sa_l'], r['sa_r'])
    if r['sa_c']: s += '[cpb %s]' % r['sa_c']
    if r['sa_land']: s += '[landing]'
    if r['sa_up'] is not None: s += '[upper %s]' % r['sa_up']
    if r['sa_ulp'] is not None: s += '[uphill_low]'
    if r['sa_crest']: s += '[crest]'
    if r['sa_dhe']: s += '[dh_entry]'
    if r['ls_blk']: s += '{LS %s blk %s}' % (r['ls'], r['ls_blk'])
    return s


def dt_weights(F):
    t = np.array([r['t'] for r in F])
    if len(t) < 2: return np.ones(len(t)) * 0.033
    d = np.diff(t, append=t[-1] + np.median(np.diff(t)))
    return np.clip(d, 0, 0.2)


def seg_stats(R, L):
    out = collections.defaultdict(list)   # seg -> list of per-lap dicts
    for lap in L:
        if lap['lap'] == 0 or lap['t1'] is None: continue
        for sg in SEGS:
            b = seg_bounds(lap, sg)
            if b is None: continue
            o0, o1 = b
            F = frames_in(R, o0, o1)
            if len(F) < 3: continue
            Om_T = lap.get('_omt')
            t0 = t_at(*OMT, o0); t1 = t_at(*OMT, o1)
            w = dt_weights(F)
            v = [r['v'] for r in F if r['v'] is not None]
            pw = [float(r['pwm']) for r in F if r['pwm'] is not None]
            S = collections.Counter()
            for r, wi in zip(F, w): S[src(r)] += wi
            e = [abs(r['e']) for r in F if r['e'] is not None and r['valid']]
            st = np.array([r['st'] for r in F if r['st'] is not None], float)
            wsat = sum(wi for r, wi in zip(F, w) if r['st'] is not None and (r['st'] >= 2040 or r['st'] <= 790))
            wbig = sum(wi for r, wi in zip(F, w) if r['st'] is not None and abs(r['st'] - 1475) >= 425)
            cin = [r['cin'] for r in F if r['cin'] is not None]; cout = [r['cout'] for r in F if r['cout'] is not None]
            gz = [abs(r['gz']) for r in F if r['gz'] is not None]
            e1 = sum(wi for r, wi in zip(F, w) if r['e'] is not None and abs(r['e']) >= 0.99)
            out[sg[0]].append(dict(lap=lap['lap'], L=o1 - o0, T=t1 - t0, v=(o1 - o0) / (t1 - t0) if t1 > t0 else None,
                                   vmin=pct(v, 5), vmax=pct(v, 95), vin=v[0] if v else None, vout=v[-1] if v else None,
                                   pwm=pct(pw, 50), pwm_mean=float(np.average(pw)) if pw else None, src=S,
                                   emax=max(e) if e else None, e1=e1, sat=wsat, big=wbig, cin=min(cin) if cin else None,
                                   cout=min(cout) if cout else None, gz=max(gz) if gz else None))
    return out


def fmt(x, f='%.2f'):
    return '—' if x is None else f % x


if __name__ == '__main__':
    runs = sys.argv[1:]
    POOL = collections.defaultdict(list)
    for run in runs:
        R = load(run); L, OMT = laps(R)
        globals()['OMT'] = OMT
        lt = [l['t1'] - l['t0'] for l in L if l['lap'] > 0 and l['t1'] is not None]
        ll = [l['odo1'] - l['odo0'] for l in L if l['lap'] > 0 and l['t1'] is not None]
        print('=' * 150)
        print('run-%s  完整圈 %d：圈時（C1s→C1s，里程內插）%s  圈長（後輪里程）%s m  平均車速 %s  效能比 %s' % (
            run, len(lt), '／'.join('%.2f' % x for x in lt), '／'.join('%.2f' % x for x in ll),
            '／'.join('%.3f' % (a / b) for a, b in zip(ll, lt)), '／'.join('%.1f%%' % (100 * a / b / VMAX) for a, b in zip(ll, lt))))
        st = seg_stats(R, L)
        print('%-10s %5s %13s %6s %11s %5s %5s %5s | %5s | %-60s | %5s %5s %5s %6s %6s %6s %5s | %5s %5s' % (
            '路段', '長m', '時間 中位(範圍)', 'v均', 'v入/出', 'vP5', 'vP95', 'PWM', '均PWM', '決定層（時間占比前三）', '|e|max', 'e≥.99s', '打滿s', '≥425s', 'in最小', 'out最小', 'gzmax', '+.05', '+.10'))
        for sg in SEGS:
            D = st.get(sg[0])
            if not D: continue
            POOL[sg[0]].extend([dict(d, run=run) for d in D])
            Lm = np.median([d['L'] for d in D]); T = [d['T'] for d in D]; Tm = np.median(T)
            vm = Lm / Tm
            S = collections.Counter()
            for d in D: S.update(d['src'])
            tot = sum(S.values())
            top = '；'.join('%s %.0f%%' % (k, 100 * v / tot) for k, v in S.most_common(3))
            g05 = Lm / vm - Lm / (vm + .05); g10 = Lm / vm - Lm / (vm + .10)
            print('%-10s %5.2f %5.2f(%4.2f-%4.2f) %6.3f %5s/%-5s %5s %5s %5s | %5s | %-60s | %5s %5s %5s %6s %6s %6s %5s | %5.2f %5.2f' % (
                sg[0], Lm, Tm, min(T), max(T), vm, fmt(np.median([d['vin'] for d in D if d['vin'] is not None])), fmt(np.median([d['vout'] for d in D if d['vout'] is not None])),
                fmt(np.median([d['vmin'] for d in D if d['vmin'] is not None])), fmt(np.median([d['vmax'] for d in D if d['vmax'] is not None])),
                fmt(np.median([d['pwm'] for d in D if d['pwm'] is not None]), '%.0f'), fmt(np.median([d['pwm_mean'] for d in D if d['pwm_mean'] is not None]), '%.1f'),
                top[:60], fmt(max([d['emax'] for d in D if d['emax'] is not None] or [None])) if any(d['emax'] for d in D) else '—',
                fmt(np.median([d['e1'] for d in D])), fmt(np.median([d['sat'] for d in D])), fmt(np.median([d['big'] for d in D])),
                fmt(min([d['cin'] for d in D if d['cin'] is not None] or [None]) if any(d['cin'] is not None for d in D) else None, '%.1f'),
                fmt(min([d['cout'] for d in D if d['cout'] is not None] or [None]) if any(d['cout'] is not None for d in D) else None, '%.1f'),
                fmt(max([d['gz'] for d in D if d['gz'] is not None] or [0]), '%.0f'), g05, g10))
            print('    決定層全部：' + '；'.join('%s %.0f%%' % (k, 100 * v / tot) for k, v in S.most_common(8)))
    import pickle
    with open(os.path.join(SCR, 'pool_' + '_'.join(runs) + '.pkl'), 'wb') as f: pickle.dump(dict(POOL), f)
