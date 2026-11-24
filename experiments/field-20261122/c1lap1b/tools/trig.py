"""c1lap1b: trigger / band-onset geometry per curve-1 pass.
Continuous distance s(t): lap_pos odo_m steps (5 Hz) linearly interpolated between the frames where the value changed (each new value
assigned to the frame where it first appears). For lap 1, s is metres from the start (odo 0 = where the car stood); for later laps,
metres since the C4e anchor (since_anchor_m, same interpolation).
Per pass: s at the FAR trigger (s_trig, continuous), near / far / far-near at the trigger and 0.1-0.3 s before it, band onset
(first frame with a >= 0.5, from lap_ff.a), the s where the band output first exceeds centre+150 us, s at tb 10 / 30 deg, y / psi there."""
import json, sys, statistics as st
from pathlib import Path
from lib import load, num
from passes import curve_passes, med
OUT = Path(__file__).resolve().parents[1] / 'proof'


def cont(F, key):
    """continuous version of a stepping odometer field (key 'odo' or 'sa'); None where absent."""
    idx = [i for i, f in enumerate(F) if num(f[key]) and num(f['t'])]
    S = [None] * len(F)
    ch = []   # (t, value) at first appearance of each new value
    last = None
    for i in idx:
        v = F[i][key]
        if last is None or v != last: ch.append((F[i]['t'], v, i)); last = v
    for i in idx:
        t = F[i]['t']
        # find bracketing change points
        k = None
        lo, hi = 0, len(ch) - 1
        while lo <= hi:
            m = (lo + hi) // 2
            if ch[m][0] <= t: k = m; lo = m + 1
            else: hi = m - 1
        if k is None: S[i] = F[i][key]; continue
        if k + 1 < len(ch) and ch[k + 1][1] > ch[k][1] and ch[k + 1][1] - ch[k][1] < 0.6:
            t0, v0, _ = ch[k]; t1, v1, _ = ch[k + 1]
            S[i] = v0 + (v1 - v0) * (t - t0) / max(1e-6, t1 - t0)
        else:
            S[i] = ch[k][1]
    return S


def analyse(run):
    F = load(run); P = curve_passes(F)
    so = cont(F, 'odo')
    out = []
    for p in P:
        if not p['band']: continue
        q0 = p['band'][0]; f0 = F[q0]
        b = [q for q in p['band'] if num(F[q]['tb'])]
        plat = med([F[q]['hd'] + F[q]['tb'] for q in b])
        r = dict(run=run, first=p['first'], lap=f0['lap'], s_trig=so[q0], odo_trig=f0['odo'], anchor=f0['anchor'], sa_trig=f0['sa'])
        if not p['first'] and num(f0['sa']) and num(so[q0]) and num(f0['odo']):
            r['sa_trig_c'] = f0['sa'] + (so[q0] - f0['odo'])   # continuous since-anchor
        r['near_trig'] = f0['near']; r['far_trig'] = f0['far']
        pre = [F[q] for q in range(max(0, q0 - 12), q0) if num(F[q]['t']) and 0.1 <= f0['t'] - F[q]['t'] <= 0.35]
        r['near_pre'] = med([f['near'] for f in pre]); r['far_pre'] = med([f['far'] for f in pre])
        r['fmn_trig'] = None if not (num(f0['far']) and num(f0['near'])) else f0['far'] - f0['near']
        r['y_pre'] = med([f['y'] for f in pre]); r['psi_pre'] = med([f['psi'] for f in pre])
        r['hd_rel_trig'] = plat - f0['hd']
        # marker: first frame (going forward from 1.2 s before the trigger) where far - near >= 0.28
        mk = None
        for q in range(max(0, q0 - 40), min(len(F), q0 + 30)):
            f = F[q]
            if num(f['far']) and num(f['near']) and f['far'] - f['near'] >= 0.28 and num(so[q]): mk = q; break
        r['s_mark'] = None if mk is None else so[mk]
        r['trig_minus_mark'] = None if mk is None or not num(so[q0]) else so[q0] - so[mk]
        # curvature marker (camera arc_fit kappa_dpm, geometric): s where kappa first <= -10 and <= -20 deg/m (two consecutive frames), mean of both
        def kmark(th):
            for q in range(max(0, q0 - 45), min(len(F), q0 + 20)):
                a = F[q]['af_k']; b_ = F[q + 1]['af_k'] if q + 1 < len(F) else None
                if num(a) and num(b_) and a <= th and b_ <= th and num(so[q]): return so[q]
            return None
        k10 = kmark(-10.); k20 = kmark(-20.)
        km = None if (k10 is None or k20 is None) else 0.5 * (k10 + k20)
        r['k10_rel_trig'] = None if k10 is None else k10 - so[q0]; r['k20_rel_trig'] = None if k20 is None else k20 - so[q0]
        r['trig_minus_kmark'] = None if km is None else so[q0] - km
        on = next((q for q in p['band'] if num(F[q]['lf_a']) and F[q]['lf_a'] >= 0.5), None)
        r['onset_minus_kmark'] = None if (on is None or km is None) else so[on] - km
        r['onset_minus_trig'] = None if on is None or not num(so[on]) or not num(so[q0]) else so[on] - so[q0]
        r['onset_dt'] = None if on is None else F[on]['t'] - f0['t']
        r['onset_minus_mark'] = None if on is None or mk is None else so[on] - so[mk]
        q10 = next((q for q in b if F[q]['tb'] >= 10), None); q30 = next((q for q in b if F[q]['tb'] >= 30), None)
        r['tb10_minus_mark'] = None if q10 is None or mk is None else so[q10] - so[mk]
        r['tb30_minus_mark'] = None if q30 is None or mk is None else so[q30] - so[mk]
        r['tb10_minus_kmark'] = None if q10 is None or km is None else so[q10] - km
        r['tb30_minus_kmark'] = None if q30 is None or km is None else so[q30] - km
        r['tb10_minus_trig'] = None if q10 is None else so[q10] - so[q0]
        r['y_at_onset'] = None if on is None else F[on]['y']
        cur = [F[q] for q in p['band']]
        ys = [f['y'] for f in cur if num(f['y'])]; r['y_min'] = min(ys) if ys else None
        ci = [f['clr_in'] for f in cur if num(f['clr_in'])]; r['clr_in_min'] = min(ci) if ci else None
        co = [f['clr_out'] for f in cur if num(f['clr_out'])]; r['clr_out_min'] = min(co) if co else None
        out.append(r)
    return out


def corr(x, y):
    import math
    p = [(a, b) for a, b in zip(x, y) if num(a) and num(b)]
    if len(p) < 3: return None, None, len(p)
    mx = sum(a for a, _ in p) / len(p); my = sum(b for _, b in p) / len(p)
    sxx = sum((a - mx) ** 2 for a, _ in p); syy = sum((b - my) ** 2 for _, b in p); sxy = sum((a - mx) * (b - my) for a, b in p)
    return (sxy / math.sqrt(sxx * syy) if sxx > 0 and syy > 0 else None), (sxy / sxx if sxx > 0 else None), len(p)


if __name__ == '__main__':
    runs = ['3f22', 'fd22', 'b23a', '2b72', 'b619', '1377', 'e69d', '54cb', '1cd7', '9af6']
    R = []
    for run in runs: R += analyse(run)
    cols = ['run', 'first', 'lap', 's_trig', 'sa_trig_c', 'near_pre', 'near_trig', 'far_trig', 'fmn_trig', 'y_pre', 'psi_pre', 'hd_rel_trig', 'trig_minus_mark', 'k10_rel_trig', 'k20_rel_trig', 'trig_minus_kmark', 'onset_minus_kmark', 'tb10_minus_kmark', 'tb30_minus_kmark', 'onset_minus_trig', 'onset_dt',
            'onset_minus_mark', 'tb10_minus_trig', 'tb10_minus_mark', 'tb30_minus_mark', 'y_at_onset', 'y_min', 'clr_in_min', 'clr_out_min']
    L = ['\t'.join(cols)]
    for r in R: L.append('\t'.join(str(round(r.get(c), 2)) if isinstance(r.get(c), float) else str(r.get(c)) for c in cols))
    for g, G in (('lap1', [r for r in R if r['first']]), ('later', [r for r in R if not r['first']]), ('all', R)):
        L.append(f'== {g} n={len(G)}')
        for c in cols[3:]:
            v = [r.get(c) for r in G if num(r.get(c))]
            if v: L.append(f'  {c}: median {st.median(v):.3f} min {min(v):.3f} max {max(v):.3f}')
        for x in ('trig_minus_kmark', 'onset_minus_kmark', 'tb10_minus_kmark', 'tb30_minus_kmark', 's_trig', 'near_pre', 'y_pre', 'psi_pre', 'hd_rel_trig', 'trig_minus_mark', 'onset_minus_trig', 'onset_minus_mark', 'tb10_minus_mark', 'tb10_minus_trig', 'fmn_trig', 'y_at_onset'):
            rr, sl, n = corr([r.get(x) for r in G], [r.get('y_min') for r in G])
            L.append(f'  y_min vs {x}: r={None if rr is None else round(rr, 2)} slope={None if sl is None else round(sl, 2)} n={n}')
        for x, y in (('near_pre', 'trig_minus_kmark'), ('y_pre', 'trig_minus_kmark'), ('psi_pre', 'trig_minus_kmark'), ('near_pre', 'trig_minus_mark'), ('y_pre', 'near_pre'), ('near_pre', 's_trig'), ('y_pre', 'trig_minus_mark')):
            rr, sl, n = corr([r.get(x) for r in G], [r.get(y) for r in G])
            L.append(f'  {y} vs {x}: r={None if rr is None else round(rr, 2)} slope={None if sl is None else round(sl, 3)} n={n}')
    (OUT / 'B1_trigger_geometry.txt').write_text('\n'.join(L), encoding='utf8')
    print('\n'.join(L))
