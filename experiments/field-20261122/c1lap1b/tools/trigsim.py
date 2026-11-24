"""c1lap1b: offline evaluation of alternative curve-1 FAR trigger rules on the recorded approach frames (open loop: valid only while the
new trigger fires before the recorded band onset, i.e. while the recorded far / near are still free of the band's steering).
Rule: fire on the first valid frame (armed window as recorded: from 1.2 s before the recorded trigger) with
      far - k * max(0, near - n0) >= far_on   and near >= -0.05.
Measures s_fire - kmark (camera curvature marker, metres) per pass; sd and lap-1 / later medians; shift vs the recorded trigger.
Extra lap-1 delay d1 is added to the s of the band onset model: onset = s_fire + 0.30 + (odo-step lag, as recorded b) + d1 (lap 1 only)."""
import statistics as st, sys, json
from pathlib import Path
from lib import load, num
from passes import curve_passes
from trig import cont
OUT = Path(__file__).resolve().parents[1] / 'proof'
RUNS = ['3f22', 'fd22', 'b23a', '2b72', 'b619', '1377', 'e69d', '54cb', '1cd7', '9af6']


def kmark(F, so, q0):
    def km(th):
        for q in range(max(0, q0 - 45), min(len(F) - 1, q0 + 20)):
            a = F[q]['af_k']; b = F[q + 1]['af_k']
            if num(a) and num(b) and a <= th and b <= th and num(so[q]): return so[q]
        return None
    a, b = km(-10.), km(-20.)
    return None if a is None or b is None else 0.5 * (a + b)


def prep():
    D = []
    for run in RUNS:
        F = load(run); so = cont(F, 'odo')
        for p in curve_passes(F):
            if not p['band']: continue
            q0 = p['band'][0]
            on = next((q for q in p['band'] if num(F[q]['lf_a']) and F[q]['lf_a'] > 0), None)
            k = kmark(F, so, q0)
            # armed window as the controller: lap 1 = odo >= start.gap_m + win_m[0] = 2.0; later = since C4e >= gap_m + win_m[0] = 8.94
            arm = 2.0 if p['first'] else 8.94
            sa0 = (F[q0]['sa'] - (F[q0]['odo'] - so[q0])) if (not p['first'] and num(F[q0]['sa'])) else None   # continuous since-anchor at the trigger
            def armed(q):
                if p['first']: return so[q] >= arm
                return sa0 is not None and sa0 + (so[q] - so[q0]) >= arm
            fr = [(so[q], F[q]['far'], F[q]['near'], F[q]['valid'], F[q]['t']) for q in range(max(0, q0 - 40), on if on is not None else q0 + 10) if num(so[q]) and armed(q)]
            ys = [F[q]['y'] for q in p['band'] if num(F[q]['y'])]
            D.append(dict(run=run, first=p['first'], s_rec=so[q0], k=k, s_on=so[on] if on is not None else None, frames=fr, y_min=min(ys) if ys else None))
    return D


def fire(d, k, n0, far_on=0.4):
    for s, far, near, valid, t in d['frames']:
        if not valid or not num(far) or not num(near): continue
        if near < -0.05: continue
        if far - k * max(0., near - n0) >= far_on: return s
    return None


if __name__ == '__main__':
    D = prep()
    L = ['k\tn0\tfar_on\tn_fired\tsd(all)\tmed_lap1\tmed_later\tmean_shift_lap1\tmean_shift_later\tmax_shift\tworst_lap1(min trig-kmark)']
    best = []
    for k in (0.0, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0):
        for n0 in (0.0, 0.03, 0.05):
            for fo in (0.4,):
                rel = []; sh1 = []; sh2 = []; miss = 0; mx = 0
                for d in D:
                    s = fire(d, k, n0, fo)
                    if s is None or d['k'] is None: miss += 1; continue
                    rel.append((d['first'], s - d['k'])); sh = s - d['s_rec']; mx = max(mx, sh)
                    (sh1 if d['first'] else sh2).append(sh)
                a = [r for _, r in rel]
                if not a or not sh1 or not sh2: L.append(f'{k}	{n0}	{fo}	missing {miss}'); continue
                l1 = [r for f, r in rel if f]; l2 = [r for f, r in rel if not f]
                row = [k, n0, fo, len(a), round(st.pstdev(a), 3), round(st.median(l1), 3), round(st.median(l2), 3), round(sum(sh1) / len(sh1), 3), round(sum(sh2) / len(sh2), 3), round(mx, 3), round(min(l1), 3)]
                L.append('\t'.join(str(x) for x in row)); best.append(row)
    L.append('')
    L.append('per pass (k=1.0, n0=0.03): run first s_rec-kmark  s_new-kmark  shift  y_min   (recorded onset - recorded trigger)')
    for d in D:
        s = fire(d, 1.0, 0.03)
        L.append('  %s %s %.3f %s %s %.1f %.3f' % (d['run'], d['first'], d['s_rec'] - d['k'], None if s is None else round(s - d['k'], 3), None if s is None else round(s - d['s_rec'], 3), d['y_min'],
                                                   (d['s_on'] - d['s_rec']) if d['s_on'] is not None else float('nan')))
    (OUT / 'C1_trigger_rule_sim.txt').write_text('\n'.join(L), encoding='utf8')
    print('\n'.join(L))
