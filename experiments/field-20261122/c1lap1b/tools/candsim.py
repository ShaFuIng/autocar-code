"""c1lap1b: replay of the candidate trigger / onset rules (lap_ff.py c1lap1b-r1 keys) on the recorded frames, all band passes.
Same open-loop rule as armsim.py (lap_ff sees the previous frame's lap_pos odometer): trigger = FAR (far - near_k*max(0, near - n0) >= 0.4,
near >= -0.05) on an armed frame; onset (a >= 0.5) = first frame with x + ext - delay >= 0.1 (ramp 0.2), x = seen odometer - d0;
ext (x_ext key) = V_EXT * time since the seen odometer value last changed, capped 0.3 m (V_EXT = band v_ref 1.10).
Reports onset - kmark (camera curvature marker) per group, its spread, and the estimate clr_in = recorded + SLOPE * (onset shift)."""
import statistics as st
from pathlib import Path
from lib import load, num
from passes import curve_passes
from trig import cont
from trigsim import kmark
OUT = Path(__file__).resolve().parents[1] / 'proof'
RUNS = ['3f22', 'fd22', 'b23a', '2b72', 'b619', '1377', 'e69d', '54cb', '1cd7', '9af6']
SLOPE = 25.17; V_EXT = 1.10


def prep():
    P = []
    for run in RUNS:
        F = load(run); so = cont(F, 'odo')
        # seen odometer (previous frame) and the time it last changed
        seen = [None] + [F[q - 1]['odo'] for q in range(1, len(F))]
        tch = [None] * len(F); last = None; tl = None
        for q in range(len(F)):
            if seen[q] != last: last = seen[q]; tl = F[q]['t']
            tch[q] = tl
        for p in curve_passes(F):
            if not p['band']: continue
            q0 = p['band'][0]; k = kmark(F, so, q0)
            on = next(q for q in p['band'] if num(F[q]['lf_a']) and F[q]['lf_a'] >= 0.5)
            ci = min((F[q]['clr_in'] for q in p['band'] if num(F[q]['clr_in'])), default=None)
            # arming: lap 1 = seen odo - 3.2 >= -1.2; later = seen since-anchor (previous frame) - 10.74 >= -1.8
            P.append(dict(run=run, first=p['first'], F=F, so=so, seen=seen, tch=tch, q0=q0, on=on, k=k, ci=ci))
    return P


def armed(d, q):
    F = d['F']
    if d['first']: return num(d['seen'][q]) and d['seen'][q] - 3.2 >= -1.2
    sa = F[q - 1]['sa']; return num(sa) and F[q - 1]['anchor'] == 'C4e' and sa - 10.74 >= -1.8


def sim(d, near_k=None, n0=0.03, delay=0.30, x_ext=False, lap1_only=True):
    F = d['F']; nk = near_k if (d['first'] or not lap1_only) else None
    dl = delay if (d['first'] or not lap1_only) else 0.30
    xe = x_ext
    qt = None
    for q in range(max(1, d['q0'] - 40), min(len(F), d['q0'] + 40)):
        f = F[q]
        if not armed(d, q): continue
        if not f['valid'] or not num(f['far']) or not num(f['near']) or f['near'] < -0.05: continue
        if f['far'] - (nk * max(0., f['near'] - n0) if nk else 0.) >= 0.4: qt = q; break
    if qt is None: return None
    d0 = d['seen'][qt] + ((min(0.3, V_EXT * (F[qt]['t'] - d['tch'][qt]))) if xe else 0.)
    for q in range(qt, min(len(F), qt + 60)):
        if not num(d['seen'][q]): continue
        x = d['seen'][q] + (min(0.3, V_EXT * (F[q]['t'] - d['tch'][q])) if xe else 0.) - d0
        if x - dl >= 0.1: return dict(qt=qt, qo=q, shift=d['so'][q] - d['so'][d['on']], on_k=d['so'][q] - d['k'])
    return None


if __name__ == '__main__':
    P = prep()
    L = [__doc__, '']
    VAR = [('recorded rule (check: shift 0)', dict()),
           ('lap1 near_k 0.5 n0 0.03', dict(near_k=0.5)),
           ('lap1 near_k 0.5 + lap1 delay 0.35', dict(near_k=0.5, delay=0.35)),
           ('x_ext all laps, delay 0.30 all', dict(x_ext=True)),
           ('x_ext all laps, delay 0.25 all', dict(x_ext=True, delay=0.25, lap1_only=False)),
           ('x_ext all + lap1 near_k 0.5, delay 0.25 later / 0.30 lap1', dict(x_ext=True, near_k=0.5)),
           ('x_ext all + lap1 near_k 0.5, delay 0.25 all', dict(x_ext=True, near_k=0.5, delay=0.25, lap1_only=False)),
           ]
    for name, kw in VAR:
        res = {True: [], False: []}; est = {True: [], False: []}; rows = []
        for d in P:
            kw2 = dict(kw)
            if kw.get('lap1_only') is False and 'near_k' in kw and not d['first']: kw2['near_k'] = None
            if kw.get('lap1_only') is False: kw2['lap1_only'] = False
            r = sim(d, **kw2)
            if r is None: rows.append(f"{d['run']}{'*' if d['first'] else ''}:none"); continue
            res[d['first']].append(r['on_k']); est[d['first']].append(d['ci'] + SLOPE * r['shift'])
            rows.append(f"{d['run']}{'*' if d['first'] else ''}:{r['on_k']:.2f}/{r['shift']:+.2f}")
        s = []
        for g in (True, False):
            v = res[g]
            if v: s.append(f"{'lap1' if g else 'later'}: onset-kmark median {st.median(v):.3f} sd {st.pstdev(v):.3f} range {min(v):.2f}..{max(v):.2f}; est clr_in median {st.median(est[g]):.1f} min {min(est[g]):.1f}")
        L.append(f'== {name}: ' + ' || '.join(s))
        L.append('   ' + ' '.join(rows))
    (OUT / 'C4_candidate_rules_replay.txt').write_text('\n'.join(L), encoding='utf8')
    print('\n'.join(L))
