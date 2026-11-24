"""c1lap1b: curve-1 exit (C1e anchor, since_anchor 0..3 m) for band and non-band passes: y / clr_out / clr_in / error / steer per 0.3 m,
where the outside-clearance minimum of the pass occurs (band turned angle or metres after C1e), band state at C1e, who writes the output,
PID integral, i_ramp, and the distance after C1e at which |y| first falls below 3 cm ("back to the line")."""
import statistics as st, json
from pathlib import Path
from lib import load, num
from passes import curve_passes, med, mn, mx
OUT = Path(__file__).resolve().parents[1] / 'proof'
RUNS = ['3f22', 'fd22', 'b23a', '2b72', 'b619', '1377', 'e69d', '54cb', '1cd7', '9af6', 'c6f3', '6f61', 'e0cd', '2728', '7658']
BINS = [0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.4, 3.0]


def one(run):
    F = load(run); out = []
    for p in curve_passes(F):
        q_end = (p['ex'][-1] if p['ex'] else p['j'] + 150)
        span = [F[q] for q in range(p['band'][0] if p['band'] else p['j'], min(len(F), q_end + 1))]
        ex = [F[q] for q in p['ex']]
        if not ex: continue
        r = dict(run=run, first=p['first'], band=bool(p['band']), lap=F[p['j']]['lap'])
        # outside clearance minimum location over curve + exit
        co = [(f['clr_out'], f) for f in span if num(f['clr_out'])]
        if co:
            m, f = min(co, key=lambda z: z[0])
            r['co_min'] = m; r['co_min_where'] = ('C1e+%.2f' % f['sa']) if f['anchor'] == 'C1e' else ('tb %.0f' % f['tb'] if num(f['tb']) else (f['seg'] or '?'))
        ci = [(f['clr_in'], f) for f in span if num(f['clr_in'])]
        if ci:
            m, f = min(ci, key=lambda z: z[0])
            r['ci_min'] = m; r['ci_min_where'] = ('C1e+%.2f' % f['sa']) if f['anchor'] == 'C1e' else ('tb %.0f' % f['tb'] if num(f['tb']) else (f['seg'] or '?'))
        f0 = ex[0]
        r['c1e_tb'] = f0['tb']; r['c1e_state'] = f0['lf_state']; r['c1e_owner'] = f0['owner']; r['c1e_I'] = f0['I']
        bend = [f for f in ex if f['lf_curve'] == 'c1']
        r['band_end_sa'] = bend[-1]['sa'] if bend else None
        r['band_end_tb'] = bend[-1]['tb'] if bend else None
        rows = {}
        for b0 in BINS:
            w = [f for f in ex if b0 <= f['sa'] < b0 + 0.3]
            rows[b0] = dict(y=med([f['y'] for f in w]), co=mn([f['clr_out'] for f in w]), err=med([f['err'] for f in w]), pre=med([f['pre'] for f in w]), I=med([f['I'] for f in w]),
                            own=(w[0]['owner'] if w else None))
        r['rows'] = rows
        back = next((f['sa'] for f in ex if num(f['y']) and abs(f['y']) <= 3.0 and f['sa'] > 0.3), None)
        r['back_sa'] = back
        r['y_peak'] = mx([f['y'] for f in ex]); r['err_peak'] = mx([f['err'] for f in ex])
        out.append(r)
    return out


if __name__ == '__main__':
    A = []
    for run in RUNS: A += one(run)
    L = ['curve-1 exits. y (cm, + = left = outside of curve 1 / left of the diagonal lane centre), co = clr_out min, err = near error, pre = steer before backlash comp, I = PID integral term (us).']
    for g, G in (('band lap 1', [a for a in A if a['band'] and a['first']]), ('band lap 2+', [a for a in A if a['band'] and not a['first']]),
                 ('pre-band lap 1', [a for a in A if not a['band'] and a['first']]), ('pre-band lap 2+', [a for a in A if not a['band'] and not a['first']])):
        L.append(f'== {g} n={len(G)}')
        for a in G:
            L.append(f"  {a['run']} lap{a['lap']}: co_min {a.get('co_min')} @ {a.get('co_min_where')} | ci_min {a.get('ci_min')} @ {a.get('ci_min_where')} | at C1e tb {a['c1e_tb']} state {a['c1e_state']} owner {a['c1e_owner']} I {None if a['c1e_I'] is None else round(a['c1e_I'], 1)}"
                     f" | band ends C1e+{a['band_end_sa']} tb {a['band_end_tb']} | y_peak {a['y_peak']} err_peak {None if a['err_peak'] is None else round(a['err_peak'], 2)} back(|y|<=3) C1e+{a['back_sa']}")
            L.append('     ' + '  '.join(f"{b:.1f}:y{a['rows'][b]['y']},co{a['rows'][b]['co']},e{None if a['rows'][b]['err'] is None else round(a['rows'][b]['err'], 2)},u{None if a['rows'][b]['pre'] is None else round(a['rows'][b]['pre'])},{a['rows'][b]['own']}" for b in BINS))
        for b in BINS:
            ys = [a['rows'][b]['y'] for a in G if num(a['rows'][b]['y'])]; es = [a['rows'][b]['err'] for a in G if num(a['rows'][b]['err'])]
            us = [a['rows'][b]['pre'] for a in G if num(a['rows'][b]['pre'])]; cs = [a['rows'][b]['co'] for a in G if num(a['rows'][b]['co'])]
            if ys: L.append(f"   C1e+{b:.1f}: y median {st.median(ys):.1f} (min {min(ys):.1f} max {max(ys):.1f})  co min {min(cs) if cs else None}  err median {st.median(es):.2f}  steer median {st.median(us):.0f}")
        bk = [a['back_sa'] for a in G if num(a['back_sa'])]
        if bk: L.append(f'   back to |y|<=3 cm: median C1e+{st.median(bk):.2f} m (n {len(bk)} of {len(G)})')
    (OUT / 'D1_exits.txt').write_text('\n'.join(L), encoding='utf8')
    print('\n'.join(L))
