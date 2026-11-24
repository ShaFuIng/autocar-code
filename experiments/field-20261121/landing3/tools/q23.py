"""landing3-20261121: questions 2 and 3 - the pre-landing pulse, the right cap, the slope-mouth speed. Per landing + correlations.
usage: python q23.py"""
import sys, numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from data import load, landings, M
import land

RUNS = ('run-e1b4-20261121', 'run-b93e-20261121', 'run-9934-20261121', 'run-4a78-20261121')
NOTE = {'e1b4': 'pulse mild ON, mouth 34, new battery', 'b93e': 'pulse off, mouth 34, new battery', '9934': 'pulse off, mouth 34, old battery', '4a78': 'pulse off, mouth 40, pre-impact afternoon'}


def extra(run):
    Q = load(run); T = np.array([q['t'] for q in Q]); out = {}
    for k in landings(Q):
        lap = Q[k]['lap']; ds = [i for i in range(max(0, k - 150), k) if Q[i]['anc'] == 'DOWN']
        if len(ds) < 20: continue
        e = lambda i: Q[i]['e'] or 0.; gz = lambda i: Q[i]['gz'] or 0.
        last = ds[-15:]
        pul = [(i, Q[i]['pulse']) for i in ds if Q[i]['pulse']]
        t_land = T[k]
        # slope: time on the slope (pitch < -8), yaw turned on the slope, speed at the mouth and at the end
        steep = [i for i in ds if Q[i]['pitch'] is not None and Q[i]['pitch'] < -8.]
        turned = sum(gz(i) * (T[i] - T[i - 1]) for i in steep[1:] if T[i] - T[i - 1] < .3)
        def v(S): return float('nan') if len(S) < 4 or Q[S[0]]['enc'] is None or Q[S[-1]]['enc'] is None else (Q[S[-1]]['enc'] - Q[S[0]]['enc']) * M / max(1e-3, T[S[-1]] - T[S[0]])
        # whole section: DASHe+2.0 -> LAND8 x 3.4 and -> x 6.0
        up = [i for i in range(max(0, k - 400), k) if Q[i]['anc'] == 'DASHe' and (Q[i]['s'] or 0) >= 2.0]
        enc0 = Q[k]['enc']; s0 = Q[k]['s'] or 0.
        x = lambda i: None if Q[i]['enc'] is None or enc0 is None else s0 + (Q[i]['enc'] - enc0) * M
        end = next((i for i in range(k, len(Q)) if Q[i]['anc'] != 'LAND8'), len(Q) - 1)
        i34 = next((i for i in range(k, end) if x(i) is not None and x(i) >= 3.4), None); i60 = next((i for i in range(k, end) if x(i) is not None and x(i) >= 6.0), None)
        mouth = [i for i in range(max(0, k - 400), k) if Q[i]['anc'] == 'DASHe' and (Q[i]['s'] or 0) >= 2.4]
        guard = sum(1 for i in range(max(0, k - 150), min(len(Q), k + 120)) if Q[i]['guard'])
        out[lap] = dict(e_slope=float(np.mean([e(i) for i in last])), e_slope_max=max(e(i) for i in ds[-30:]), turned_slope=turned, t_slope=(T[steep[-1]] - T[steep[0]]) if steep else float('nan'),
                        v_mouth=v(mouth[-12:]) if mouth else float('nan'), v_slope_in=v(steep[:14]), v_slope_out=v(steep[-14:]),
                        pulse_first_dt=(T[pul[0][0]] - t_land) if pul else float('nan'), pulse_xe=[round(p.get('x_e') or 0, 2) for _, p in pul][:1] + [round(p.get('x_e') or 0, 2) for _, p in pul][-1:],
                        pulse_g=[round(p.get('g') or 0, 2) for _, p in pul], t_sec34=(T[i34] - T[up[0]]) if up and i34 else float('nan'), t_sec60=(T[i60] - T[up[0]]) if up and i60 else float('nan'),
                        t_up=(T[steep[0]] - T[up[0]]) if up and steep else float('nan'), guard=guard, x_last_slope=(lambda L: L[-1] if L else float('nan'))([Q[i]['s'] for i in ds if Q[i]['s'] is not None]))
    return out


def main():
    R = []
    for run in RUNS:
        ex = extra(run)
        for r in land.rows(run):
            if r['lap'] in ex: r.update(ex[r['lap']]); R.append(r)
    print('E. slope and pre-landing (per landing). e_slope = mean image error on the last 15 slope frames (+ = line to the right / car left of it)')
    print('  run  lap | v mouth | v slope in/out | t slope | turned on slope deg | e_slope (max) | pulse: first frame dt before LAND8, x_e first..last, g per frame | outer clr 45deg | inner clr x<=3.4 | guard frames')
    for r in R:
        print('  %s %3s | %.2f | %.2f / %.2f | %.2f | %+5.1f | %+.2f (%+.2f) | %s %s %s | %+5.1f | %+5.1f | %d' % (r['run'], r['lap'], r['v_mouth'], r['v_slope_in'], r['v_slope_out'], r['t_slope'], r['turned_slope'], r['e_slope'], r['e_slope_max'],
              ('%+.2f s' % r['pulse_first_dt']) if r['pulse_n'] else '-', r['pulse_xe'] if r['pulse_n'] else '', r['pulse_g'] if r['pulse_n'] else '', r['clr_out45'], r['clr_in34'], r['guard']))
    print()
    print('F. section times (s): DASHe+2.0 m -> slope start | slope | LAND8 x 0.28 -> 3.4 | DASHe+2.0 -> x 3.4 | DASHe+2.0 -> x 6.0')
    for run in RUNS:
        S = [r for r in R if r['run'] == run[4:8]]
        if not S: continue
        f = lambda key: ' '.join('%.2f' % r[key] for r in S)
        print('  %s (%s): up %s | slope %s | 0-3.4 %s | to 3.4: %s (median %.2f) | to 6.0: %s (median %.2f) | v mouth median %.2f, v slope out median %.2f, pitch settle max %.2f s' % (
            run[4:8], NOTE[run[4:8]], f('t_up'), f('t_slope'), f('t34'), f('t_sec34'), np.nanmedian([r['t_sec34'] for r in S]), f('t_sec60'), np.nanmedian([r['t_sec60'] for r in S]),
            np.nanmedian([r['v_mouth'] for r in S]), np.nanmedian([r['v_slope_out'] for r in S]), max(r['pitch_settle'] for r in S)))
    print()
    A = [r for r in R if r['run'] != '4a78']   # mouth 34 only
    def corr(a, b, S=A):
        x = np.array([r[a] for r in S], float); y = np.array([r[b] for r in S], float); m = np.isfinite(x) & np.isfinite(y)
        return float(np.corrcoef(x[m], y[m])[0, 1]) if m.sum() > 3 else float('nan'), int(m.sum())
    print('G. correlations over the mouth-34 landings (n = %d; Pearson r, small sample - |r| below ~0.55 is not significant at n 13)' % len(A))
    for a, b in (('v_slope_out', 'clr_out45'), ('v_slope_out', 'e_max'), ('v_slope_out', 'gz_min'), ('e_slope', 'clr_out45'), ('turned_slope', 'clr_out45'), ('turn05', 'clr_out45'), ('turn10', 'clr_out45'), ('gz_min', 'clr_out45'),
                 ('gz_min', 'clr_in34'), ('gz_min', 'swing_e'), ('turn10', 'swing_e'), ('turn10', 'clr_in34'), ('clr_out45', 'clr_in34'), ('v_slope_out', 't34'), ('e_slope', 'turn05')):
        print('   %-14s vs %-10s r %+.2f (n %d)' % (a, b, *corr(a, b)))
    print()
    print('H. pulse (e1b4) vs no pulse (b93e + 9934), same mouth cap 34')
    P = [r for r in R if r['run'] == 'e1b4']; N = [r for r in R if r['run'] in ('b93e', '9934')]
    for name, S in (('e1b4 all', P), ('e1b4 pulse applied', [r for r in P if r['pulse_applied']]), ('e1b4 pulse not applied', [r for r in P if not r['pulse_applied']]), ('no-pulse runs all', N),
                    ('no-pulse runs, e_slope >= 0.15', [r for r in N if r['e_slope'] >= .15]), ('no-pulse runs, e_slope < 0.15', [r for r in N if r['e_slope'] < .15])):
        if not S: print('   %-32s n 0' % name); continue
        g = lambda key: np.array([r[key] for r in S], float)
        print('   %-32s n %d | outer clr 45deg: %s (median %+.1f, <-10: %d) | inner clr: median %+.1f min %+.1f | turned +0.5 m %+.0f, +1.0 m %+.0f | e_slope %+.2f | e max %.2f | gz min %+.0f | back-swing e %+.2f' % (
            name, len(S), ' '.join('%+.0f' % v for v in g('clr_out45')), np.nanmedian(g('clr_out45')), int((g('clr_out45') < -10).sum()), np.nanmedian(g('clr_in34')), np.nanmin(g('clr_in34')), np.median(g('turn05')), np.median(g('turn10')),
            np.median(g('e_slope')), np.median(g('e_max')), np.median(g('gz_min')), np.median(g('swing_e'))))


if __name__ == '__main__':
    main()
