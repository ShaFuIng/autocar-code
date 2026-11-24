"""landing3-20261121: one row per landing. usage: python land.py run-... [...]"""
import sys, numpy as np
from data import load, landings, M

def rows(run):
    Q = load(run); T = np.array([q['t'] for q in Q]); out = []
    for k in landings(Q):
        lap = Q[k]['lap']
        # t_land = first frame with speed_adapt.landing (first level frame after the slope) near k
        j = next((i for i in range(max(0, k - 30), k + 5) if Q[i]['landing']), k); t0 = T[j]
        enc0 = Q[k]['enc']; s0 = Q[k]['s'] or 0.
        def x(i):   # metres after LAND8 by encoder (5 Hz encoder, so +-0.2 m)
            return None if Q[i]['enc'] is None or enc0 is None else s0 + (Q[i]['enc'] - enc0) * M
        end = next((i for i in range(k, len(Q)) if Q[i]['anc'] not in ('LAND8',)), len(Q) - 1)
        W = [i for i in range(k, end) if x(i) is not None]
        if len(W) < 60 or x(W[-1]) < 5.: continue
        ds = [i for i in range(max(0, k - 120), k) if Q[i]['anc'] == 'DOWN']   # slope frames
        gz = lambda i: Q[i]['gz'] or 0.
        def turned(a, b): return sum(gz(i) * (T[i] - T[i - 1]) for i in range(a + 1, b + 1) if T[i] - T[i - 1] < .3)
        def at(xm): return next((i for i in W if x(i) >= xm), W[-1])
        post = [i for i in range(j, min(len(Q), j + 200)) if T[i] - t0 <= 6.]
        big = [T[i] - t0 for i in post if Q[i]['pitch'] is not None and abs(Q[i]['pitch']) >= 3. and T[i] - t0 > .15]
        seg = lambda a, b: [i for i in W if a <= x(i) < b]
        e = lambda i: Q[i]['e'] or 0.
        i34 = at(3.4); i20 = at(2.0); i15 = at(1.5)
        ph = lambda name: max([x(i) for i in W if Q[i]['bo_phase'] == name] or [float('nan')])
        floor = next((x(i) for i in W if Q[i]['cm'] == ('after_down', 0, 'FLOOR')), float('nan'))
        bzend = max([x(i) for i in W if Q[i]['bz_cap'] == 31.0] or [float('nan')])
        landend = max([x(i) for i in W if Q[i]['landing']] or [float('nan')])
        c4 = next((x(i) for i in W if Q[i]['curve_by'] == 'c4' and (Q[i]['pwm'] or 0) >= 45), float('nan'))
        v = lambda a, b: (lambda S: (Q[S[-1]]['enc'] - Q[S[0]]['enc']) * M / max(1e-3, T[S[-1]] - T[S[0]]) if len(S) > 3 else float('nan'))(seg(a, b))
        # outer / inner clearance
        a45 = next((i for i in W if abs(turned(k, i)) >= 45.), W[-1])
        co = [Q[i]['clr_out'] for i in range(k, a45 + 1) if Q[i]['clr_out'] is not None]; ci = [Q[i]['clr_in'] for i in W if x(i) <= 3.4 and Q[i]['clr_in'] is not None]
        pul = [Q[i]['pulse'] for i in ds if Q[i]['pulse']]; ap = [p for p in pul if p.get('applied')]
        lift = max([(p.get('steer_us') or 0) - (p.get('in_steer_us') or 0) for p in ap] or [0.])
        slope_gz = np.mean([gz(i) for i in ds[-12:]]) if ds else float('nan')
        cs = seg(0.9, 2.6)
        out.append(dict(run=run[4:8], lap=lap, v_down=(lambda S: (Q[S[-1]]['enc'] - Q[S[0]]['enc']) * M / max(1e-3, T[S[-1]] - T[S[0]]) if len(S) > 3 else float('nan'))(ds[-20:]),
                        v0=v(0, .8), v2=v(1.5, 2.5), v3=v(2.6, 3.4), t34=T[i34] - T[k], t20_34=T[i34] - T[i20], pitch_settle=max(big) if big else 0., pitch_max=max([abs(Q[i]['pitch']) for i in post if Q[i]['pitch'] is not None and T[i] - t0 > .15] or [0]),
                        e_max=max(e(i) for i in seg(0, 1.5)), x_e05=max([x(i) for i in seg(0, 3.4) if abs(e(i)) > .5] or [0.]), gz_min=min(gz(i) for i in seg(0, 1.5)),
                        swing_gz=max([gz(i) for i in cs] or [0]), swing_e=min([e(i) for i in cs] or [0]), cap_end=ph('CAP'), ramp_end=ph('RAMP'), bo_end=max([x(i) for i in W if Q[i]['bo']] or [float('nan')]), floor=floor,
                        bz_end=bzend, land_end=landend, c4_from=c4, pwm_0_15=np.mean([Q[i]['pwm'] or 0 for i in seg(0, 1.5)]), pwm_15_34=np.mean([Q[i]['pwm'] or 0 for i in seg(1.5, 3.4)]),
                        turn05=turned(k, at(.5 + s0)), turn10=turned(k, at(1.0 + s0)), turn20=turned(k, i20), clr_out45=min(co) if co else float('nan'), clr_in34=min(ci) if ci else float('nan'),
                        pulse_n=len(pul), pulse_applied=len(ap), pulse_lift=lift, pulse_g=max([p.get('g') or 0 for p in pul] or [float('nan')]), slope_gz=slope_gz,
                        gz_sd_15_34=np.std([gz(i) for i in seg(1.5, 3.4)]), e_p90_15_34=np.percentile([abs(e(i)) for i in seg(1.5, 3.4)], 90), sent_max=max([Q[i]['sent'] or 0 for i in seg(0, 1.6)])))
    return out

if __name__ == '__main__':
    R = [r for run in sys.argv[1:] for r in rows(run)]
    def tab(title, cols, fmt):
        print(title); print('  run  lap | ' + ' | '.join(cols))
        for r in R: print('  %s %3s | ' % (r['run'], r['lap']) + ' | '.join(f % r[c] for c, f in zip(cols, fmt)))
    tab('A. speed and time (m/s, s; x = metres after LAND8 by encoder)', ['v_down', 'v0', 'v2', 'v3', 't34', 't20_34', 'pwm_0_15', 'pwm_15_34'], ['%.2f', '%.2f', '%.2f', '%.2f', '%.2f', '%.2f', '%.0f', '%.0f'])
    tab('B. settling: pitch (last |pitch|>=3 s after landing, max), error peak, last x with |e|>0.5, gz min, back-swing (max gz / min e in x 0.9-2.6), 1.5-3.4 m gz sd / |e| P90', ['pitch_settle', 'pitch_max', 'e_max', 'x_e05', 'gz_min', 'swing_gz', 'swing_e', 'gz_sd_15_34', 'e_p90_15_34'], ['%.2f', '%.1f', '%.2f', '%.2f', '%+.0f', '%+.0f', '%+.2f', '%.0f', '%.2f'])
    tab('C. layers (x m): bottom_owner CAP end / RAMP end / scope end, curve-map FLOOR, brake-zone cap 31 end, speed_adapt landing end, c4 PWM >=45 from, sent max 0-1.6 m', ['cap_end', 'ramp_end', 'bo_end', 'floor', 'bz_end', 'land_end', 'c4_from', 'sent_max'], ['%.2f', '%.2f', '%.2f', '%.2f', '%.2f', '%.2f', '%.2f', '%.0f'])
    tab('D. line: turned deg at +0.5 / +1.0 m / x 2.0, outer clearance min in first 45 deg, inner clearance min x<=3.4, pulse frames / applied / max lift us / max g, slope gz mean (last 12 slope frames)', ['turn05', 'turn10', 'turn20', 'clr_out45', 'clr_in34', 'pulse_n', 'pulse_applied', 'pulse_lift', 'pulse_g', 'slope_gz'], ['%+.0f', '%+.0f', '%+.0f', '%+.1f', '%+.1f', '%d', '%d', '%.0f', '%.2f', '%+.1f'])
