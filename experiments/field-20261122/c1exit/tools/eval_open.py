"""open-loop replay evaluation: what the step would have sent on the recorded passes (same recorded camera / gyro / encoder input; the car's reaction to the new command is NOT modelled).
  python eval_open.py <replay out dir> c1 <tag> [runs]     curve 1: reference = <out>/off/<run>.old.jsonl (the car's programs + configs of that run), new = <out>/<tag>/<run>.new.jsonl
  python eval_open.py <replay out dir> c2 <tag> [runs]     curve 2 exit"""
import sys, json, numpy as np
from pathlib import Path
D = Path(sys.argv[1]); which = sys.argv[2]; tag = sys.argv[3]; runs = sys.argv[4:] or (['c6f3', '6f61', 'e0cd', '1eab', '4bbe'] if which == 'c1' else ['c6f3', '6f61'])
C = 1475.


def load(p):
    rows = []
    with open(p, encoding='utf8') as f:
        f.readline()
        for l in f:
            d = json.loads(l); p_ = d['proposal']
            rows.append(dict(seq=d['frame_seq'], pre=d.get('pre_steer'), sent=d.get('sent_steer'), t=p_.get('t'), lp=p_.get('lap_pos') or {}, lff=p_.get('lap_ff'), e=p_.get('image_error'),
                             gz=(p_.get('inputs') or {}).get('gz'), ct=p_.get('curve_trim'), pwm=d.get('pwm')))
    return rows


def g(x, d=np.nan): return d if x is None else x


tot_out = 0; tot_fr = 0
for run in runs:
    po = D / 'off' / ('%s.old.jsonl' % run); pn = D / tag / ('%s.new.jsonl' % run)
    if not (po.exists() and pn.exists()): print(run, 'missing replay output'); continue
    O = load(po); Nw = load(pn); n = min(len(O), len(Nw)); assert all(O[k]['seq'] == Nw[k]['seq'] for k in range(n))
    t = np.array([g(O[k]['t']) for k in range(n)]); po_ = np.array([g(O[k]['pre']) for k in range(n)], float) - C; pn_ = np.array([g(Nw[k]['pre']) for k in range(n)], float) - C
    so = np.array([g(O[k]['sent']) for k in range(n)], float) - C; sn = np.array([g(Nw[k]['sent']) for k in range(n)], float) - C
    anc = [O[k]['lp'].get('anchor') for k in range(n)]; lap = [O[k]['lp'].get('lap') for k in range(n)]
    cur = 'c1' if which == 'c1' else 'c2'; an_end = 'C1e' if which == 'c1' else 'C2e'
    act = np.array([bool(Nw[k]['lff'] and Nw[k]['lff'].get('curve') == cur) for k in range(n)])
    ends = [k for k in range(1, n) if anc[k] == an_end and anc[k - 1] != an_end]
    print('\n%s  tag %s  frames %d' % (run, tag, n))
    win = np.zeros(n, bool)
    if which == 'c1':
        print('  lap | trigger  x_pred  tb@trig plat | band frames  own  end(why, x_m, tb) | last 1.2 s before C1e: peak sent old -> new | in curve tb 40..120: pre median old -> new, sd old -> new, max frame step old -> new | after C1e 0..1 s: pre min/max old -> new')
        for ke in ends:
            k0 = ke
            while k0 > 0 and t[ke] - t[k0] < 4.5: k0 -= 1
            k1 = ke
            while k1 < n - 1 and t[k1] - t[ke] < 2.5: k1 += 1
            win[k0:k1 + 1] = True
            ks = [k for k in range(k0, k1) if act[k]]
            if not ks:
                print('  %3s | not triggered (anchor before the curve: %s)' % (lap[ke], anc[k0])); continue
            f = Nw[ks[0]]['lff']; b0 = f.get('band') or {}; e_ = [Nw[k]['lff'] for k in ks if Nw[k]['lff'].get('end')]
            own = sum(1 for k in ks if (Nw[k]['lff'].get('band') or {}).get('own')); last = Nw[ks[-1]]['lff']
            A = [k for k in range(k0, ke) if t[ke] - t[k] <= 1.2]; tbv = np.array([(Nw[k]['lff'].get('band') or {}).get('turned_deg', np.nan) if act[k] else np.nan for k in range(n)], float)
            M = [k for k in ks if 40 <= tbv[k] <= 120]; B = [k for k in range(ke, k1) if t[k] - t[ke] <= 1.0]
            st = lambda x, idx: np.nanmax(np.abs(np.diff(x[idx]))) if len(idx) > 2 else np.nan
            print('  %3s | %-4s %+6.2f  %+6.1f  %s | %4d  %4d  %s %.2f %.0f | %4.0f -> %4.0f | %+4.0f -> %+4.0f, %3.0f -> %3.0f, %3.0f -> %3.0f | %+4.0f/%+4.0f -> %+4.0f/%+4.0f' % (
                lap[ke], f.get('kind'), f.get('x_pred_m', np.nan), b0.get('turned_deg', np.nan), 'yes' if b0.get('plat') else 'NO ', len(ks), own, (e_[0].get('end') if e_ else '-'), last.get('x_m', np.nan), (last.get('band') or {}).get('turned_deg', np.nan),
                np.nanmax(so[A]), np.nanmax(sn[A]), np.nanmedian(po_[M]) if M else np.nan, np.nanmedian(pn_[M]) if M else np.nan, np.nanstd(po_[M]) if M else np.nan, np.nanstd(pn_[M]) if M else np.nan, st(po_, M), st(pn_, M),
                np.nanmin(po_[B]), np.nanmax(po_[B]), np.nanmin(pn_[B]), np.nanmax(pn_[B])))
        # lap from the start (no C1e-anchored window before the first exit on lap 0 is needed: it is the first exit)
    else:
        print('  lap | band END tb old -> new (x_m old -> new, why) | pre median by band tb (old band tb): 170-180  180-190  190-200  200-210 old | new | first 1.5 s after C2e: pre min/max old -> new')
        acto = np.array([bool(O[k]['lff'] and O[k]['lff'].get('curve') == 'c2') for k in range(n)])
        for ke in ends:
            k0 = ke
            while k0 > 0 and t[ke] - t[k0] < 6.0: k0 -= 1
            k1 = ke
            while k1 < n - 1 and t[k1] - t[ke] < 3.0: k1 += 1
            win[k0:k1 + 1] = True
            ko = [k for k in range(k0, k1) if acto[k]]; kn = [k for k in range(k0, k1) if act[k]]
            if not ko or not kn: print('  %3s | band not active' % lap[ke]); continue
            # heading scale: the old band's tb while it exists, extended by the integrated gyro after its END
            tbo = {k: (O[k]['lff'].get('band') or {}).get('turned_deg') for k in ko}; kl = ko[-1]; tb = np.full(n, np.nan); last_tb = None
            for k in range(k0, k1):
                if k in tbo and tbo[k] is not None: tb[k] = tbo[k]; last_tb = (k, tbo[k])
                elif last_tb is not None and g(O[k]['gz']) == g(O[k]['gz']): tb[k] = tb[k - 1] - O[k]['gz'] * (t[k] - t[k - 1])
            def med(x, a):
                idx = [k for k in range(k0, k1) if a <= tb[k] < a + 10]
                return np.nanmedian(x[idx]) if idx else np.nan
            eo = [O[k]['lff'] for k in ko if O[k]['lff'].get('end')]; en = [Nw[k]['lff'] for k in kn if Nw[k]['lff'].get('end')]
            B = [k for k in range(ke, k1) if t[k] - t[ke] <= 1.5]
            print('  %3s | %5.1f -> %5.1f (%.2f -> %.2f, %s -> %s) | %+4.0f %+4.0f %+4.0f %+4.0f | %+4.0f %+4.0f %+4.0f %+4.0f | %+4.0f/%+4.0f -> %+4.0f/%+4.0f' % (
                lap[ke], (O[ko[-1]]['lff'].get('band') or {}).get('turned_deg', np.nan), (Nw[kn[-1]]['lff'].get('band') or {}).get('turned_deg', np.nan), O[ko[-1]]['lff'].get('x_m', np.nan), Nw[kn[-1]]['lff'].get('x_m', np.nan),
                eo[0].get('end') if eo else '-', en[0].get('end') if en else '-', med(po_, 170), med(po_, 180), med(po_, 190), med(po_, 200), med(pn_, 170), med(pn_, 180), med(pn_, 190), med(pn_, 200),
                np.nanmin(po_[B]), np.nanmax(po_[B]), np.nanmin(pn_[B]), np.nanmax(pn_[B])))
    ch = np.abs(np.nan_to_num(sn) - np.nan_to_num(so)) > 0.05
    pw = sum(1 for k in range(n) if O[k]['pwm'] != Nw[k]['pwm'])
    print('  frames with a different sent command: %d of %d; outside the %s windows (-%.1f s .. +%.1f s around %s): %d; frames with a different throttle: %d' % (ch.sum(), n, cur, 4.5 if which == 'c1' else 6.0, 2.5 if which == 'c1' else 3.0, an_end, int((ch & ~win).sum()), pw))
    if (ch & ~win).any() and ends:
        ko = np.where(ch & ~win)[0]; te = np.array([t[k] for k in ends]); dt_ = np.array([np.min(np.where(t[k] - te >= 0, t[k] - te, 1e9)) for k in ko]); dv = np.abs(sn[ko] - so[ko])
        print('    outside-window differences: %.1f .. %.1f s after the nearest earlier %s; |difference| median %.1f max %.1f us%s' % (dt_.min(), dt_.max(), an_end, np.median(dv), dv.max(),
              '' if run in ('c6f3', '6f61') else '   (this run was driven with an older curve-2 / curve-4 lap_ff.json than the base of the step file: those parts differ too)'))
    tot_out += int((ch & ~win).sum()) if run in ('c6f3', '6f61') else 0; tot_fr += n if run in ('c6f3', '6f61') else 0
print('\nTOTAL (runs driven with the base lap_ff.json 052be3dc: c6f3, 6f61) frames %d, changed outside the windows %d' % (tot_fr, tot_out))
