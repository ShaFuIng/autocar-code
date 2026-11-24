"""proof C2 (c3refit): open-loop replay of each step. Old side = the car's program (adcde919 / 3b74ddc4) + the car's files of that run (out/off/<run>.old.jsonl:
6f61 = x-band r1 shadow, c6f3 = x-band r1 owning, d60e_new = d60e's input without any x-band); new side = candidate program + the step's files (out/<step>/<run>.new.jsonl). Same recorded camera / IMU / encoder stream on both sides,
so this shows WHAT the step changes in the command and WHERE, not how the car would then move.
  python eval_open.py <replay out dir> <step> [<step> ...]
Zones (from the old side's lap_pos): c3 = anchor C3s; c4 = anchor LAND8, or anchor C4e within 3 m (the band's exit); else = rest of the lap."""
import json, sys, os
from pathlib import Path
import numpy as np

D = Path(sys.argv[1]); RUNS = ('6f61', 'c6f3', 'd60e_new'); C = 1475.


def load(p):
    with open(p, encoding='utf8') as f:
        f.readline(); return [json.loads(l) for l in f]


def zone(p):
    lp = p.get('lap_pos') or {}; a = lp.get('anchor'); x = lp.get('since_anchor_m')
    if a == 'C3s': return 'c3', x
    if a == 'LAND8': return 'c4', x
    if a == 'C4e' and x is not None and x <= 3.0: return 'c4x', x
    return 'else', x


OLD = {r: load(D / 'off' / ('%s.old.jsonl' % r)) for r in RUNS}
for step in sys.argv[2:]:
    print('=' * 20, step)
    for r in RUNS:
        o = OLD[r]; n = load(D / step / ('%s.new.jsonl' % r))
        assert len(o) == len(n), (len(o), len(n))
        tot = len(o); ds = np.array([float(b['sent_steer']) - float(a['sent_steer']) if a.get('sent_steer') is not None and b.get('sent_steer') is not None else 0. for a, b in zip(o, n)])
        dp = np.array([float(b['pwm'] or 0) - float(a['pwm'] or 0) for a, b in zip(o, n)]); z = [zone(a['proposal']) for a in o]
        dq = np.array([float(b['proposal'].get('steer_us') or 0) - float(a['proposal'].get('steer_us') or 0) for a, b in zip(o, n)])   # command before the backlash stage
        rs = sum(1 for a, b in zip(o, n) if a.get('reason') != b.get('reason'))
        ch = np.abs(ds) > 0.05; zn = np.array([q[0] for q in z])
        print('%s: frames %d | steering changed on %d (c3 %d, c4 %d, c4 exit %d, rest of the lap %d) | throttle changed on %d | reason changed on %d' % (
            r, tot, ch.sum(), (ch & (zn == 'c3')).sum(), (ch & (zn == 'c4')).sum(), (ch & (zn == 'c4x')).sum(), (ch & (zn == 'else')).sum(), (np.abs(dp) > 1e-6).sum(), rs))
        if (ch & (zn == 'else')).any():
            ii = np.where(ch & (zn == 'else'))[0]
            segs = {}
            for i in ii: s = (o[i]['proposal'].get('lap_pos') or {}); k = '%s after %s' % (s.get('seg'), s.get('anchor')); segs.setdefault(k, []).append(abs(ds[i]))
            print('      rest of the lap: ' + '; '.join('%s: %d frames, max %.0f us' % (k, len(v), max(v)) for k, v in segs.items()))
        xs = np.array([q[1] if q[1] is not None else np.nan for q in z], float); m0 = zn == 'c3'
        XB = [((b['proposal'].get('arc2_c3') or {}).get('xband') or {}) for b in n]
        if not any(XB): continue
        gx = lambda k: np.array([q.get(k) if q.get(k) is not None else np.nan for q in XB], float)
        oc = np.array([float(a['proposal'].get('steer_us') or np.nan) - C for a in o]); nc = np.array([float(b['proposal'].get('steer_us') or np.nan) - C for b in n]); tg = gx('target_us') - C
        print('   c3  x bin (odometer)  frames changed | old cmd-centre med | new cmd med | band target med = u_x + heading + position + camera + gyro terms (med) | heading h_deg - h_ref | arc_lat y | new cmd: largest frame-to-frame step')
        for a_ in np.arange(0.3, 4.8, 0.3):
            m = m0 & (xs >= a_) & (xs < a_ + 0.3)
            if not m.any(): continue
            ii = np.where(m)[0]; cs = np.abs(np.diff(nc[ii]))[np.diff(ii) == 1]; st = np.nanmax(cs) if len(cs) else np.nan
            md = lambda v: np.nanmedian(v[m]) if np.isfinite(v[m]).any() else np.nan
            print('      %.1f-%.1f  %4d  %4d | %+5.0f | %+5.0f | %+5.0f = %+5.0f %+4.0f %+4.0f %+4.0f %+4.0f | %+5.1f | %+5.1f | %4.0f' % (a_, a_ + 0.3, m.sum(), (np.abs(dq[m]) > 0.05).sum(), md(oc), md(nc), md(tg), md(gx('prof_us') - C), md(gx('h_us')), md(gx('y_us')), md(gx('e_us')), md(gx('gz_us')),
                  md(gx('h_deg') - gx('h_ref')), md(gx('y_cm')), st))
        mm = m0 & (xs >= 0.9) & (xs < 4.05)
        stp = lambda v: np.nanpercentile(np.abs(np.diff(v[np.where(mm)[0]]))[np.diff(np.where(mm)[0]) == 1], 95)
        print('      0.9-4.05 m: frame-to-frame command step P95: old %.0f us, new %.0f us | command range old %+.0f..%+.0f, new %+.0f..%+.0f | share of frames with the band target at its cap / floor: %.1f %%' % (
            stp(oc), stp(nc), np.nanmin(oc[mm]), np.nanmax(oc[mm]), np.nanmin(nc[mm]), np.nanmax(nc[mm]), 100. * np.nanmean((np.abs(gx('raw_us')[mm] - C - 300.) < 0.5) | (np.abs(gx('raw_us')[mm] - C + 40.) < 0.5))))
