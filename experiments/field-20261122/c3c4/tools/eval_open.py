"""proof C2: open-loop replay of each step. Old side = the car's program + the car's files of that run (out/off/<run>.old.jsonl);
new side = lap_ff.py 1a1ea129 + the step's files (out/<step>/<run>.new.jsonl). Same recorded camera / IMU / encoder stream on both sides,
so this shows WHAT the step changes in the command and WHERE, not how the car would then move.
  python eval_open.py <replay out dir> <step> [<step> ...]
Zones (from the old side's lap_pos): c3 = anchor C3s; c4 = anchor LAND8, or anchor C4e within 3 m (the band's exit); else = rest of the lap."""
import json, sys, os
from pathlib import Path
import numpy as np

D = Path(sys.argv[1]); RUNS = ('d60e', 'ccf1'); C = 1475.


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
        for zname, w, lo, hi in (('c3', 0.3, 0.0, 6.9), ('c4', 0.6, 0.0, 9.0)):
            m0 = zn == zname
            if not (ch & m0).any(): continue
            print('   %s  x bin     frames changed | old cmd-centre med | new med | delta med / min / max (before the backlash stage)' % zname)
            xs = np.array([q[1] if q[1] is not None else np.nan for q in z], float)
            for a in np.arange(lo, hi, w):
                m = m0 & (xs >= a) & (xs < a + w)
                if not m.any(): continue
                oc = np.array([float(o[i]['proposal'].get('steer_us') or np.nan) - C for i in np.where(m)[0]]); nc = np.array([float(n[i]['proposal'].get('steer_us') or np.nan) - C for i in np.where(m)[0]])
                print('      %.1f-%.1f  %4d  %4d | %+5.0f | %+5.0f | %+5.0f / %+5.0f / %+5.0f' % (a, a + w, m.sum(), (np.abs(dq[m]) > 0.05).sum(), np.nanmedian(oc), np.nanmedian(nc), np.nanmedian(dq[m]), np.nanmin(dq[m]), np.nanmax(dq[m])))
