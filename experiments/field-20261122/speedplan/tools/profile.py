# -*- coding: utf-8 -*-
"""距離剖面：以某地標為 0，每 step m 一格，列各圈合併的車速（MCU）、油門、決定層、|e|、舵量偏中心、內外線淨空。
用法：python profile.py <anchor> <from_m> <to_m> <step> <run> [run ...] [--laps 2-]  （--laps 2- ＝只取第 2 圈起；1 ＝只第 1 圈）"""
import sys, collections
import numpy as np
from common import *
from segtable import src

args = [a for a in sys.argv[1:]]
lapsel = None
if '--laps' in args:
    i = args.index('--laps'); lapsel = args[i + 1]; del args[i:i + 2]
anc, a, b, step = args[0], float(args[1]), float(args[2]), float(args[3]); runs = args[4:]


def lap_ok(k):
    if lapsel is None: return True
    if lapsel.endswith('-'): return k >= int(lapsel[:-1])
    return k == int(lapsel)


nb = int(round((b - a) / step))
B = [collections.defaultdict(list) for _ in range(nb)]
nlap = 0
for run in runs:
    R = load(run); L, _ = laps(R)
    X = np.array([r['x'] for r in R])
    for lap in L:
        if lap['lap'] == 0 or not lap_ok(lap['lap']): continue
        x0 = lap['X'].get(anc)
        if x0 is None: continue
        nlap += 1
        for r in R:
            d = r['x'] - x0
            if d < a or d >= b: continue
            k = int((d - a) / step)
            if k >= nb: continue
            Bk = B[k]
            if r['v'] is not None: Bk['v'].append(r['v'])
            if r['pwm'] is not None: Bk['pwm'].append(float(r['pwm']))
            Bk['src'].append(src(r))
            if r['e'] is not None and r['valid']: Bk['e'].append(abs(r['e']))
            if r['st'] is not None: Bk['st'].append(r['st'] - 1475)
            if r['cin'] is not None: Bk['cin'].append(r['cin'])
            if r['cout'] is not None: Bk['cout'].append(r['cout'])
            if r['gz'] is not None: Bk['gz'].append(r['gz'])
print('剖面 %s %+.1f..%+.1f m（%s；%d 圈%s）' % (anc, a, b, ','.join(runs), nlap, '' if lapsel is None else '，圈 ' + lapsel))
print('%7s %4s | %5s %5s %5s | %5s %5s | %-44s | %5s %5s | %5s %5s | %5s %5s | %5s' % ('d', 'n', 'v P10', 'v中', 'v P90', 'PWM中', 'P10', '主決定層', '|e|中', 'max', 'st中', 'max', 'in最小', 'out最小', 'gz中'))
for k in range(nb):
    Bk = B[k]
    if not Bk['src']: continue
    c = collections.Counter(Bk['src']).most_common(2)
    tot = len(Bk['src'])
    s = '；'.join('%s %d%%' % (x, 100 * n / tot) for x, n in c)
    f = lambda v, q, fm='%.2f': (fm % np.percentile(v, q)) if v else '—'
    print('%+7.2f %4d | %5s %5s %5s | %5s %5s | %-44s | %5s %5s | %5s %5s | %5s %5s | %5s' % (
        a + k * step, tot, f(Bk['v'], 10), f(Bk['v'], 50), f(Bk['v'], 90), f(Bk['pwm'], 50, '%.0f'), f(Bk['pwm'], 10, '%.0f'), s[:44],
        f(Bk['e'], 50), f(Bk['e'], 100), f(Bk['st'], 50, '%.0f'), (('%.0f' % max(Bk['st'], key=abs)) if Bk['st'] else '—'),
        (('%.0f' % min(Bk['cin'])) if Bk['cin'] else '—'), (('%.0f' % min(Bk['cout'])) if Bk['cout'] else '—'), f(Bk['gz'], 50, '%.0f')))
