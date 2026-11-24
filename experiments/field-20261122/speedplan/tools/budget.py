# -*- coding: utf-8 -*-
"""時間預算：讀 segtable.py 存的 pool pickle，列各路段在各輪的中位時間、相對 1.44 m/s 的損失、+0.05/+0.10 m/s 可省的秒數。"""
import sys, pickle, os
import numpy as np
from common import SCR, SEGS, VMAX

runs = sys.argv[1:]
P = pickle.load(open(os.path.join(SCR, 'pool_' + '_'.join(runs) + '.pkl'), 'rb'))
print('路段時間預算（每格＝該輪各圈中位；損失＝時間−長度/1.44；+0.05／+0.10＝以主輪 %s 的平均車速估）' % runs[0])
print('%-10s %6s | %s | %6s %6s | %6s %6s %6s' % ('路段', '長m', ' '.join('%7s' % r for r in runs), '主v', '損失s', '+.05', '+.10', '占圈%'))
tot = {r: 0. for r in runs}; rows = []
for sg in SEGS:
    D = P.get(sg[0]) or []
    med = {}
    for r in runs:
        T = [d['T'] for d in D if d['run'] == r]
        med[r] = float(np.median(T)) if T else None
        if T: tot[r] += med[r]
    Dm = [d for d in D if d['run'] == runs[0]]
    Lm = float(np.median([d['L'] for d in Dm])); Tm = med[runs[0]]; v = Lm / Tm
    loss = Tm - Lm / VMAX
    rows.append((sg[0], Lm, med, v, loss, Lm / v - Lm / (v + .05), Lm / v - Lm / (v + .10), Tm))
for name, Lm, med, v, loss, g5, g10, Tm in rows:
    print('%-10s %6.2f | %s | %6.3f %6.2f | %6.2f %6.2f %5.1f%%' % (name, Lm, ' '.join('%7s' % ('—' if med[r] is None else '%.2f' % med[r]) for r in runs), v, loss, g5, g10, 100 * Tm / tot[runs[0]]))
print('%-10s %6s | %s |' % ('合計', '', ' '.join('%7.2f' % tot[r] for r in runs)))
print()
print('損失排名（主輪）：')
for name, Lm, med, v, loss, g5, g10, Tm in sorted(rows, key=lambda x: -x[4]):
    print('  %-10s 損失 %.2f s（%.2f m，%.2f s，%.3f m/s）' % (name, loss, Lm, Tm, v))
