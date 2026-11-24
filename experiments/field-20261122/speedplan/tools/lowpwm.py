# -*- coding: utf-8 -*-
"""各路段每圈落在 min_pwm／mid_pwm 附近的時間，與 weave_slow 標記：判斷 min_pwm、mid_pwm 這類全圈鍵的影響範圍。"""
import sys, collections
from common import *
from segtable import dt_weights
runs = sys.argv[1:]
A = collections.defaultdict(lambda: collections.Counter()); nl = 0
for run in runs:
    R = load(run); L, _ = laps(R)
    for lap in L:
        if lap['lap'] == 0 or lap['t1'] is None: continue
        nl += 1
        for sg in SEGS:
            b = seg_bounds(lap, sg)
            if b is None: continue
            F = frames_in(R, *b); w = dt_weights(F)
            for r, wi in zip(F, w):
                if r['reason'] != 'PROPOSAL_ONLY':
                    A[sg[0]]['lost/hold'] += wi; continue
                p = r['pwm'] or 0
                if r['sa_l'] == 'LOW' and r['sa_ws']: A[sg[0]]['LOW+weave_slow'] += wi
                if r['sa_l'] == 'LOW' and not r['sa_ws'] and r['sa_c'] is None and r['sa_up'] is None and not r['sa_land']: A[sg[0]]['LOW無彎鍵(非weave)'] += wi
                if r['sa_l'] == 'MID' and r['sa_c'] is None and r['sa_up'] is None and not r['sa_land']: A[sg[0]]['MID無彎鍵'] += wi
                if p <= 24.5: A[sg[0]]['PWM≤24.5'] += wi
                elif p <= 28.5: A[sg[0]]['PWM 24.5-28.5'] += wi
                elif p <= 30.5: A[sg[0]]['PWM 28.5-30.5'] += wi
print('每圈平均秒數（%s，%d 圈）' % (','.join(runs), nl))
keys = ['lost/hold', 'LOW+weave_slow', 'LOW無彎鍵(非weave)', 'MID無彎鍵', 'PWM≤24.5', 'PWM 24.5-28.5', 'PWM 28.5-30.5']
print('%-10s ' % '路段' + ' '.join('%14s' % k for k in keys))
tot = collections.Counter()
for sg in SEGS:
    c = A[sg[0]]; tot.update(c)
    print('%-10s ' % sg[0] + ' '.join('%14.2f' % (c[k] / nl) for k in keys))
print('%-10s ' % '合計' + ' '.join('%14.2f' % (tot[k] / nl) for k in keys))
