# -*- coding: utf-8 -*-
"""三個油門 100 區（主直線／斜線／圓圈直線）達成率、擋下事件與原因、車速讀數尖峰是否參與。"""
import sys, collections
from common import *
from segtable import dt_weights
runs = sys.argv[1:]
Z = ('LS_main100', 'LS_diag100', 'LS_circ100')
for run in runs:
    R = load(run)
    st = {z: collections.Counter() for z in Z}; ev = {z: collections.Counter() for z in Z}; pos = {z: [] for z in Z}; trips = {z: set() for z in Z}
    w = dt_weights(R); prev = None
    for i, (r, wi) in enumerate(zip(R, w)):
        z = r['ls']
        if z in Z:
            trips[z].add((r['lap'], z))
            st[z]['all'] += wi
            if (r['pwm'] or 0) >= 95: st[z]['>=95'] += wi
            if r['ls_blk']:
                st[z]['blocked'] += wi
                if prev is None or not prev['ls_blk'] or prev['ls'] != z:   # 擋下事件起點
                    why = r['ls_blk']
                    if why == 'LOW': why = 'LOW:%s%s' % (r['sa_r'], '+weave' if r['sa_ws'] else '')
                    ev[z][why] += 1; pos[z].append(round(r['ls_rel'] or 0, 1))
        prev = r
    print('run-%s' % run)
    for z in Z:
        n = len(trips[z]) or 1; a = st[z]['all'] or 1
        print('  %-11s 趟 %d｜區內 ≥95 占 %.0f%%｜被擋 %.0f%%｜擋下事件 %d（%.1f/趟）：%s｜位置(m) %s' % (
            z, n, 100 * st[z]['>=95'] / a, 100 * st[z]['blocked'] / a, sum(ev[z].values()), sum(ev[z].values()) / n,
            '、'.join('%s %d' % kv for kv in ev[z].most_common()), sorted(pos[z])))
