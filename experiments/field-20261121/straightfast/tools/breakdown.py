"""straightfast-20261121 step 1: why is the PWM below 100 inside the three PWM-100 zones (recorded frames, car config)?
usage: python breakdown.py > ../proof/S1_breakdown.txt"""
import sys, collections
import numpy as np
from common import *


def classify(F):
    """adds F[i]['cls'] for frames inside a 100 zone"""
    last_block = None; prev = None
    for f in F:
        z = f['zone']
        if z is None: prev = None; last_block = None; continue
        if (z, f['lap']) != prev: last_block = None
        prev = (z, f['lap'])
        ls = f['ls']; pw = f['pwm'] or 0.
        if ls.get('blocked'):
            b = ls['blocked']
            if b == 'LOW': b = 'LOW:%s%s' % (f['sa_reason'], '+weave' if f['weave'] else '')
            elif b == 'ERROR': b = 'ERR_GUARD'
            elif b == 'STEER': b = 'STEER_GUARD'
            f['cls'] = 'BLOCK ' + b; last_block = f['cls']
        elif pw < float(ls.get('pwm', 0)) - 0.02: f['cls'] = ('CAP_AFTER (brake zone %s)' % f['bz']) if f['bz'] is not None else 'CAP_AFTER (other)'
        elif pw >= 99.5: f['cls'] = 'FULL'
        elif ls.get('taper'): f['cls'] = 'TAPER'
        else: f['cls'] = 'RAMP after ' + (last_block[6:] if last_block else 'zone entry')
    return F


def main():
    tot = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0., 0.]))   # zone -> cls -> [frames, deficit*dt, time]
    loss = collections.defaultdict(list); passes = collections.Counter(); sadist = collections.defaultdict(collections.Counter)
    for run in RUNS:
        F = classify(load(run)); t = np.array([f['t'] for f in F]); enc = [f['enc'] for f in F]
        last = max(f['lap'] for f in F if f['lap'] is not None)
        groups = collections.OrderedDict()
        for i, f in enumerate(F):
            if f['zone'] and f['lap'] is not None and 1 <= f['lap'] < last: groups.setdefault((f['zone'], f['lap']), []).append(i)
        for (z, lap), idx in groups.items():
            passes[z] += 1
            tt = t[idx]; pw = np.array([F[i]['pwm'] or 0. for i in idx]); dt = np.diff(tt, append=tt[-1]); dt[dt > 0.2] = 0.
            for n, i in enumerate(idx):
                c = tot[z][F[i]['cls']]; c[0] += 1; c[1] += max(0., 100. - pw[n]) * dt[n]; c[2] += dt[n]
                sadist[z][(F[i]['sa_level'], F[i]['sa_reason'], F[i]['weave'])] += 1
            vm = np.array([meas_v(t, enc, i) for i in idx]); ok = np.isfinite(vm)
            if ok.sum() < 5: continue
            v0 = vm[ok][0]; best = min(np.arange(0.85, 1.151, 0.01), key=lambda k: np.nanmean((sim(tt, pw, v0, k) - vm) ** 2))
            va = sim(tt, pw, v0, best); vb = sim(tt, np.full(len(tt), 100.), v0, best)
            da = np.concatenate([[0.], np.cumsum(0.5 * (va[1:] + va[:-1]) * np.diff(tt))]); db = np.concatenate([[0.], np.cumsum(0.5 * (vb[1:] + vb[:-1]) * np.diff(tt))])
            T = tt[-1] - tt[0]; Tn = float(np.interp(da[-1], db, tt - tt[0]))
            loss[z].append((T - Tn, float(np.sqrt(np.nanmean((va - vm) ** 2))), best, va[-1], vb[min(len(vb) - 1, int(np.searchsorted(db, da[-1])))], T, da[-1]))
    print('S1. PWM-100 zones, recorded frames of %s (complete laps only). Car config lap_speed.json 77ffbe32 / speed_adapt.json 774d8f9c.' % ', '.join(r[4:8] for r in RUNS))
    print('    class = the rule that kept the frame below 100. deficit = sum((100 - PWM) * dt) in PWM*s; share of the deficit ~ share of the lost time.')
    grand = 0.
    for z, pre, anc in ZONES:
        L = np.array(loss[z]); n = sum(c[0] for c in tot[z].values()); D = sum(c[1] for c in tot[z].values())
        med = float(np.median(L[:, 0])); grand += med
        print('\n== zone %s (%s), %d passes, %d frames | zone time median %.2f s, distance %.2f m | model, counted only up to the zone end: all-100 inside the zone would save median %.2f s per pass (min %.2f max %.2f; model rms vs encoder %.3f m/s, k %.2f); speed at the zone end %.2f -> %.2f m/s'
              % (z, anc, passes[z], n, np.median(L[:, 5]), np.median(L[:, 6]), med, L[:, 0].min(), L[:, 0].max(), np.median(L[:, 1]), np.median(L[:, 2]), np.median(L[:, 3]), np.median(L[:, 4])))
        print('   %-46s %7s %8s %9s %10s' % ('class', 'frames', '%frames', '%deficit', 's/pass'))
        for cls, c in sorted(tot[z].items(), key=lambda kv: -kv[1][1]):
            print('   %-46s %7d %7.1f%% %8.1f%% %10.3f' % (cls, c[0], 100. * c[0] / n, 100. * c[1] / D if D else 0., med * c[1] / D if D else 0.))
        print('   speed_adapt (level, reason, weave_slow) inside the zone: ' + ', '.join('%s %.0f%%' % (k, 100. * v / n) for k, v in sadist[z].most_common(8)))
    print('\nSUM of the three zone medians: %.2f s per lap counted only up to each zone end (PWM 100 on every in-zone frame, entry ramp included)' % grand)
    print('The speed carried past the zone end is not in this number; proof/E_candidates.txt counts the window up to the next curve entry (model and a model-free regression).')
    print('s/pass in the tables = the zone-only figure split by the share of the PWM deficit; read them as proportions.')


if __name__ == '__main__':
    main()
