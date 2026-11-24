"""chatter-20261122 straights: per run / straight (lap_pos seg main_straight / diagonal_v / circle_straight / upper_straight) and per
exit window (C1e / C2e / C4e + 0..3 m):
  err_sd   image_error minus its 1.5 s centred mean, sd;  err_p2p  max-min of that;  err_mean  mean image_error
  gzlf_sd  gz smoothed over 0.15 s minus its 1.5 s centred mean (0.7..6 Hz band: steering-scale yaw swing), sd; per_s its period
           from zero crossings (s)
  flips/s  play-comp reversals per second
100-zone throttle (lap_speed zone names starting 'A main straight 100' / 'A diagonal 100' / 'A circle straight 100'):
  at100    share of zone frames with PWM >= 95
  blocks   number of block events (frame with lap_speed.blocked after a frame without) and their reasons
           (lap_speed.blocked value + speed_adapt reason), median position in the zone (rel_m).
python straights.py OUTDIR RUN [RUN ...]  -> OUTDIR/straights.tsv, OUTDIR/zones100.txt"""
import sys, os, statistics, collections
from common import load, num, pct, std, period

SEGS = dict(main_straight='main', diagonal_v='diag', circle_straight='circle', upper_straight='upper')
ZONES = (('A main straight 100', 'main'), ('A diagonal 100', 'diag'), ('A circle straight 100', 'circle'))


def mavg(t, x, w):
    out = []; j0 = 0; j1 = 0; s = 0.; n = len(x)
    # O(n^2) fallback is fine for <= 400 frames
    for i in range(n):
        vals = [x[j] for j in range(n) if abs(t[j] - t[i]) <= w / 2]
        out.append(sum(vals) / len(vals))
    return out


def seg_metrics(F, idx):
    t = [F[i]['t'] for i in idx]
    e = [(F[i]['t'], F[i]['err']) for i in idx if num(F[i]['err'])]
    g = [(F[i]['t'], F[i]['gz']) for i in idx if num(F[i]['gz'])]
    m = dict(dur=round(t[-1] - t[0], 2), flips=sum(1 for i in idx if F[i]['pc_rev']))
    m['flips_s'] = round(m['flips'] / m['dur'], 2) if m['dur'] > 0.5 else None
    if len(e) > 10:
        te = [a for a, _ in e]; xe = [b for _, b in e]; me = mavg(te, xe, 1.5); he = [a - b for a, b in zip(xe, me)]
        m.update(err_sd=round(std(he), 3), err_p2p=round(max(he) - min(he), 3), err_mean=round(statistics.mean(xe), 3), err_absmax=round(max(abs(v) for v in xe), 3))
    if len(g) > 10:
        tg = [a for a, _ in g]; xg = [b for _, b in g]; s1 = mavg(tg, xg, 0.15); s2 = mavg(tg, xg, 1.5); lf = [a - b for a, b in zip(s1, s2)]
        m.update(gzlf_sd=round(std(lf), 2), per_s=None if period(tg, lf) is None else round(period(tg, lf), 2))
    return m


def main():
    od = sys.argv[1]; os.makedirs(od, exist_ok=True); rows = []; zl = []
    for run in sys.argv[2:]:
        F = load(run)
        # straight segments by lap_pos.seg (whole segment, any owner) and exit windows
        groups = collections.defaultdict(list)
        for i, f in enumerate(F):
            if f['seg'] in SEGS and f['lap'] is not None: groups[(SEGS[f['seg']], f['lap'])].append(i)
            if f['exit'] and f['lap'] is not None: groups[(f['exit'] + '+0..3', f['lap'])].append(i)
        for (name, lap), idx in sorted(groups.items(), key=lambda x: (x[0][0], x[0][1])):
            if len(idx) < 15: continue
            m = seg_metrics(F, idx); m.update(run=run, seg=name, lap=lap)
            m['own_in'] = round(sum(1 for i in idx if F[i]['band'] and F[i]['band'][1]) / len(idx), 2)
            rows.append(m)
        # 100 zones
        Z = collections.defaultdict(lambda: dict(n=0, at=0, ev=collections.Counter(), pos=[], trips=set()))
        prevb = False
        for i, f in enumerate(F):
            ls = f['ls']
            if not isinstance(ls, dict): prevb = False; continue
            zn = str(ls.get('zone') or '')
            key = next((k for p, k in ZONES if zn.startswith(p)), None)
            if key is None: prevb = False; continue
            z = Z[key]; z['n'] += 1; z['trips'].add(f['lap'])
            if num(f['pwm']) and f['pwm'] >= 95: z['at'] += 1
            b = 'blocked' in ls
            if b and not prevb:
                z['ev'][f"{ls['blocked']}/{f['sa_reason']}"] += 1
                if num(ls.get('rel_m')): z['pos'].append(ls['rel_m'])
            prevb = b
        for k, z in Z.items():
            zl.append(f"{run} {k:6s} trips {len(z['trips'])} frames {z['n']:4d} at100 {z['at'] / max(1, z['n']):.0%}  block events {sum(z['ev'].values()):2d} "
                      f"({sum(z['ev'].values()) / max(1, len(z['trips'])):.2f}/trip)  pos median {pct(z['pos'], .5) if z['pos'] else '-'}  reasons {dict(z['ev'])}")
    keys = ['run', 'seg', 'lap', 'dur', 'flips', 'flips_s', 'err_mean', 'err_absmax', 'err_sd', 'err_p2p', 'gzlf_sd', 'per_s', 'own_in']
    with open(os.path.join(od, 'straights.tsv'), 'w', encoding='utf8') as g:
        g.write('\t'.join(keys) + '\n')
        for r in rows: g.write('\t'.join('' if r.get(k) is None else str(r.get(k)) for k in keys) + '\n')
    # per run x seg medians
    with open(os.path.join(od, 'straights_summary.txt'), 'w', encoding='utf8') as g:
        agg = collections.defaultdict(list)
        for r in rows: agg[(r['seg'], r['run'])].append(r)
        g.write('seg       run  laps  flips/s  err_sd  err_p2p  err|max|  gzlf_sd  period_s  own_in   (medians over laps)\n')
        for (s, run), rs in sorted(agg.items()):
            def md(k):
                v = [x[k] for x in rs if x.get(k) is not None]; return round(statistics.median(v), 3) if v else None
            g.write(f"{s:10s} {run} {len(rs):4d}  {md('flips_s')}  {md('err_sd')}  {md('err_p2p')}  {md('err_absmax')}  {md('gzlf_sd')}  {md('per_s')}  {md('own_in')}\n")
    with open(os.path.join(od, 'zones100.txt'), 'w', encoding='utf8') as g:
        g.write('\n'.join(zl) + '\n')


if __name__ == '__main__':
    main()
