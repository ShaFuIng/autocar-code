"""chatter-20261122 measurement (offline, recorded runs only).
python measure.py OUTDIR RUN [RUN ...]
Writes OUTDIR/regions_per_lap.tsv (one row per run/region/lap instance), OUTDIR/regions_summary.tsv (per run/region),
OUTDIR/drivers.tsv (reversal driver counts per run/region), OUTDIR/flip_response.txt (yaw response after play-comp flips).
Definitions:
  pre   = proposal.steer_us (final command before the play compensation); sent = proposal.play_comp.steer_us (sent to the servo).
  rev   = play_comp.reversal True (the output compensation changed side: sent jumps by right_us+left_us = 110 us on top of the move).
  dpre / dsent = |frame-to-frame change| between consecutive frames of the instance with dt <= 0.1 s.
  swing = range of pre between this reversal and the previous one (same instance); 'near' = swing < 30 us (<= 2 x threshold 15).
  gz_hp = gz minus 0.5 s centred mean; per = main period from gz_hp zero crossings; y_hp = arc_lat y_cm minus 1.0 s centred mean.
  gzlf = gz smoothed 0.15 s minus 1.5 s centred mean (steering-scale yaw swing, ~0.7-6 Hz); gzlf_per its zero-crossing period.
  own   = share of frames whose final command equals a full-authority band output (c1/c2 lap_ff band own, c3 xband target, c4 band own).
Driver of a reversal: the component whose change from the frame of the detector's previous extreme to the switch frame is largest
in the direction of the switch. Band frames: prof (feed-forward; split prof_v when the speed reading changed and the curvature did
not), e (camera error term), gz (yaw-rate term), h / y (c3 r2 heading / position terms), clamp (sum of terms != target change).
Other frames: P, D, I (PID x gain k), yaw (yaw_inner term), other (rest: layers / floors / smoothing)."""
import sys, os, math, statistics, collections, random
from common import load, instances, pct, hp, std, period, num
from straights import mavg

TH = 15.


def own_applied(f):
    p = f['raw']; pre = f['pre']
    if not num(pre): return None
    a = p.get('lap_ff4')
    if isinstance(a, dict) and isinstance(a.get('band'), dict) and a['band'].get('state') == 'ON':
        b = a['band']
        if (b.get('own') or 0) >= 0.99 and num(b.get('band_us')) and abs(b['band_us'] - pre) < 0.6: return 'c4'
    a = p.get('arc2_c3')
    if isinstance(a, dict) and isinstance(a.get('xband'), dict):
        b = a['xband']
        if (b.get('own') or 0) >= 0.99 and num(b.get('target_us')) and abs(b['target_us'] - pre) < 0.6: return 'c3'
    a = p.get('lap_ff')
    if isinstance(a, dict) and isinstance(a.get('band'), dict) and a['band'].get('own') and num(a.get('steer_us')) and abs(a['steer_us'] - pre) < 0.6:
        return a.get('curve')
    return None


def comps(f, own):
    p = f['raw']
    if own == 'c4':
        b = p['lap_ff4']['band']; return dict(prof=b.get('prof_us'), e=b.get('e_us'), gz=b.get('gz_us')), b.get('kappa'), p['lap_ff4'].get('v')
    if own == 'c3':
        b = p['arc2_c3']['xband']; return dict(prof=b.get('prof_us'), e=b.get('e_us'), gz=b.get('gz_us'), h=b.get('h_us'), y=b.get('y_us')), b.get('kappa'), p['arc2_c3'].get('v')
    if own in ('c1', 'c2'):
        b = p['lap_ff']['band']; return dict(prof=b.get('prof_us'), e=b.get('e_us'), gz=b.get('gz_us')), b.get('kappa'), p['lap_ff'].get('v')
    k = f['k'] if num(f['k']) else 1.
    return dict(P=(f['P'] or 0) * k if num(f['P']) else None, D=(f['D'] or 0) * k if num(f['D']) else None,
                I=(f['I'] or 0) * k if num(f['I']) else None, yaw=f['yterm']), None, f['v']


def detector(F):
    """replica of play_comp.PlayComp on pre (frames with play_comp logged). Returns {switch index: extreme index}."""
    d = 0; ext = None; ei = None; out = {}; mism = 0; n = 0
    for i, f in enumerate(F):
        if f['sent'] is None or not num(f['pc_raw']): continue
        raw = f['pc_raw']; sw = False
        if ext is None: ext = 1475.; ei = i
        if d == 0:
            if raw - ext >= TH: d = 1; ext = raw; ei = i; sw = True
            elif ext - raw >= TH: d = -1; ext = raw; ei = i; sw = True
        elif d > 0:
            if raw >= ext: ext = raw; ei_new = i; ei = i
            elif ext - raw >= TH: out[i] = ei; d = -1; ext = raw; ei = i; sw = True
        else:
            if raw <= ext: ext = raw; ei = i
            elif raw - ext >= TH: out[i] = ei; d = 1; ext = raw; ei = i; sw = True
        n += 1
        if (i in out) != f['pc_rev'] and not (sw and f['pc_rev']): mism += 1
    return out, mism, n


def inst_metrics(F, idx, sw2ext):
    t = [F[i]['t'] for i in idx]
    dur = t[-1] - t[0] if len(t) > 1 else 0.
    revs = [i for i in idx if F[i]['pc_rev']]
    dpre = []; dsent = []
    for a, b in zip(idx, idx[1:]):
        if b - a == 1 and num(F[a]['t']) and num(F[b]['t']) and F[b]['t'] - F[a]['t'] <= 0.1:
            if num(F[a]['pre']) and num(F[b]['pre']): dpre.append(abs(F[b]['pre'] - F[a]['pre']))
            if num(F[a]['sent']) and num(F[b]['sent']): dsent.append(abs(F[b]['sent'] - F[a]['sent']))
    swings = []; prev = None
    for i in revs:
        if prev is not None:
            seg = [F[j]['pre'] for j in range(prev, i + 1) if num(F[j]['pre'])]
            if seg: swings.append(max(seg) - min(seg))
        prev = i
    tg = [(F[i]['t'], F[i]['gz']) for i in idx if num(F[i]['gz']) and num(F[i]['t'])]
    gzh = hp([a for a, _ in tg], [b for _, b in tg]) if len(tg) > 5 else []
    ty = [(F[i]['t'], F[i]['y']) for i in idx if num(F[i]['y']) and num(F[i]['t'])]
    yh = hp([a for a, _ in ty], [b for _, b in ty], 1.0) if len(ty) > 8 else []
    te = [(F[i]['t'], F[i]['err']) for i in idx if num(F[i]['err']) and num(F[i]['t'])]
    eh = hp([a for a, _ in te], [b for _, b in te]) if len(te) > 5 else []
    owns = collections.Counter(own_applied(F[i]) for i in idx)
    lf = []
    if len(tg) > 10:
        tt = [a for a, _ in tg]; xx = [b for _, b in tg]; lf = [a - b for a, b in zip(mavg(tt, xx, 0.15), mavg(tt, xx, 1.5))]
    return dict(n=len(idx), dur=round(dur, 2), rev=len(revs), rev_s=round(len(revs) / dur, 2) if dur > 0.3 else None,
                dpre_med=r1(pct(dpre, .5)), dpre_p95=r1(pct(dpre, .95)), dpre_max=r1(max(dpre) if dpre else None),
                dsent_med=r1(pct(dsent, .5)), dsent_p95=r1(pct(dsent, .95)), dsent_max=r1(max(dsent) if dsent else None),
                near=r2(sum(1 for s in swings if s < 30) / len(swings)) if swings else None, swing_med=r1(pct(swings, .5)),
                gz_sd=r1(std([b for _, b in tg])), gzhp_sd=r1(std(gzh)) if gzh else None,
                gz_per=r2(period([a for a, _ in tg], gzh)) if gzh else None,
                y_rng=r1(max(b for _, b in ty) - min(b for _, b in ty)) if ty else None, yhp_sd=r1(std(yh)) if yh else None,
                y_n=len(ty), errhp_sd=r3(std(eh)) if eh else None,
                own=r2(sum(v for k, v in owns.items() if k) / len(idx)),
                gzlf_sd=r1(std(lf)) if lf else None, gzlf_per=r2(period([a for a, _ in tg], lf)) if lf else None)


def r1(x): return None if x is None else round(x, 1)
def r2(x): return None if x is None else round(x, 2)
def r3(x): return None if x is None else round(x, 3)


def drivers(F, idx, sw2ext):
    c = collections.Counter(); sizes = collections.defaultdict(list)
    for i in idx:
        if not F[i]['pc_rev'] or i not in sw2ext: continue
        e = sw2ext[i]
        sgn = 1 if F[i]['pre'] > F[e]['pre'] else -1
        own_i = own_applied(F[i]); own_e = own_applied(F[e])
        if own_i and own_i == own_e:
            ci, ki, vi = comps(F[i], own_i); ce, ke, ve = comps(F[e], own_i)
            dd = {k: (ci[k] - ce[k]) for k in ci if num(ci.get(k)) and num(ce.get(k))}
            tot = F[i]['pre'] - F[e]['pre']; dd['clamp'] = tot - sum(dd.values())
            best = max(dd, key=lambda k: sgn * dd[k])
            if best == 'prof' and num(vi) and num(ve) and abs(vi - ve) > 0.02 and (not num(ki) or not num(ke) or abs(ki - ke) < 0.5): best = 'prof_v'
            c['band:' + best] += 1
            for k, v in dd.items(): sizes['band:' + k].append(sgn * v)
        else:
            ci, _, _ = comps(F[i], None); ce, _, _ = comps(F[e], None)
            dd = {k: (ci[k] - ce[k]) for k in ci if num(ci.get(k)) and num(ce.get(k))}
            tot = F[i]['pre'] - F[e]['pre']; dd['other'] = tot - sum(dd.values())
            best = max(dd, key=lambda k: sgn * dd[k])
            c['pid:' + best] += 1
    return c


def flip_response(F, sw2ext):
    """yaw change after reversals, sign-aligned so that positive = the car yawed the way the flip pushes (right flip -> gz down).
    near flips (pre swing < 30 us since the extreme) vs far flips vs random frames (baseline). Uses gz(t+tau) - gz(t)."""
    ts = [f['t'] for f in F]; out = collections.defaultdict(list)
    def gz_at(t0):
        lo, hi = 0, len(F) - 1
        while lo < hi:
            m = (lo + hi) // 2
            if (ts[m] or 0) < t0: lo = m + 1
            else: hi = m
        return F[lo]['gz'] if num(F[lo]['gz']) and abs((ts[lo] or 0) - t0) < 0.06 else None
    rnd = random.Random(1)
    for i, f in enumerate(F):
        if not num(f['t']) or not num(f['gz']) or f['reg'] is None: continue
        cand = []
        if f['pc_rev'] and i in sw2ext:
            e = sw2ext[i]; d = 1 if f['pre'] > F[e]['pre'] else -1
            swing = abs(f['pre'] - F[e]['pre'])
            cand.append(('near' if swing < 30 else 'far', d))
        if rnd.random() < 0.1: cand.append(('random', 1 if rnd.random() < 0.5 else -1))
        for kind, d in cand:
            for tau in (0.2, 0.3, 0.45):
                g1 = gz_at(f['t'] + tau)
                if g1 is not None: out[(kind, tau)].append(-d * (g1 - f['gz']))
    return out


def main():
    od = sys.argv[1]; os.makedirs(od, exist_ok=True)
    rows = []; summ = []; drv = []; flips = collections.defaultdict(list); chk = []
    for run in sys.argv[2:]:
        F = load(run)
        sw2ext, mism, n = detector(F); chk.append(f'{run}: detector replica mismatches {mism} of {n} play_comp frames')
        for key in ('reg', 'exit'):
            agg = collections.defaultdict(list); dc = collections.defaultdict(collections.Counter)
            for lab, lap, idx in instances(F, key):
                if len(idx) < 8: continue
                m = inst_metrics(F, idx, sw2ext); m.update(run=run, reg=lab, lap=lap); rows.append(m); agg[lab].append(m)
                dc[lab] += drivers(F, idx, sw2ext)
            for lab, ms in agg.items():
                dur = sum(m['dur'] for m in ms); rv = sum(m['rev'] for m in ms)
                def med(k):
                    v = [m[k] for m in ms if m[k] is not None]; return r2(statistics.median(v)) if v else None
                summ.append(dict(run=run, reg=lab, inst=len(ms), dur=round(dur, 1), rev=rv, rev_s=r2(rv / dur) if dur else None,
                                 **{k: med(k) for k in ('dpre_med', 'dpre_p95', 'dpre_max', 'dsent_med', 'dsent_p95', 'dsent_max', 'near',
                                                        'gz_sd', 'gzhp_sd', 'gz_per', 'gzlf_sd', 'gzlf_per', 'y_rng', 'yhp_sd', 'errhp_sd', 'own')}))
                tot = sum(dc[lab].values())
                drv.append(dict(run=run, reg=lab, n=tot, **{k: f'{v}({v / tot:.0%})' for k, v in dc[lab].most_common()}))
        fr = flip_response(F, sw2ext)
        for k, v in fr.items(): flips[k] += v
    def tsv(name, R):
        keys = []
        for r in R:
            for k in r:
                if k not in keys: keys.append(k)
        with open(os.path.join(od, name), 'w', encoding='utf8') as g:
            g.write('\t'.join(keys) + '\n')
            for r in R: g.write('\t'.join('' if r.get(k) is None else str(r.get(k)) for k in keys) + '\n')
    tsv('regions_per_lap.tsv', rows); tsv('regions_summary.tsv', summ); tsv('drivers.tsv', drv)
    with open(os.path.join(od, 'flip_response.txt'), 'w', encoding='utf8') as g:
        g.write('\n'.join(chk) + '\n\n')
        g.write('yaw change after a play-comp reversal, sign-aligned (+ = yawed the way the flip pushes), deg/s; all runs given\n')
        for k in sorted(flips):
            v = flips[k]; g.write(f'{k[0]:7s} tau {k[1]:.2f}s  n {len(v):5d}  mean {statistics.mean(v):6.2f}  median {statistics.median(v):6.2f}  sd {statistics.pstdev(v):6.2f}\n')


if __name__ == '__main__':
    main()
