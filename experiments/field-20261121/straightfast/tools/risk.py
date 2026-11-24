"""straightfast-20261121 risk numbers (recorded frames + the emulated candidates).
usage: python risk.py > ../proof/R_risk.txt"""
import sys, json, collections
from pathlib import Path
import numpy as np
from common import *
from breakdown import classify
import emul, evalc

ON = HERE.parent / 'on'


def raw_owner(run):
    out = []
    for l in open(str(EXP / run / 'pid.jsonl'), encoding='utf8'):
        if '"kind": "sample"' not in l[:40]: continue
        p = json.loads(l).get('proposal') or {}
        if p.get('t') is None: continue
        out.append((p.get('owner'), bool(p.get('crossing_pass')), bool(p.get('arc2_flat') or p.get('arc2_blind')), (p.get('speed_adapt') or {}).get('target_pwm')))
    return out


def main():
    cm = evalc.cand_module()
    cfgs = dict(A2=(json.loads((ON / 'lap_speed.A2_relax_rate120.json').read_text(encoding='utf8')), None), B2=(json.loads((ON / 'lap_speed.B2_two_level_all.json').read_text(encoding='utf8')), cm))
    print('R. risk numbers, runs %s' % ', '.join(r[4:8] for r in RUNS))
    # 1. where the block events are (position histogram) and what steers there
    print('\n1. Where do the blocked frames of the three 100 zones sit (0.5 m bins after the anchor), and who owns the steering there?')
    pos = collections.defaultdict(collections.Counter); own = collections.defaultdict(collections.Counter); allpos = collections.defaultdict(collections.Counter)
    ev_gz = []
    for run in RUNS:
        F = classify(load(run)); O = raw_owner(run); assert len(F) == len(O)
        for f, o in zip(F, O):
            if not f['zone'] or f['since'] is None: continue
            b = int(f['since'] / 0.5); allpos[f['zone']][b] += 1
            if f['cls'].startswith('BLOCK'): pos[f['zone']][b] += 1; own[f['zone']][o[0]] += 1
    for z, _, anc in ZONES:
        print('  %-6s blocked frames per bin (of all frames in the bin): %s' % (z, '  '.join('%s+%.1f: %d/%d' % (anc, b * 0.5, pos[z][b], allpos[z][b]) for b in sorted(allpos[z]) if pos[z][b])))
        print('         steering owner on the blocked frames: %s' % dict(own[z].most_common(6)))
    # 2. candidates: frames changed by owner, speed at the curve entries (from E), frames inside crossing-owner windows
    for name, (cfg, mod) in cfgs.items():
        print('\n2-%s. PWM-changed frames by steering owner (crossing owners = arc2_flat / arc2_blind / crossing_pass), and the PWM there' % name)
        by = collections.Counter(); dp = collections.defaultdict(list); rz = collections.defaultdict(list)
        for run in RUNS:
            A = emul.run(evalc.CAR, str(EXP / run)); B = emul.run(cfg, str(EXP / run), module=mod); O = raw_owner(run)
            for a, b, o in zip(A, B, O):
                if not isinstance(a['new_pwm'], (int, float)) or not isinstance(b['new_pwm'], (int, float)): continue
                if abs(a['new_pwm'] - b['new_pwm']) > 1e-6:
                    by[o[0]] += 1; dp[o[0]].append(b['new_pwm'] - a['new_pwm'])
                    zn = str((b['new_ls'] or {}).get('zone') or '')
                    if zn.startswith('A ramp'): rz[run].append((a['since'], a['new_pwm'], b['new_pwm']))
        print('   ' + ', '.join('%s %d (mean +%.0f PWM)' % (k, v, np.mean(dp[k])) for k, v in by.most_common(10)))
        if any(rz.values()):
            allr = [x for v in rz.values() for x in v]
            print('   ramp zone (RAMP+0.3..1.9, allow_climb) frames changed: %d ; PWM old mean %.1f -> new mean %.1f, max increase %+.1f ; positions RAMP+%.2f .. +%.2f m'
                  % (len(allr), np.mean([x[1] for x in allr]), np.mean([x[2] for x in allr]), max(x[2] - x[1] for x in allr), min(x[0] for x in allr), max(x[0] for x in allr)))
    # 3. the frames with real trouble inside the zones and what B2 does on them
    print('\n3. In-zone frames with |e| > 0.40 or |gz_lp| > 25 dps (three runs) and the PWM around them: recorded / car config emulated / B2')
    cfg, mod = cfgs['B2']
    for run in RUNS:
        A = emul.run(evalc.CAR, str(EXP / run)); B = emul.run(cfg, str(EXP / run), module=mod)
        t = np.array([r['t'] for r in A]); glp = evalc.lp_abs_gz(t, [r['gz'] for r in A]); last = -9.
        for i, r in enumerate(A):
            if evalc.zone100(r) is None or r['lap'] is None or r['lap'] < 1 or r['e'] is None: continue
            if (abs(r['e']) > 0.40 or (np.isfinite(glp[i]) and glp[i] > 25.)) and r['t'] - last > 0.5:
                last = r['t']; j = min(len(A) - 1, int(np.searchsorted(t, r['t'] + 0.6)))
                tw = (B[i]['new_ls'] or {}).get('two_level') or {}
                print('   %s %s%+.2f  e %+.2f gz_lp %4.1f | PWM now: recorded %3.0f, old %3.0f, B2 %3.0f (%s) | min in the next 0.6 s: old %3.0f, B2 %3.0f | max |e| next 0.6 s %.2f'
                      % (run[4:8], r['anchor'], r['since'], r['e'], glp[i], r['rec_pwm'], r['new_pwm'], B[i]['new_pwm'], tw.get('state') or (B[i]['new_ls'] or {}).get('blocked'),
                         min(x['new_pwm'] for x in A[i:j + 1]), min(x['new_pwm'] for x in B[i:j + 1]), max(abs(x['e']) for x in A[i:j + 1] if x['e'] is not None)))
    # 4. is the present slow-down damping a weave? gz sd in the zones: passes with many blocked frames vs passes with none
    print('\n4. Do the present slow-downs damp a weave? Per zone pass: number of blocked frames vs the yaw-rate spread and the error of that pass (recorded)')
    rows = collections.defaultdict(list)
    for run in RUNS:
        F = classify(load(run)); groups = collections.OrderedDict()
        for i, f in enumerate(F):
            if f['zone'] and f['lap'] is not None and f['lap'] >= 1: groups.setdefault((f['zone'], f['lap']), []).append(f)
        for (z, lap), P in groups.items():
            if len(P) < 40: continue
            gz = np.array([f['gz'] if f['gz'] is not None else np.nan for f in P]); e = np.array([abs(f['e']) if f['e'] is not None else np.nan for f in P])
            rows[z].append((sum(f['cls'].startswith('BLOCK') for f in P), float(np.nanstd(gz)), float(np.nanpercentile(e, 90)), float(np.nanmean([f['pwm'] or 0 for f in P]))))
    for z, _, _ in ZONES:
        a = np.array(rows[z]); cl = a[a[:, 0] == 0]; bl = a[a[:, 0] > 0]
        print('  %-6s passes without a blocked frame: n %d, gz sd %.1f dps, |e| P90 %.2f, mean PWM %.0f | passes with blocked frames: n %d (median %d frames), gz sd %.1f dps, |e| P90 %.2f, mean PWM %.0f | corr(blocked frames, gz sd) %.2f'
              % (z, len(cl), np.mean(cl[:, 1]) if len(cl) else float('nan'), np.mean(cl[:, 2]) if len(cl) else float('nan'), np.mean(cl[:, 3]) if len(cl) else float('nan'),
                 len(bl), np.median(bl[:, 0]) if len(bl) else 0, np.mean(bl[:, 1]) if len(bl) else float('nan'), np.mean(bl[:, 2]) if len(bl) else float('nan'), np.mean(bl[:, 3]) if len(bl) else float('nan'),
                 np.corrcoef(a[:, 0], a[:, 1])[0, 1] if len(a) > 2 else float('nan')))


if __name__ == '__main__':
    main()
