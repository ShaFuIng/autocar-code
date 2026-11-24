"""C2. open-loop replay of the on configs: what land4 would have sent on the recorded passes (the car's recorded motion, camera, IMU and encoder
are the inputs; the car does not react to the new command) against what the car sent.
  python eval_open.py <replay out dir>      (folders off / step0_shadow / step1_slope / step2_slope_land / step3_early_c4band / step3_only_json)"""
import json, sys, statistics as S
from pathlib import Path
from desc import *

OUT = Path(sys.argv[1])


def load(tag, run):
    p = OUT / tag / ('%s.new.jsonl' % run)
    if not p.exists(): return None, {}
    with open(p, encoding='utf8') as f:
        meta = json.loads(f.readline()); rows = [json.loads(l) for l in f]
    return meta, {r['frame_seq']: r for r in rows}


R = all_desc()
REP = {tag: {run: load(tag, run) for run in RUNS} for tag in ('off', 'step0_shadow', 'step1_slope', 'step2_slope_land', 'step3_early_c4band', 'step3_only_json')}


def rr(tag, d, n):
    return REP[tag][d['run']][1].get(g(d['F'][n], '_v', 'frame_seq'))


def pct(v, q): v = sorted(v); return v[min(len(v) - 1, int(q * len(v)))]


# ---- 0. shadow mode changes no command
tot = same = 0
for run in RUNS:
    a = REP['off'][run][1]; b = REP['step0_shadow'][run][1]
    for k, r in a.items():
        q = b.get(k)
        if q is None: continue
        tot += 1; same += (r['sent_steer'] == q['sent_steer'] and r['pwm'] == q['pwm'])
print('0. shadow mode (step0_shadow: track_only, both stages off): %d frames, dispatched steering + PWM identical to the off replay on %d' % (tot, same))
nsh = sum(1 for run in RUNS for r in REP['step0_shadow'][run][1].values() if 'shadow_us' in (r['proposal'].get('land4') or {}))
print('   frames with a shadow command logged: %d' % nsh)

# ---- 1. PWM never changes in any on config
for tag in ('step1_slope', 'step2_slope_land', 'step3_early_c4band', 'step3_only_json'):
    tot = same = st = 0
    for run in RUNS:
        a = REP['off'][run][1]; b = REP[tag][run][1]
        for k, r in a.items():
            q = b.get(k)
            if q is None: continue
            tot += 1; same += r['pwm'] == q['pwm']; st += r['sent_steer'] != q['sent_steer']
    print('1. %-20s frames %d | PWM identical %d | dispatched steering changed on %d frames' % (tag, tot, same, st))

# ---- 2. where steering changes (by zone), step configs vs off
print('\n2. frames whose dispatched steering differs from the off replay, by position (all 24 passes); outside = not within mouth-0.3 s .. landing+3.5 m of any pass')
inzone = {}
for d in R:
    for n, p in enumerate(d['F']):
        im = d['i_mouth'] - d['a']
        if d['t'][n] < d['t'][im] - 0.3: continue
        x = d['x'][n] if d.get('x') else None
        z = 'slope' if (d['i_land'] is None or n < d['i_land'] - d['a']) else ('x<1.2' if x < 1.2 else ('1.2-1.5' if x < 1.5 else ('1.5-2.8' if x < 2.8 else ('2.8-3.5' if x < 3.5 else None))))
        if z: inzone[(d['run'], g(p, '_v', 'frame_seq'))] = z
for tag in ('step1_slope', 'step2_slope_land', 'step3_early_c4band', 'step3_only_json'):
    cnt = {}
    for run in RUNS:
        a = REP['off'][run][1]; b = REP[tag][run][1]
        for k, r in a.items():
            q = b.get(k)
            if q is None or r['sent_steer'] == q['sent_steer']: continue
            z = inzone.get((run, k), 'outside'); cnt[z] = cnt.get(z, 0) + 1
    print('   %-20s %s' % (tag, cnt))

# ---- 3. slope mouth: the five false-error passes + all passes
print('\n3. slope mouth, first 0.5 s (pre-compensation command - centre): car log | land4 (step1 replay)')
print('   run lap  max e   | car: max / min / largest step | land4: max / min / largest step | land4 state')
for d in R:
    F = d['F']; im = d['i_mouth'] - d['a']
    w = [n for n in range(im, len(F)) if d['t'][n] - d['t'][im] <= 0.5]
    car = [F[n]['steer_us'] - C0 for n in w]; es = [F[n].get('image_error') for n in w[:8] if F[n].get('image_error') is not None]
    new = []
    for n in w:
        q = rr('step1_slope', d, n)
        if q is not None and q['proposal'].get('steer_us') is not None: new.append(q['proposal']['steer_us'] - C0)
    q0 = rr('step1_slope', d, w[min(3, len(w) - 1)]); l4 = (q0['proposal'].get('land4') or {}) if q0 else {}
    stc = max(abs(b - a) for a, b in zip(car, car[1:])); stn = max(abs(b - a) for a, b in zip(new, new[1:])) if len(new) > 1 else float('nan')
    mark = '  <-- false error' if es and max(es) >= 0.9 else ''
    print('   %s %d  %5.2f  | %+5.0f / %+5.0f / %4.0f | %+5.0f / %+5.0f / %4.0f | %s %s%s' % (d['run'], d['lap'], max(es) if es else float('nan'), max(car), min(car), stc,
                                                                                   max(new) if new else float('nan'), min(new) if new else float('nan'), stn, l4.get('state'), l4.get('reason', ''), mark))

# ---- 4. frame-by-frame at the 46c9 lap 2 mouth
print('\n4. 46c9 lap 2 (the failure), frames from the mouth: pitch, camera error | car pre-comp command (owner) | land4 step1 command (open loop: the car had already been turned by the real command)')
d = [x for x in R if x['run'] == '46c9' and x['lap'] == 2][0]; im = d['i_mouth'] - d['a']
for n in range(im - 1, im + 14):
    p = d['F'][n]; q = rr('step1_slope', d, n); l4 = (q['proposal'].get('land4') or {}) if q else {}
    print('   %+.2f s pitch %6.1f e %5s | car %7.1f (%s%s) | land4 %s psi %s th %s cam %s -> %s' % (d['t'][n] - d['t'][im], pitch(p) or 0, 'None' if p.get('image_error') is None else '%.2f' % p['image_error'], p['steer_us'], p.get('owner'),
          ', jump_guard hold' if 'held_us' in (p.get('jump_guard') or {}) else '', l4.get('state'), l4.get('psi'), l4.get('th'), l4.get('cam_deg'), l4.get('steer_us')))

# ---- 5. landing: car vs land4 by x
print('\n5. landing (23 passes), pre-compensation command - centre at x (m since the landing): car log | land4 step2 replay (open loop)')
XS = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4)
print('   run lap | ' + ' '.join('x%.1f car/new' % x for x in XS) + ' | psi-th at 0 / 0.5 / 1.0 | cam deg min..max | glitch deg | largest step car / new (x<=1.5) | online x err max (m)')
agg = {x: ([], []) for x in XS}; steps = ([], []); xerrs = []; gl = []; hd = {0.0: [], 0.5: [], 1.0: []}
for d in R:
    if d['i_land'] is None: continue
    F = d['F']; il = d['i_land'] - d['a']; row = []
    seq = [(n, d['x'][n], F[n]['steer_us'] - C0, rr('step2_slope_land', d, n)) for n in range(il, len(F)) if d['x'][n] is not None and d['x'][n] <= 1.6]
    for x in XS:
        c = min(seq, key=lambda s: abs(s[1] - x)); q = c[3]
        nv = (q['proposal']['steer_us'] - C0) if q else float('nan')
        row.append('%+4.0f/%+4.0f' % (c[2], nv)); agg[x][0].append(c[2]); agg[x][1].append(nv)
    l4s = [(s[1], (s[3]['proposal'].get('land4') or {})) for s in seq if s[3]]
    def he(x):
        c = min(l4s, key=lambda s: abs(s[0] - x))[1]
        return None if c.get('psi') is None else c['psi'] - c['th']
    h = [he(0.0), he(0.5), he(1.0)]
    for k_, v_ in zip((0.0, 0.5, 1.0), h):
        if v_ is not None: hd[k_].append(v_)
    cams = [l.get('cam_deg') for _, l in l4s if l.get('cam_deg') is not None]
    gg = [l.get('glitch_deg') for _, l in l4s if l.get('glitch_deg') is not None]; gl.append(gg[-1] if gg else 0.)
    car = [s[2] for s in seq if s[1] <= 1.5]; new = [s[3]['proposal']['steer_us'] - C0 for s in seq if s[3] and s[1] <= 1.5]
    sc = max(abs(b - a) for a, b in zip(car, car[1:])); sn = max(abs(b - a) for a, b in zip(new, new[1:])); steps[0].append(sc); steps[1].append(sn)
    xe = [abs(l['x_m'] - x) for x, l in l4s if l.get('x_m') is not None and l.get('state') == 'LAND']; xerrs += xe
    print('   %s %d | %s | %s | %s | %+.1f | %3.0f / %3.0f | %.2f' % (d['run'], d['lap'], ' '.join(row), ' / '.join('None' if v is None else '%+.1f' % v for v in h),
                                                                '%+.1f..%+.1f' % (min(cams), max(cams)) if cams else 'None', gl[-1], sc, sn, max(xe) if xe else float('nan')))
print('   median  | ' + ' '.join('%+4.0f/%+4.0f' % (S.median(agg[x][0]), S.median(agg[x][1])) for x in XS))
print('   sd      | ' + ' '.join('%4.0f/%4.0f' % (S.pstdev(agg[x][0]), S.pstdev(agg[x][1])) for x in XS))
print('   largest frame-to-frame step x<=1.5: car median %.0f max %.0f us | land4 median %.0f max %.0f us' % (S.median(steps[0]), max(steps[0]), S.median(steps[1]), max(steps[1])))
print('   heading error psi - th of the RECORDED trajectories against the table (deg): ' + ' | '.join('x %.1f: mean %+.1f sd %.1f' % (k_, S.mean(v_), S.pstdev(v_)) for k_, v_ in hd.items()))
print('   online x (encoder + speed extrapolation) minus the offline interpolated x: |err| median %.3f P90 %.3f max %.3f m (n %d)' % (S.median(xerrs), pct(xerrs, .9), max(xerrs), len(xerrs)))
print('   passes with a yaw glitch removed: %d of %d (%s)' % (sum(1 for x in gl if abs(x) > 0.1), len(gl), ', '.join('%+.1f' % x for x in gl if abs(x) > 0.1)))

# ---- 6. step 3: curve-4 band from 1.2 m
print('\n6. step 3 (curve-4 band from 1.2 m, arc2 landing band ends 1.2..1.5 m): pre-comp command - centre in x bins, median [P10..P90] over the 23 landings')
print('   x bin      car log            step2 replay       step3 replay       step3_only_json replay | band state ON share (step3) | largest step car / step2 / step3')
for lo_, hi_ in ((1.0, 1.2), (1.2, 1.5), (1.5, 1.9), (1.9, 2.3), (2.3, 2.8), (2.8, 3.5)):
    col = {k: [] for k in ('car', 'step2_slope_land', 'step3_early_c4band', 'step3_only_json')}; on = [0, 0]; stp = {k: [] for k in ('car', 'step2_slope_land', 'step3_early_c4band')}
    for d in R:
        if d['i_land'] is None: continue
        F = d['F']; il = d['i_land'] - d['a']; prev = {}
        for n in range(il, len(F)):
            x = d['x'][n]
            if x is None or not (lo_ <= x < hi_): continue
            col['car'].append(F[n]['steer_us'] - C0)
            if 'car' in prev: stp['car'].append(abs(F[n]['steer_us'] - C0 - prev['car']))
            prev['car'] = F[n]['steer_us'] - C0
            for tag in ('step2_slope_land', 'step3_early_c4band', 'step3_only_json'):
                q = rr(tag, d, n)
                if q is None or q['proposal'].get('steer_us') is None: continue
                v = q['proposal']['steer_us'] - C0; col[tag].append(v)
                if tag in stp:
                    if tag in prev: stp[tag].append(abs(v - prev[tag]))
                    prev[tag] = v
                if tag == 'step3_early_c4band':
                    on[1] += 1; on[0] += g(q['proposal'], 'lap_ff4', 'band', 'state') == 'ON'
    f = lambda v: '%+4.0f [%+4.0f..%+4.0f]' % (S.median(v), pct(v, .1), pct(v, .9)) if v else 'None'
    print('   %.1f-%.1f  %s  %s  %s  %s | %3.0f%% | %3.0f / %3.0f / %3.0f' % (lo_, hi_, f(col['car']), f(col['step2_slope_land']), f(col['step3_early_c4band']), f(col['step3_only_json']), 100. * on[0] / max(1, on[1]),
                                                                max(stp['car'] or [0]), max(stp['step2_slope_land'] or [0]), max(stp['step3_early_c4band'] or [0])))
# band first ON position in step 3
v = []
for d in R:
    if d['i_land'] is None: continue
    for n in range(d['i_land'] - d['a'], len(d['F'])):
        q = rr('step3_early_c4band', d, n)
        if q and g(q['proposal'], 'lap_ff4', 'band', 'state') == 'ON' and (g(q['proposal'], 'lap_ff4', 'band', 'w') or 0) >= 1.: v.append(d['x'][n]); break
print('   step 3: curve-4 band fully on (w = 1) at x %.2f..%.2f m (median %.2f, n %d); car today 2.63..2.94 (median 2.79)' % (min(v), max(v), S.median(v), len(v)))
v = []
for d in R:
    if d['i_land'] is None: continue
    for n in range(d['i_land'] - d['a'], len(d['F'])):
        q = rr('step3_early_c4band', d, n)
        if q and g(q['proposal'], 'land4', 'state') == 'END': v.append(d['x'][n]); break
print('   step 3: land4 handed over (END) at x %.2f..%.2f m (median %.2f, n %d)' % (min(v), max(v), S.median(v), len(v)))
# land4 states per pass
print('\n7. land4 pass states in the step2 replay: ', end='')
st = {}
for d in R:
    q = rr('step2_slope_land', d, d['i_mouth'] - d['a'] + 5); l = (q['proposal'].get('land4') or {}) if q else {}
    k = l.get('state', 'none') + ('/' + l['reason'] if l.get('reason') else ''); st[k] = st.get(k, 0) + 1
print(st)
n_other = 0
for run in RUNS:
    for k, r in REP['step2_slope_land'][run][1].items():
        if 'land4' in r['proposal'] and (run, k) not in inzone: n_other += 1
print('   frames with a land4 record outside the 24 slope passes (mouth-0.3 s .. landing+3.5 m): %d' % n_other)
