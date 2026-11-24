"""Per-run field summary (11-19). usage (cwd = experiments): python field-20261119/an.py run-8545-20261119 run-c212-20261118 ..."""
import sys, json, numpy as np, collections
sys.path.insert(0, 'field-20261118')
from ld import load
gz = lambda p: (p.get('inputs') or {}).get('gz') or 0.
sent = lambda p: (p.get('play_comp') or {}).get('steer_us') or p.get('steer_us') or 0.
err = lambda p: p.get('image_error')
C = float(__import__("os").environ.get("CEN","1510"))


def speed(Q):
    t = np.array([p['t'] for p in Q]); o = np.array([(p.get('lap_pos') or {}).get('odo_m') or 0. for p in Q])
    v = np.zeros(len(Q)); j = 0
    for i in range(len(Q)):
        while t[i] - t[j] > 0.6: j += 1
        v[i] = (o[i] - o[j]) / (t[i] - t[j]) if t[i] > t[j] else 0.
    return v


def dur(P, f):
    if len(P) < 2: return 0.
    dt = np.diff([p['t'] for p in P]); return float(sum(d for d, p in zip(dt, P[1:]) if f(p) and d < 0.3))


def main(run):
    Q = load(run); v = speed(Q)
    for p, x in zip(Q, v): p['_v_ms'] = x
    c1 = [p['t'] for a, p in zip(Q, Q[1:]) if p['_s'] == 'curve1' and a['_s'] != 'curve1']
    print('=====', run, 'frames', len(Q), 'curve-1 laps', [round(b - a, 1) for a, b in zip(c1, c1[1:])])
    pw = np.array([p.get('pwm') or 0 for p in Q]); print('  pwm mean %.1f  >=50 %.0f%%  <=28 %.0f%%' % (pw.mean(), (pw >= 49.5).mean() * 100, (pw <= 28).mean() * 100))
    G = collections.defaultdict(list)
    for p in Q: G[(p['_lap'], p['_s'])].append(p)
    laps = sorted({k[0] for k in G if k[0] >= 1})
    full = [l for l in laps if l < max(laps)] or laps
    print('  segment median duration s / mean pwm / mean speed (full laps %s)' % full)
    tot = 0
    for s in ['curve1', 'diagonal_v', 'curve2', 'circle_straight', 'left_bend', 'curve3', 'ramp', 'top', 'dashed_turn', 'upper_straight', 'downhill', 'curve4', 'main_straight']:
        d = [G[(l, s)][-1]['t'] - G[(l, s)][0]['t'] for l in full if G.get((l, s))]
        if not d: continue
        P = [p for l in full for p in G.get((l, s), [])]; tot += np.median(d)
        print('    %-16s %5.2f  pwm %4.1f  v %.2f  [%s]' % (s, np.median(d), np.mean([p.get('pwm') or 0 for p in P]), np.mean([p['_v_ms'] for p in P]), ' '.join('%.1f' % x for x in d)))
    print('    sum of medians %.1f' % tot)
    # ---- ramp
    print('  RAMP per lap: min speed on ramp+top(first 1.5 m) | first RAMP-relative position where pwm<38 | pwm at crest (top first 0.5 s) | gz sd')
    for l in laps:
        R = G.get((l, 'ramp'), []); T = G.get((l, 'top'), [])
        if not R: continue
        t0 = T[0]['t'] if T else R[-1]['t']; Tf = [p for p in T if p['t'] - t0 < 1.5]
        lo = [((p.get('lap_pos') or {}).get('since_anchor_m'), p.get('pwm')) for p in R if (p.get('pwm') or 99) < 38 and ((p.get('lap_pos') or {}).get('since_anchor_m') or 0) > 0.6]
        sa = [((p.get('speed_adapt') or {}).get('reason')) for p in R]
        g = np.array([gz(p) for p in R])
        print('    lap %d  vmin %.2f  ramp %.1fs len %.2f m  first<38 %s  crest pwm %s  gz sd %.1f  special %d' % (l, min(p['_v_ms'] for p in R[len(R) // 3:] + Tf), R[-1]['t'] - R[0]['t'], (R[-1]['lap_pos'].get('since_anchor_m') or 0), lo[0] if lo else None, ['%.0f' % (p.get('pwm') or 0) for p in Tf[:12:3]], g.std(), sum(1 for x in sa if x and 'SPECIAL' in x)))
    # ---- landing
    print('  LANDING per lap: sent-centre in [-0.45,-0.15] s (median/min) | gz at landing | e>=0.95 s within 2.5 s | min e 0.8-2.5 s | approach applied frames | invalid frames +-1 s | sent<=c-100 frames before landing')
    for l in laps:
        D = G.get((l, 'downhill'), [])
        if not D: continue
        tl = D[-1]['t']; W = [p for p in Q if tl - 1.0 <= p['t'] <= tl + 2.5]
        pre = [sent(p) - C for p in W if -0.45 <= p['t'] - tl <= -0.15]
        g0 = np.mean([gz(p) for p in W if -0.05 <= p['t'] - tl <= 0.3])
        post = [p for p in W if p['t'] > tl]
        if len(post) < 20: continue
        ap = [p for p in W if (p.get('arc2_land') or {}).get('state') == 'APPROACH']
        apa = sum(1 for p in ap if p['arc2_land'].get('applied'))
        em = [err(p) for p in post if p['t'] - tl > 0.8 and err(p) is not None]
        print('    lap %d  pre %s/%s  gz0 %+.0f  e>=.95 %.2fs  emin %s  approach %d/%d  invalid %d  low %d  sent peak %.0f  gz min %.0f' % (
            l, '%+.0f' % np.median(pre) if pre else '-', '%+.0f' % min(pre) if pre else '-', g0, dur(post, lambda p: (err(p) or 0) >= 0.95), '%.2f' % min(em) if em else '-', apa, len(ap),
            sum(1 for p in W if abs(p['t'] - tl) < 1 and not p.get('valid')), sum(1 for p in W if p['t'] < tl and sent(p) - C <= -100), max(sent(p) for p in post), min(gz(p) for p in post)))
    # ---- V window
    print('  V CROSSING (arc2_blind) per lap: states | how | dpsi range | e range in C1e+4.8..7.8 | gz range | steer range | invalid')
    for l in laps:
        P = [p for p in G.get((l, 'diagonal_v'), []) if p.get('arc2_blind')]
        if not P: continue
        st = []
        for p in P:
            s = p['arc2_blind'].get('state')
            if not st or st[-1] != s: st.append(s)
        H = [p for p in P if p['arc2_blind'].get('applied')]
        W = [p for p in G[(l, 'diagonal_v')] if 4.8 <= ((p.get('lap_pos') or {}).get('since_anchor_m') or -1) <= 7.8]
        e = [err(p) for p in W if err(p) is not None]
        dp = [p['arc2_blind'].get('dpsi') for p in H if p['arc2_blind'].get('dpsi') is not None]
        print('    lap %d  %s  how %s  dpsi %s  e %s mean %s  gz %s  sent %s  invalid %d  v %.2f' % (l, '>'.join(str(x) for x in st)[:60], sorted({str(p['arc2_blind'].get('how')) for p in H}),
              '%+.1f..%+.1f' % (min(dp), max(dp)) if dp else '-', '%+.2f..%+.2f' % (min(e), max(e)) if e else '-', '%+.2f' % np.mean(e) if e else '-',
              '%+.0f..%+.0f' % (min(gz(p) for p in W), max(gz(p) for p in W)) if W else '-', '%.0f..%.0f' % (min(sent(p) for p in W), max(sent(p) for p in W)) if W else '-', sum(1 for p in W if not p.get('valid')), np.mean([p['_v_ms'] for p in W]) if W else 0))
    # ---- circle crossing r4
    print('  CIRCLE CROSSING r4 per lap: states | dpsi range | e range C2e+0..6 | invalid')
    for l in laps:
        P = [p for p in G.get((l, 'circle_straight'), []) if p.get('arc2_blind_c2')]
        if not P: continue
        st = []
        for p in P:
            s = p['arc2_blind_c2'].get('state')
            if not st or st[-1] != s: st.append(s)
        dp = [p['arc2_blind_c2'].get('dpsi') for p in P if p['arc2_blind_c2'].get('applied') and p['arc2_blind_c2'].get('dpsi') is not None]
        W = G[(l, 'circle_straight')]; e = [err(p) for p in W if err(p) is not None]
        print('    lap %d  %s  dpsi %s  e %s  invalid %d' % (l, '>'.join(str(x) for x in st)[:50], '%+.1f..%+.1f' % (min(dp), max(dp)) if dp else '-', '%+.2f..%+.2f' % (min(e), max(e)) if e else '-', sum(1 for p in W if not p.get('valid'))))
    # ---- curves
    for cv in ('curve1', 'curve2', 'curve3', 'curve4'):
        print('  %s per lap: dur | e>=0.95 s | sent>=2000 s | sent max | e median | e min | gz sd | invalid | v' % cv.upper())
        for l in laps:
            P = G.get((l, cv), [])
            if len(P) < 20: continue
            e = [err(p) for p in P if err(p) is not None]; ex = ''
            if cv == 'curve2':
                B = [p for p in P if ((p.get('lap_ff') or {}).get('band') or {}).get('turned_deg') is not None]
                deep = [err(p) for p in B if 110 <= p['lap_ff']['band']['turned_deg'] <= 160 and err(p) is not None]
                fl = [p for p in B if 110 <= p['lap_ff']['band']['turned_deg'] <= 160]
                ex = '  deep(110-160) e med %s  pre-comp med %s' % ('%.2f' % np.median(deep) if deep else '-', '%.0f' % np.median([p.get('steer_us') or 0 for p in fl]) if fl else '-')
            print('    lap %d  %.1fs  %.2f  %.2f  %.0f  %+.2f  %+.2f  %.1f  %d  %.2f%s' % (l, P[-1]['t'] - P[0]['t'], dur(P, lambda p: (err(p) or 0) >= 0.95), dur(P, lambda p: sent(p) >= 2000), max(sent(p) for p in P),
                  np.median(e) if e else 0, min(e) if e else 0, np.std([gz(p) for p in P]), sum(1 for p in P if not p.get('valid')), np.mean([p['_v_ms'] for p in P]), ex))
    # ---- straights
    print('  STRAIGHTS gz sd (1 s high-pass) per lap: main(after +3 m) | diagonal | circle')
    for l in laps:
        out = []
        for s, m in (('main_straight', 3.0), ('diagonal_v', 0.8), ('circle_straight', 0.5)):
            P = [p for p in G.get((l, s), []) if ((p.get('lap_pos') or {}).get('since_anchor_m') or 0) >= m]
            if len(P) < 20: out.append('  - '); continue
            t = np.array([p['t'] for p in P]); g = np.array([gz(p) for p in P])
            hp = np.array([g[i] - g[(t > t[i] - .5) & (t < t[i] + .5)].mean() for i in range(len(t))]); out.append('%4.1f' % hp.std())
        print('    lap %d  %s' % (l, '  '.join(out)))
    # ---- dropouts
    drops = []; i = 0
    while i < len(Q):
        if not Q[i].get('valid'):
            j = i
            while j + 1 < len(Q) and not Q[j + 1].get('valid'): j += 1
            if Q[j]['t'] - Q[i]['t'] >= 0.25: drops.append((round(Q[i]['t'], 1), round(Q[j]['t'] - Q[i]['t'], 2), Q[i]['_lap'], Q[i]['_s']))
            i = j + 1
        else: i += 1
    print('  DROPOUTS >=0.25 s:', len(drops), drops)
    lost = sum(1 for p in Q if (p.get('lap_pos') or {}).get('lost')); print('  lap_pos lost frames', lost)


for r in sys.argv[1:]: main(r)
