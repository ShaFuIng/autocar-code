"""c3refit unit tests (laptop only): candidate/lap_ff.py (x-band-r2, 6b3632f1) against the car's lap_ff.py (adcde919 = c3c4/candidate) and the on/ files.
     python -B tests/test_c3refit.py"""
import copy, hashlib, importlib.util, json, math, random, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent; ROOT = HERE.parent; EXP = ROOT.parents[1]
SNAP = EXP / 'orin-web-snapshot-20261122'; CAR = EXP / 'field-20261122' / 'c3c4'; ON = ROOT / 'on'; CAND = ROOT / 'candidate'
sys.path.insert(0, str(SNAP)); sys.path.insert(0, str(ROOT / 'tools')); sys.dont_write_bytecode = True


def mod(name, path):
    sp = importlib.util.spec_from_file_location(name, path); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m); return m


OLD = mod('lap_ff_car', CAR / 'candidate' / 'lap_ff.py'); NEW = mod('lap_ff_new', CAND / 'lap_ff.py')
J = lambda p: json.loads(Path(p).read_text(encoding='utf8'))
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()[:8]
N = 0
CAR_ARC2 = CAR / 'on' / 'c3_own_step1_in10' / 'arc2.json'; CAR_SHADOW = CAR / 'on' / 'c3_own_step0_shadow' / 'arc2.json'; CAR_LFF = ROOT / 'tools' / 'base' / 'lap_ff.car_052be3dc.json'


def ok(c, msg):
    global N; N += 1
    assert c, msg


def diff(a, b, pre=''):
    out = []
    for k in sorted(set(a) | set(b)):
        if a.get(k) == b.get(k): continue
        if isinstance(a.get(k), dict) and isinstance(b.get(k), dict): out += diff(a[k], b[k], pre + k + '.')
        else: out.append(pre + k)
    return out


def c3_pass(M, arc2, seed, near_of=None, pid_of=None, stair=True, y_of=None, turn_scale=1., v=1.0, al=True, x_end=7.0):
    """one synthetic curve-3 pass through the arc2 c3 block with a 5 Hz odometer staircase, a lap_pos-like heading history and arc_lat y of the previous frame.
    -> list of dict(out, info, st, near, x (true distance), xs (odometer), t, h (true heading since the anchor step))"""
    raw = arc2 if isinstance(arc2, dict) else J(arc2); c3 = M.load_block(raw.get('c3'), M.load_config(CAR_LFF)); F = M.LapFF4(c3, 'c3'); rng = random.Random(seed); out = []
    Dt = 200.; yaw = 0.; t = 10.; hist = []; D = Dt; tick = 0.; d0 = None; al_prev = None; h0 = None
    while True:
        t += 1 / 30.; Dt += v / 30.
        if t - tick >= 0.2: tick = t; D = Dt
        if not stair: D = Dt
        hist.append((D, yaw))
        if d0 is None and Dt >= 203.: d0 = D; h0 = [y for d, y in hist if d == D][0]   # anchor = this odometer value; heading at the first frame carrying it
        x = (Dt - d0) if d0 is not None else -1.
        xs = (D - d0) if d0 is not None else -1.
        if d0 is not None and Dt - d0 > x_end: break
        g_ = -turn_scale * (30. + 15. * math.sin(max(0., x) * 2.0)) * v + rng.uniform(-2, 2); yaw += g_ / 30.
        near = (0.35 + 0.2 * math.sin(x * 1.7 + seed) + rng.uniform(-.05, .05)) if near_of is None else near_of(x)

        def yaw_at(d, H=list(hist), ynow=yaw):
            for i in range(len(H) - 1, 0, -1):
                if H[i - 1][0] <= d <= H[i][0]:
                    (a0, y0), (a1, y1) = H[i - 1], H[i]
                    return y0 if a1 <= a0 else y0 + (y1 - y0) * (d - a0) / (a1 - a0)
            return ynow
        lp = dict(lap=1, anchor='C3s' if d0 is not None else 'C2e', anchor_d=d0 if d0 is not None else 190., since_anchor_m=xs if d0 is not None else D - 190., lost=False, odo_m=D, t=t, yaw_imu=hist[-1][1], yaw_at=yaw_at, pitch=0.5, gz_imu=g_)
        if al: lp['al'] = al_prev
        st = (1475. + 40. + 220. * near + rng.uniform(-30, 30)) if pid_of is None else pid_of(x)
        o, i = F.step(lp, near, True, v, hist[-1][1], st, 1475., 780., 2050., True, now=t)
        out.append(dict(out=o, info=i, st=st, near=near, x=x, xs=xs, t=t, h=(h0 - hist[-1][1]) if h0 is not None else None, F=F))
        al_prev = (t, None if y_of is None else y_of(x))
    return out


def xbi(q): return (q['info'] or {}).get('xband')


def test_hashes():
    ok(sha(CAND / 'lap_ff.py') == '6b3632f1' and sha(CAND / 'live_pid.py') == 'b3267293', 'candidate hashes %s %s' % (sha(CAND / 'lap_ff.py'), sha(CAND / 'live_pid.py')))
    ok(sha(CAR / 'candidate' / 'lap_ff.py') == 'adcde919' and sha(CAR / 'candidate' / 'live_pid.py') == '3b74ddc4' and sha(CAR_ARC2) == '2b81e9b2' and sha(CAR_LFF) == '052be3dc', 'car base hashes')


def test_patches():
    for scr, base, outp in (('patch_r2.py', CAR / 'candidate' / 'lap_ff.py', CAND / 'lap_ff.py'), ('patch_live_r2.py', CAR / 'candidate' / 'live_pid.py', CAND / 'live_pid.py')):
        r = subprocess.run([sys.executable, '-B', str(CAND / scr), '--check', str(base), str(outp)], capture_output=True, text=True); ok(r.returncode == 0 and 'OK' in r.stdout, scr + ' does not reproduce: ' + r.stdout + r.stderr)
    a = (CAR / 'candidate' / 'live_pid.py').read_text(encoding='utf8').splitlines(); b = (CAND / 'live_pid.py').read_text(encoding='utf8').splitlines()
    ok(len(b) == len(a) + 1 and sum(1 for q in b if q not in a) == 2, 'live_pid: expected one changed and one added line')


def test_key_absent():
    """no r2 key: the new module = the car's module value for value, with the car's own x-band (owning and shadow) and with no x-band."""
    n = 0
    for arc2 in (CAR_ARC2, CAR_SHADOW, SNAP / 'arc2.json'):
        for seed in range(4):
            for kw in (dict(), dict(stair=False), dict(near_of=lambda x: 0.95 if 2.0 < x < 2.6 else 0.3), dict(al=False)):
                a = c3_pass(OLD, arc2, seed, **kw); b = c3_pass(NEW, arc2, seed, **kw)
                ok(len(a) == len(b) and all(p['out'] == q['out'] and p['info'] == q['info'] for p, q in zip(a, b)), 'key absent differs: %s seed %d %s' % (Path(arc2).parent.name, seed, list(kw)))
                n += len(a)
    F = c3_pass(NEW, CAR_ARC2, 0)[-1]['F']; ok(F.xb_on and F.xb.get('_r2') is False and F.xb_dr is False, 'r1 table took the r2 path')
    return n


def test_configs():
    car = J(CAR_ARC2)
    for step, shadow in (('step0_shadow', True), ('step1_own_conservative', False), ('step2_own_standard', False), ('step3_own_prerotated', False)):
        c = J(ON / step / 'arc2.json'); X = c['c3']['xband']
        ok(diff(car, c) and all(d.startswith('c3.xband') for d in diff(car, c)), step + ' differs outside c3.xband: %s' % diff(car, c))
        ok(bool(X.get('shadow')) == shadow and X['enabled'] is True and X['x_dr'] is True, step + ' flags')
        ok(sha(ON / step / 'curve_trim.json') == '40078635', step + ' curve_trim')
        F = c3_pass(NEW, c, 0, x_end=1.0)[-1]['F']; ok(F.xb_on and F.xb['_r2'] and F.xb_dr, step + ' not loaded as r2')
        u = [q[1] for q in X['u_x']]; ok(max(u) <= 200. and min(u) >= 40. and max(abs(a - b) for a, b in zip(u, u[1:])) <= 35., step + ' feed-forward not smooth: %s' % u)
        h = [q[1] for q in X['h_x']]; ok(all(b > a for a, b in zip(h, h[1:])), step + ' h_x not increasing')
        ok(X['w_x'] == [[0.6, 0.0], [0.9, 1.0], [4.05, 1.0], [4.35, 0.0]], step + ' w_x')
    ok(sha(ON / 'rollback_r1_shadow_f4573960' / 'arc2.json') == 'f4573960' and sha(ON / 'rollback_all_off_d61e3f58' / 'arc2.json') == 'd61e3f58' and sha(ON / 'rollback_car_now_2b81e9b2' / 'arc2.json') == '2b81e9b2', 'roll-back files')


def test_law():
    """the r2 target = the documented formula = tools/sim.py band_target (the model uses the same arithmetic as the code)."""
    import sim
    c = J(ON / 'step2_own_standard' / 'arc2.json'); X = c['c3']['xband']; X2 = dict(X, rate_us_s=0.)
    c2 = copy.deepcopy(c); c2['c3']['xband'] = X2
    P = c3_pass(NEW, c2, 3, y_of=lambda x: -8. + 3. * math.sin(x)); k = 0
    for q in P:
        xb = xbi(q)
        if xb is None or xb['own'] < 1.0: continue
        g = q['info']['g']; lpgz = None
        tgt, parts = sim.band_target(X2, xb['x_m'], q['near'], q['info']['v'], g, 0., xb['h_deg'], None, 1 / 30., xb['y_cm'])
        ok(abs(parts['prof'] - (xb['prof_us'] - 1490.)) < 0.11 or abs(parts['prof'] - (xb['prof_us'] - q['F'].c['_c_eff_us'])) < 0.11, 'feed-forward %s %s' % (parts['prof'], xb['prof_us']))
        ok(abs(parts['et'] - xb['e_us']) < 0.4 and abs(parts['ht'] - xb['h_us']) < 0.6 and abs(parts['yt'] - xb['y_us']) < 0.5, 'terms at x %.2f: %s %s' % (xb['x_m'], parts, xb))
        ok(abs(xb['out_us'] - xb['target_us']) < 0.06 and abs(xb['raw_us'] - xb['target_us']) < 0.06, 'own: out != target'); k += 1
        ok(abs(xb['h_deg'] - q['h']) < 1.5, 'heading since the anchor step: %s vs true %s' % (xb['h_deg'], q['h']))
    ok(k > 60, 'band never owned (%d)' % k)


def test_x_dr():
    c = J(ON / 'step2_own_standard' / 'arc2.json'); P = c3_pass(NEW, c, 1, y_of=lambda x: -8.)
    R = [(q['x'], q['xs'], xbi(q)) for q in P if xbi(q)]
    ok(len(R) > 80, 'no band frames')
    e_dr = [abs(xb['x_m'] - x) for x, xs, xb in R[8:]]; e_st = [abs(xs - x) for x, xs, xb in R[8:]]
    ok(max(e_dr) < 0.05 and max(e_st) > 0.15, 'dead reckoning: max error %.3f m (staircase %.3f m)' % (max(e_dr), max(e_st)))
    ok(all(abs(xb['xs_m'] - xs) < 0.002 for x, xs, xb in R), 'xs_m is not the staircase')
    d = [b[2]['x_m'] - a[2]['x_m'] for a, b in zip(R, R[1:])]; ok(max(d) < 0.08 and min(d) > -0.03, 'x jumps: %.3f .. %.3f' % (min(d), max(d)))
    c1 = copy.deepcopy(c); c1['c3']['xband']['x_dr'] = False
    R1 = [xbi(q) for q in c3_pass(NEW, c1, 1) if xbi(q)]; ok(all(abs(xb['x_m'] - xb['xs_m']) < 1e-9 for xb in R1), 'x_dr false still dead-reckons')
    # a stopped odometer: at most dr_max_s of speed is added
    F = NEW.LapFF4(NEW.load_block(c['c3'], NEW.load_config(CAR_LFF)), 'c3'); F._xd = [10., 5.0]
    A = dict(d_land=9.0); r = F._xb2(A, dict(odo_m=10., gz_imu=0.), 1.0, 0.3, True, 1.0, 0.16, 1500., 1475., 2050., 7.0, 30.)
    ok(abs(r['info']['x_m'] - 1.3) < 1e-6, 'dr_max_s cap: %s' % r['info']['x_m'])


def test_terms():
    c = J(ON / 'step2_own_standard' / 'arc2.json'); X = c['c3']['xband']
    lo = [xbi(q) for q in c3_pass(NEW, c, 2, turn_scale=0.8, y_of=lambda x: -8.) if xbi(q) and xbi(q)['own'] == 1.0]
    hi = [xbi(q) for q in c3_pass(NEW, c, 2, turn_scale=1.25, y_of=lambda x: -8.) if xbi(q) and xbi(q)['own'] == 1.0]
    ok(sum(q['h_us'] for q in lo) / len(lo) > 20. and sum(q['h_us'] for q in hi) / len(hi) < -20., 'heading term sign: under-turned %.0f, over-turned %.0f' % (sum(q['h_us'] for q in lo) / len(lo), sum(q['h_us'] for q in hi) / len(hi)))
    ok(all(abs(q['h_us']) <= X['h_lim_us'] + 0.06 for q in lo + hi), 'heading term limit')
    wild = [xbi(q) for q in c3_pass(NEW, c, 2, turn_scale=3.0, y_of=lambda x: -8.) if xbi(q) and xbi(q)['h_deg'] is not None and abs(xbi(q)['h_ref'] - xbi(q)['h_deg']) > 25.5]
    ok(len(wild) > 20 and all(q['h_us'] == 0. for q in wild), 'heading more than h_dev_max off the plan must give no heading term')
    ins = [xbi(q) for q in c3_pass(NEW, c, 2, y_of=lambda x: -19.) if xbi(q) and xbi(q)['own'] == 1.0]; outs = [xbi(q) for q in c3_pass(NEW, c, 2, y_of=lambda x: +4.) if xbi(q) and xbi(q)['own'] == 1.0]
    ok(all(q['y_us'] < -20. for q in ins[3:]) and all(q['y_us'] > 20. for q in outs[3:]), 'position term sign (inside -> less right, outside -> more right)')
    ok(all(abs(q['y_us']) <= X['y_lim_us'] + 0.06 for q in ins + outs), 'position term limit')
    none = [xbi(q) for q in c3_pass(NEW, c, 2, y_of=None) if xbi(q)]; ok(all(q['y_us'] == 0. and q['y_cm'] is None for q in none), 'no arc_lat value: term must be 0')
    noal = [xbi(q) for q in c3_pass(NEW, c, 2, al=False) if xbi(q)]; ok(all(q['y_us'] == 0. for q in noal), 'no arc_lat key: term must be 0')
    held = c3_pass(NEW, c, 2, y_of=lambda x: -19. if x < 2.0 else None); t2 = [q['t'] for q in held if q['x'] >= 2.0][0]
    ok(all(xbi(q)['y_us'] < -20. for q in held if xbi(q) and 2.03 < q['x'] and q['t'] - t2 < 0.5) and all(xbi(q)['y_us'] == 0. for q in held if xbi(q) and q['t'] - t2 > 0.7), 'hold of the last arc_lat value (0.6 s)')
    # rate limit
    jump = c3_pass(NEW, c, 5, near_of=lambda x: -0.3 if int(x / 0.3) % 2 else 0.9 if x < 3.9 else 0.3, y_of=lambda x: -8.)
    T = [(q['t'], xbi(q)['target_us'], xbi(q)['raw_us']) for q in jump if xbi(q)]
    ok(max(abs(b[1] - a[1]) for a, b in zip(T, T[1:]) if b[0] - a[0] < 0.05) <= X['rate_us_s'] / 30. + 0.2, 'rate limit exceeded')
    ok(max(abs(b[2] - a[2]) for a, b in zip(T, T[1:])) > 60., 'the test input did not jump')
    # cap / floor
    ok(all(1475. - X['floor_below_us'] - 0.06 <= t_[1] <= 1475. + X['cap_rel_us'] + 0.06 for t_ in T), 'cap / floor')


def test_scope():
    c = J(ON / 'step2_own_standard' / 'arc2.json'); off = copy.deepcopy(c); del off['c3']['xband']
    for seed in range(3):
        a = c3_pass(NEW, off, seed, y_of=lambda x: -8.); b = c3_pass(NEW, c, seed, y_of=lambda x: -8.)
        ok(all(p['out'] == q['out'] for p, q in zip(a, b) if q['x'] < 0.35 or q['x'] > 4.8), 'command changed outside C3s+0.35..4.8 m')
        ok(all(xbi(q) is None for q in b if q['x'] < 0.35 or q['x'] > 4.4), 'band info outside its range')
        ok(any(abs(p['out'] - q['out']) > 5. for p, q in zip(a, b) if 1.0 < q['x'] < 4.0), 'band changed nothing')
        s = c3_pass(NEW, J(ON / 'step0_shadow' / 'arc2.json'), seed, y_of=lambda x: -8.)
        ok(all(p['out'] == q['out'] for p, q in zip(a, s)) and sum(1 for q in s if xbi(q)) > 60, 'shadow changed the command / logged nothing')
    # escape: near error 0.92 -> 1.00 hands back to the chain
    e = c3_pass(NEW, c, 0, near_of=lambda x: 1.0 if 2.0 < x < 2.5 else 0.3, pid_of=lambda x: 1700., y_of=lambda x: -8.)
    ok(all(xbi(q)['own'] == 0. and q['out'] == 1700. for q in e if xbi(q) and 2.05 < q['x'] < 2.45), 'escape at error 1.0')
    m = c3_pass(NEW, c, 0, near_of=lambda x: 0.9, pid_of=lambda x: 1700., y_of=lambda x: -8.); ok(all(xbi(q)['own'] == 1.0 for q in m if xbi(q) and 1.0 < q['x'] < 4.0), 'error 0.90 must not release')
    # a broken r2 table switches the whole band off
    for k, bad in (('u_x', [[0.6, 1.0]]), ('h_x', 'x'), ('k_h_us', 'a'), ('rate_us_s', -1.)):
        cb = copy.deepcopy(c); cb['c3']['xband'][k] = bad; F = c3_pass(NEW, cb, 0, x_end=0.5)[-1]['F']; ok(F.xb is None and F.xb_on is False, 'broken %s did not disable the band' % k)
    # lost anchor heading history: heading term off, the rest works
    F = NEW.LapFF4(NEW.load_block(c['c3'], NEW.load_config(CAR_LFF)), 'c3'); A = dict(d_land=9.0)
    r = F._xb2(A, dict(odo_m=10., gz_imu=0., yaw_at='x'), 1.5, 0.4, True, 1.0, 0.16, 1500., 1475., 2050., 7.0, 40.); ok(r['info']['h_us'] == 0. and r['info']['h_deg'] is None, 'heading term without history')


if __name__ == '__main__':
    T = (test_hashes, test_patches, test_key_absent, test_configs, test_law, test_x_dr, test_terms, test_scope)
    for t in T:
        n0 = N; r = t(); print('%-18s OK  %4d checks%s' % (t.__name__, N - n0, ('  (%d frames compared)' % r) if r else ''))
    print('ALL %d tests, %d checks passed' % (len(T), N))
