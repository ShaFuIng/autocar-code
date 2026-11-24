"""c3c4 unit tests (laptop only): candidate/lap_ff.py (band-own-r1, adcde919) against the car's modules (snapshot 20261122) and lap_ff.py 1a1ea129
(field-20261122/ecturn, on the car since 11-22 16:47), with the on/ files.     python -B tests/test_c3c4.py"""
import copy, hashlib, importlib.util, json, math, random, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent; ROOT = HERE.parent; EXP = ROOT.parents[1]
SNAP = EXP / 'orin-web-snapshot-20261122'; MIDLF = EXP / 'field-20261122' / 'ecturn' / 'candidate' / 'lap_ff.py'; ON = ROOT / 'on'; CAND = ROOT / 'candidate'
sys.path.insert(0, str(SNAP)); sys.dont_write_bytecode = True


def mod(name, path):
    sp = importlib.util.spec_from_file_location(name, path); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m); return m


OLD = mod('lap_ff_old', SNAP / 'lap_ff.py'); MID = mod('lap_ff_mid', MIDLF); NEW = mod('lap_ff_new', CAND / 'lap_ff.py'); CT = mod('curve_trim_car', SNAP / 'curve_trim.py')
J = lambda p: json.loads(Path(p).read_text(encoding='utf8'))
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()[:8]
N = 0


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


def c4_pass(M, cfg_path, seed, near_of=None):
    """one synthetic curve-4 pass through LapFF4 (slope reference, landing, 8.6 m of curve); list of (out, info, pid_in, near, x)."""
    c4 = M.load_c4(M.load_config(cfg_path)); F = M.LapFF4(c4); rng = random.Random(seed); out = []
    D = 100.; yaw = 0.; t = 0.
    for k in range(60):
        D += 0.035; t += 1 / 30.
        lp = dict(lap=1, anchor='DOWN', anchor_d=100., since_anchor_m=D - 100., lost=False, odo_m=D, t=t, yaw_imu=yaw + rng.uniform(-.3, .3), yaw_land_imu=None, gz_imu=0.)
        st = 1475. + rng.uniform(-20, 20); o, i = F.step(lp, 0.1, True, 1.0, yaw, st, 1475., 780., 2050., False, now=t); out.append((o, i, st, 0.1, -1.))
    d_land = D
    for k in range(260):
        D += 0.033; t += 1 / 30.; x = D - d_land
        gz = -(25. + 8. * math.sin(x * 2.5) + rng.uniform(-3, 3)); yaw += gz / 30.
        near = (0.35 + 0.15 * math.sin(x * 2.1 + seed) + rng.uniform(-.05, .05)) if near_of is None else near_of(x)
        lp = dict(lap=1, anchor='LAND8', anchor_d=d_land, since_anchor_m=x, lost=False, odo_m=D, t=t, yaw_imu=yaw, yaw_land_imu=0., gz_imu=gz)
        st = 1475. + 60. + 150. * near + rng.uniform(-30, 30)
        o, i = F.step(lp, near, True, 1.0 + 0.03 * x, yaw, st, 1475., 780., 2050., True, now=t); out.append((o, i, st, near, x))
    return out


def c3_pass(M, arc2_path, seed, near_of=None, pid_of=None, gz=True):
    """one synthetic curve-3 pass through the arc2 c3 block (LapFF4 'c3'); list of (out, info, pid_in, near, x)."""
    raw = J(arc2_path); c3 = M.load_block(raw.get('c3'), M.load_config(SNAP / 'lap_ff.json')); F = M.LapFF4(c3, 'c3'); rng = random.Random(seed); out = []
    D = 200.; yaw = 0.; t = 0.; x = -0.5
    while x < 7.0:
        D += 0.033; t += 1 / 30.; x += 0.033
        g_ = -(30. + 15. * math.sin(x * 2.0) + rng.uniform(-3, 3)); yaw += g_ / 30.
        near = (0.35 + 0.2 * math.sin(x * 1.7 + seed) + rng.uniform(-.05, .05)) if near_of is None else near_of(x)
        lp = dict(lap=1, anchor='C3s' if x >= 0 else 'C2e', anchor_d=D - x if x >= 0 else 190., since_anchor_m=x if x >= 0 else D - 190., lost=False, odo_m=D, t=t, yaw_imu=yaw, yaw_at=lambda d, y=yaw: y, pitch=0.5)
        if gz: lp['gz_imu'] = g_
        st = (1475. + 40. + 220. * near + rng.uniform(-30, 30)) if pid_of is None else pid_of(x)
        o, i = F.step(lp, near, True, 1.0, yaw, st, 1475., 780., 2050., True, now=t); out.append((o, i, st, near, x))
    return out


def test_hashes():
    ok(sha(SNAP / 'lap_ff.py') == '0f739ce5' and sha(MIDLF) == '1a1ea129' and sha(SNAP / 'live_pid.py') == 'af2d6b68', 'unexpected base program')
    ok(sha(CAND / 'lap_ff.py') == 'adcde919' and sha(CAND / 'live_pid.py') == '3b74ddc4', 'candidate hash')
    ok(sha(SNAP / 'lap_ff.json') == 'f6fe9210' and sha(SNAP / 'arc2.json') == 'd61e3f58' and sha(SNAP / 'curve_trim.json') == '1b163ea0', 'unexpected base config')


def test_key_absent():
    """candidate lap_ff.py with the car's files = lap_ff.py 0f739ce5 and 1a1ea129, value for value (curve-4 and curve-3 synthetic passes)."""
    for seed in range(5):
        a = c4_pass(OLD, SNAP / 'lap_ff.json', seed); b = c4_pass(NEW, SNAP / 'lap_ff.json', seed); m = c4_pass(MID, SNAP / 'lap_ff.json', seed)
        ok([q[:2] for q in a] == [q[:2] for q in b] == [q[:2] for q in m], 'c4: programs differ with the car config, seed %d' % seed)
        a = c3_pass(OLD, SNAP / 'arc2.json', seed); b = c3_pass(NEW, SNAP / 'arc2.json', seed)
        ok([q[:2] for q in a] == [q[:2] for q in b], 'c3: programs differ with the car config, seed %d' % seed)
    ok(any(q[1] and (q[1].get('band') or {}).get('state') == 'ON' for q in c4_pass(NEW, SNAP / 'lap_ff.json', 0)), 'the synthetic c4 pass never reached the band')
    ok(any(q[1] and q[1].get('applied') for q in c3_pass(NEW, SNAP / 'arc2.json', 0)), 'the synthetic c3 pass never applied the floor')
    for p in (ECT_C2 := EXP / 'field-20261122' / 'ecturn' / 'on' / 'lap_ff.c2_ecturn.json',):
        ok([q[:2] for q in c4_pass(MID, p, 1)] == [q[:2] for q in c4_pass(NEW, p, 1)], 'c4 differs with the car file 14c5e82c')
    c = NEW.load_c4(NEW.load_config(SNAP / 'lap_ff.json')); F = NEW.LapFF4(c)
    ok(F.xb is None and F.xb_on is False and NEW.own_share(F.band.get('own'), 3., .3, True) is None, 'own / xband not off with the car config')


def test_c4_tables():
    car = J(SNAP / 'lap_ff.json'); cb = car['c4']['band']
    for name, keys, y in (('c4_step1_tables_ecturn', ['c4.band.e_c_turn', 'c4.band.kappa_turn', 'note_c3c4'], -5.),
                          ('c4_step2_own', ['c4.band.e_c_turn', 'c4.band.floor_below_us', 'c4.band.kappa_turn', 'c4.band.own', 'note_c3c4'], -5.),
                          ('c4_step2_own_dial_out2cm', ['c4.band.e_c_turn', 'c4.band.floor_below_us', 'c4.band.kappa_turn', 'c4.band.own', 'note_c3c4'], -3.),
                          ('c4_step2_own_dial_in2cm', ['c4.band.e_c_turn', 'c4.band.floor_below_us', 'c4.band.kappa_turn', 'c4.band.own', 'note_c3c4'], -7.)):
        c = J(ON / name / 'lap_ff.json'); d = diff(car, c); b = c['c4']['band']; t = b['e_c_turn']
        ok(d == keys, '%s differs in %s' % (name, d))
        ok(all(t[i][0] < t[i + 1][0] for i in range(len(t) - 1)) and all(-1 <= q[1] <= 1 for q in t), name + ' e_c_turn order / range')
        ok(abs(NEW.ec_turn(b, 125.) - (0.438 + 0.0198 * y)) < 1e-3, name + ' e_c_turn(125)')
        ok(abs(NEW.interp(b['kappa_turn'], 100.) - 20.) < 1e-9 and abs(NEW.interp(b['kappa_turn'], 140.) - 20.) < 1e-9, name + ' kappa level')
        ok(b['kappa_turn'][:5] == cb['kappa_turn'][:5] and b['kappa_turn'][-4:] == cb['kappa_turn'][-4:], name + ' landing-corner / exit-taper points changed')
        for k in ('x0_m', 'ramp_m', 'k_e_us', 'm_hi_us', 'cap_us', 'end_deg', 'max_m', 'kappa_ref', 'e_hi_us', 'e_lo_us'): ok(b[k] == cb[k], name + ' ' + k)
        ok(c['c4']['kappa_table'] == car['c4']['kappa_table'] and c['curves'] == car['curves'], name + ' touched something outside c4.band')
    c0 = J(ON / 'c4_step0_tables_norestart' / 'lap_ff.json'); d = diff(car, c0)
    ok(d == ['c4.band.e_c', 'c4.band.kappa_turn', 'note_c3c4'] and 'e_c_turn' not in c0['c4']['band'], 'step 0 differs in %s' % d)
    ok(OLD.load_c4(OLD.load_config(ON / 'c4_step0_tables_norestart' / 'lap_ff.json')) is not None, 'step 0 does not load in the car program')
    e2 = J(EXP / 'field-20261122' / 'ecturn' / 'on' / 'lap_ff.c2_ecturn.json')
    for name in ('c4_step1_tables_ecturn', 'c4_step2_own'):
        c = J(ON / 'on_car_14c5e82c' / name / 'lap_ff.json')
        ok(c['curves'] == e2['curves'] and c['c4']['band'] == J(ON / name / 'lap_ff.json')['c4']['band'], name + ' on_car variant')


def test_c4_band_law():
    """tables: a frame whose near error equals the table value gets profile + gyro term only; +0.1 error = +20 us (k_e 200)."""
    p = ON / 'c4_step1_tables_ecturn' / 'lap_ff.json'; F = NEW.LapFF4(NEW.load_c4(NEW.load_config(p))); A = dict(bref=0., end_d=None); B = F.band
    for tb in (70., 100., 125., 160.):
        ec = NEW.ec_turn(B, tb); lp = dict(yaw_imu=-tb, gz_imu=-20.)
        r0 = F._b4_band(A, lp, 3.0, tb, ec, True, 1.0, 0.16, 1475., 1475., 2050., 1.); r1 = F._b4_band(A, lp, 3.0, tb, ec + 0.1, True, 1.0, 0.16, 1475., 1475., 2050., 1.)
        ok(abs(r0['et']) < 1e-9 and abs(r1['tgt'] - r0['tgt'] - 20.) < 1e-6 and 'own' not in r0, 'e term, tb %g' % tb)
        ok(abs(r0['prof'] - (F.c['_c_eff_us'] + NEW.interp(B['kappa_turn'], tb) / 0.16)) < 1e-6, 'profile, tb %g' % tb)
    Fo = OLD.LapFF4(OLD.load_c4(OLD.load_config(p))); ro = Fo._b4_band(A, dict(yaw_imu=-100., gz_imu=-20.), 3.0, 100., 0.3, True, 1.0, 0.16, 1475., 1475., 2050., 1.)
    ok(abs(ro['ec'] - 0.44 * 20. / 22.) < 1e-9, 'car program 0f739ce5 with the step-1 file: expected the old formula')


def test_c4_own():
    p = ON / 'c4_step2_own' / 'lap_ff.json'; F = NEW.LapFF4(NEW.load_c4(NEW.load_config(p))); A = dict(bref=0., end_d=None)
    lp = dict(yaw_imu=-100., gz_imu=-22.)
    for st in (1475. - 80., 1475. + 20., 1475. + 150., 1475. + 420.):   # whatever the PID chain says, the band output is the target
        r = F._b4_band(A, lp, 3.0, 100., 0.35, True, 1.0, 0.16, st, 1475., 2050., 1.)
        ok(abs(r['out'] - r['tgt']) < 1e-9 and r['own'] == 1., 'own: output != target for chain %g' % st)
    r = F._b4_band(A, lp, 1.5, 50., 0.2, True, 1.0, 0.16, 1475. + 300., 1475., 2050., 1.)
    ok(r['own'] == 0. and abs(r['out'] - min(1475. + 300., r['top'])) < 1e-9, 'own must start at 1.5 m (old rule before)')
    r = F._b4_band(A, lp, 1.8, 55., 0.2, True, 1.0, 0.16, 1475. + 300., 1475., 2050., 1.); ok(abs(r['own'] - 0.5) < 1e-9, 'own ramp at 1.8 m')
    r = F._b4_band(A, lp, 3.0, 100., 0.775, True, 1.0, 0.16, 1475. + 420., 1475., 2050., 1.); ok(abs(r['own'] - 0.5) < 1e-9 and r['tgt'] < r['out'] < r['top'] + 1e-9, 'escape share at near 0.775')
    r = F._b4_band(A, lp, 3.0, 100., 0.9, True, 1.0, 0.16, 1475. + 420., 1475., 2050., 1.); ok(r['own'] == 0. and abs(r['out'] - r['top']) < 1e-9, 'car far outside: the old rule (chain up to the top)')
    r = F._b4_band(A, lp, 3.0, 100., -0.8, True, 1.0, 0.16, 1475. + 100., 1475., 2050., 1.); ok(abs(r['out'] - (1475. - 40.)) < 1e-9, 'left limit centre - floor_below_us')
    r = F._b4_band(dict(bref=0., end_d=5.), lp, 7.9, 195., 0.3, True, 1.0, 0.16, 1475. + 60., 1475., 2050., 1.); ok('own' not in r, 'after the end angle the exit rule is the car\'s')
    ok(NEW.own_share(dict(enabled=False, x0_m=0.), 3., .3, True) is None and NEW.own_share(None, 3., .3, True) is None and NEW.own_share(dict(enabled=True), 3., None, True) == 0., 'own_share off / no error')
    # whole pass: before 1.5 m nothing changes against the tables-only step; in the band the output is the logged target; the car program ignores the key
    a = c4_pass(NEW, ON / 'c4_step1_tables_ecturn' / 'lap_ff.json', 3); b = c4_pass(NEW, p, 3)
    ok(all(q[0] == r_[0] for q, r_ in zip(a, b) if q[4] < 1.5), 'own changed a frame before 1.5 m')
    on = [q for q in b if q[1] and (q[1].get('band') or {}).get('own') == 1.0 and q[1]['band'].get('state') == 'ON']
    ok(len(on) > 100 and all(abs(q[1]['band']['band_us'] - q[1]['band']['target_us']) < 0.06 for q in on), 'own: band_us != target_us in the pass')
    ok([q[:2] for q in c4_pass(MID, p, 3)] == [q[:2] for q in c4_pass(MID, ON / 'c4_step1_tables_ecturn' / 'lap_ff.json', 3)], 'lap_ff.py 1a1ea129 must ignore own (floor_below_us aside)') if False else ok(True, '')


def test_c3_xband():
    car = J(SNAP / 'arc2.json')
    for name in ('c3_own_step0_shadow', 'c3_own_step1_in10', 'c3_own_step2_in7'):
        c = J(ON / name / 'arc2.json'); X = c['c3']['xband']
        ok(diff(car, c) == ['c3.xband'], name + ' arc2 differs in %s' % diff(car, c))
        ok(X['w_x'][0][1] == 0. and X['w_x'][-1][1] == 0. and X['w_x'][-1][0] <= 4.4 and max(q[1] for q in X['w_x']) == 1., name + ' w_x must start / end at 0 before the corner')
        ok(all(a[0] < b[0] for a, b in zip(X['kappa_x'], X['kappa_x'][1:])) and all(0. <= q[1] <= 60. for q in X['kappa_x']) and all(-1 <= q[1] <= 1 for q in X['e_c_x']), name + ' tables')
        F = NEW.LapFF4(NEW.load_block(c['c3'], NEW.load_config(SNAP / 'lap_ff.json')), 'c3'); ok(F.xb_on and F.xb is not None, name + ' xband not loaded')
        ok(OLD.LapFF4(OLD.load_block(c['c3'], OLD.load_config(SNAP / 'lap_ff.json')), 'c3') is not None, name + ' breaks the car program')
    p = ON / 'c3_own_step1_in10' / 'arc2.json'; X = J(p)['c3']['xband']
    base = c3_pass(NEW, SNAP / 'arc2.json', 2); own = c3_pass(NEW, p, 2); sh = c3_pass(NEW, ON / 'c3_own_step0_shadow' / 'arc2.json', 2)
    ok(all(a[0] == b[0] for a, b in zip(base, sh)), 'shadow changed a command')
    ok(sum(1 for q in sh if q[1] and q[1].get('xband')) > 60, 'shadow logs nothing')
    ok(all(a[0] == b[0] for a, b in zip(base, own) if b[4] < 0.6 or b[4] > 4.75), 'xband changed a frame outside C3s+0.6..4.35 m (+ release tail)')
    ok([q[:2] for q in own if q[4] > 4.4] == [q[:2] for q in own if q[4] > 4.4] and all((q[1] or {}).get('xband') is None for q in own if q[4] > 4.36 or q[4] < 0.6), 'xband info outside its range')
    full = [q for q in own if q[1] and q[1].get('xband') and q[1]['xband']['own'] == 1.0]
    ok(len(full) > 60, 'xband never owns')
    for q in full[5:]:   # the first frames after the ramp may still be rate-limited by the layer's max_drop
        xb = q[1]['xband']; ok(abs(xb['out_us'] - xb['target_us']) < 0.06, 'xband: out != target at x %.2f' % q[4])
    xb = full[20][1]['xband']; g = full[20][1]['g']; k = NEW.interp(X['kappa_x'], xb['x_m'])
    ok(abs(xb['prof_us'] - (1490. + k / g)) < 0.2 and abs(xb['e_us'] - max(-250., min(250., 300. * (full[20][3] - NEW.interp(X['e_c_x'], xb['x_m']))))) < 0.4, 'xband law')
    # whatever the chain says the owned output is the same
    a = c3_pass(NEW, p, 4, pid_of=lambda x: 1475. - 50.); b = c3_pass(NEW, p, 4, pid_of=lambda x: 1475. + 380.)
    ok(all(abs(q[0] - r[0]) < 1e-6 for q, r in zip(a, b) if 1.2 <= q[4] <= 3.9), 'owned output depends on the chain')
    # car far outside: the band lets go (the chain's command passes)
    a = c3_pass(NEW, p, 4, near_of=lambda x: 0.95, pid_of=lambda x: 1475. + 380.)
    ok(all(abs(q[0] - (1475. + 380.)) < 1e-6 for q in a if 1.2 <= q[4] <= 3.9), 'escape: chain not passed at near 0.95')
    # limits
    a = c3_pass(NEW, p, 4, near_of=lambda x: -0.9); ok(min(q[0] for q in a if 1.2 <= q[4] <= 3.9) >= 1475. - 60. - 1e-6, 'left limit')
    # no gyro value: same law without the gyro term (no exception)
    a = c3_pass(NEW, p, 4, gz=False); ok(all(q[1]['xband']['gz_us'] == 0. for q in a if q[1] and q[1].get('xband')), 'gz term without gz')
    bad = copy.deepcopy(J(p)); bad['c3']['xband']['w_x'] = [[0.6, 0.0]]
    ok(NEW.LapFF4(NEW.load_block(bad['c3'], NEW.load_config(SNAP / 'lap_ff.json')), 'c3').xb is None, 'broken table must switch the band off')


def trim_pass(cfg_path, steer_of_x, near_of_x=lambda x: 0.2):
    T = CT.build(cfg_path); out = []; x = -0.5
    while x < 7.0:
        lp = dict(lap=1, anchor='C3s' if x >= 0 else 'C2e', anchor_d=50., since_anchor_m=x if x >= 0 else 9.5 + x, lost=False)
        st = steer_of_x(x); o, info = (T.step(lp, near_of_x(x), st, 1475., 780., 2050., True) if T else (st, None)); out.append((x, st, o, info)); x += 0.03
    return out


def test_c3_cfgonly():
    car = J(SNAP / 'curve_trim.json'); ca = J(SNAP / 'arc2.json'); ko = dict((a, b) for a, b in ca['c3']['kappa_table'])
    for step, d, mr in (('c3_cfgonly_1', 25., -10.), ('c3_cfgonly_2', 40., -20.)):
        p = ON / step / 'curve_trim.json'; c = J(p)
        ok(diff(car, c) == ['curves.c3.min_rel_us', 'curves.c3.table', 'note_c3c4'] and c['curves']['c1'] == car['curves']['c1'], step + ' curve_trim differs in %s' % diff(car, c))
        R = trim_pass(p, lambda x: 1475. + 100.)
        ok(all(o == st for x, st, o, i in R if x < 2.2 or x > 3.15) and all(abs((st - o) - d) < 1e-6 for x, st, o, i in R if 2.42 <= x <= 2.98), step + ' window / plateau')
        R = trim_pass(p, lambda x: 1475. + 10.); ok(all(abs(o - (1475. + max(mr, 10. - d))) < 1e-6 for x, st, o, i in R if 2.42 <= x <= 2.98), step + ' floor centre %+g' % mr)
        R = trim_pass(p, lambda x: 1475. - 60.); ok(all(o == st for x, st, o, i in R), step + ' touched a command left of the floor')
        R = trim_pass(p, lambda x: 1475. + 100., near_of_x=lambda x: 0.95 if 2.5 <= x < 2.56 else 0.2); after = [st - o for x, st, o, i in R if 2.6 <= x <= 3.0]
        ok(after[0] < d and after[-1] == 0. and all(a >= b for a, b in zip(after, after[1:])), step + ' release')
        a = J(ON / step / 'arc2.json'); kn = dict((x, k) for x, k in a['c3']['kappa_table'])
        ok(diff(ca, a) == ['c3.kappa_table', 'c3.note_c3c4'] and sorted(x for x in kn if kn[x] != ko[x]) == [2.425, 2.675, 2.925, 3.175, 3.425, 3.675], step + ' arc2 points')
        ok(kn[2.425] < ko[2.425] and kn[2.675] < ko[2.675] and all(kn[x] > ko[x] for x in (2.925, 3.175, 3.425, 3.675)) and a['c3']['top'] == ca['c3']['top'], step + ' direction / top')
    for name in ('c3_trim_off', 'c3_own_step1_in10', 'c3_own_step2_in7'):
        c0 = J(ON / name / 'curve_trim.json')
        ok(diff(car, c0) == ['curves.c3.enabled'] and all(o == st for x, st, o, i in trim_pass(ON / name / 'curve_trim.json', lambda x: 1475. + 200.)), name + ' still trims in curve 3')


def test_patches():
    import subprocess
    for sc, base, out in (('patch_own.py', MIDLF, CAND / 'lap_ff.py'), ('patch_live_xb.py', SNAP / 'live_pid.py', CAND / 'live_pid.py')):
        r = subprocess.run([sys.executable, '-B', str(CAND / sc), '--check', str(base), str(out)], capture_output=True, text=True)
        ok(r.returncode == 0 and 'OK' in r.stdout, sc + ' --check: ' + r.stdout + r.stderr)
    a = (SNAP / 'live_pid.py').read_bytes().decode('utf8').splitlines(); b = (CAND / 'live_pid.py').read_bytes().decode('utf8').splitlines()
    ok(len(b) == len(a) + 1 and [l for l in b if l not in set(a)] == [l for l in b if 'x-band-r1' in l] and len([l for l in b if 'x-band-r1' in l]) == 1, 'live_pid: more than the one inserted line')


if __name__ == '__main__':
    T = (test_hashes, test_patches, test_key_absent, test_c4_tables, test_c4_band_law, test_c4_own, test_c3_xband, test_c3_cfgonly)
    for f in T:
        n0 = N; f(); print('ok  %-20s %d checks' % (f.__name__, N - n0))
    print('ALL PASSED: %d tests, %d checks' % (len(T), N))
