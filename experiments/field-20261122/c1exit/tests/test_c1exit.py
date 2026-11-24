"""c1exit unit tests (laptop only).  python -B test_c1exit.py
old = base/lap_ff.car_adcde919.py (the car), new = candidate/lap_ff.py (patch_c1exit.py). Both need rate_indep.py of the snapshot on sys.path."""
import sys, os, json, copy, math, random, importlib.util, hashlib
from pathlib import Path
HERE = Path(__file__).resolve().parent; ROOT = HERE.parent
EXP = Path(os.environ.get('C1X_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
sys.path.insert(0, str(EXP / 'orin-web-snapshot-20261122'))


def load(path, name):
    sp = importlib.util.spec_from_file_location(name, str(path)); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m); return m


OLD = load(ROOT / 'base' / 'lap_ff.car_adcde919.py', 'lap_ff_old'); NEW = load(ROOT / 'candidate' / 'lap_ff.py', 'lap_ff_new')
CAR = ROOT / 'base' / 'lap_ff.car_052be3dc.json'
N = 0


def ok(c, msg):
    global N
    N += 1
    if not c: raise AssertionError(msg)


def stream(seed, lap=2, n=420, anchor='C4e', gap=10.74, lap0=False, over=0., e_exit=None, total=152.):
    """a synthetic curve-1 pass: (lp, far, near, v, yaw, steer) per frame. tb rises 0 -> 152 (+ over), near ~0.6 in the curve."""
    R = random.Random(seed); out = []; D0 = 300. if not lap0 else 0.; x = (gap - 2.5) if not lap0 else 0.2; tb = 0.; ds = 0.036
    for k in range(n):
        s = x - gap                                # metres past the curve start
        L = total / 45.5
        if 0. < s < L: tb = min(total + over, tb + 45. * ds)
        elif s >= L and tb < total + over: tb = min(total + over, tb + 20. * ds)
        inc = 0. < s < L - 0.04
        near = (0.6 if inc else 0.05) + R.uniform(-0.05, 0.05)
        if e_exit is not None and s >= L - 0.04: near = e_exit
        far = near + (0.75 if -0.2 < s < L - 0.14 else 0.)
        v = 1.1 + R.choice([-0.2, -0.1, 0., 0.1, 0.25])
        yaw = -tb + R.uniform(-0.3, 0.3)
        an = anchor; since = x; ad = D0; lp_lap = lap
        if lap0: an = None; since = None; ad = None; lp_lap = 0
        lp = dict(lap=lp_lap, anchor=an, anchor_d=ad, since_anchor_m=since, lost=False, odo_m=D0 + x, yaw_imu=yaw, gz_imu=(-50. if inc else 0.) + R.uniform(-5, 5))
        steer = 1475. + (200. if inc else 10.) + R.uniform(-60, 60)
        out.append((lp, far, near, v, yaw, steer)); x += ds
    return out


def drive(mod, cfg_path_or_dict, st, lost_every=0):
    c = mod.load_config(cfg_path_or_dict) if not isinstance(cfg_path_or_dict, dict) else cfg_path_or_dict
    L = mod.LapFF(c); res = []
    for k, (lp, far, near, v, yaw, steer) in enumerate(st):
        if lost_every and k % lost_every == lost_every - 1: res.append(('L', L.lost(dict(lp), v, steer, 1475., 780., 2050., k / 30.))); continue
        res.append(L.step(dict(lp), far, near, True, v, 0., yaw, steer, 1475., 780., 2050., True))
    return res


def cfg_from(path, mut=None):
    d = json.loads(Path(path).read_text(encoding='utf8'))
    if mut: mut(d)
    p = HERE / '_tmp_cfg.json'; p.write_text(json.dumps(d), encoding='utf8'); return p


def t1_patch_reproducible():
    import subprocess
    r = subprocess.run([sys.executable, str(ROOT / 'candidate' / 'patch_c1exit.py'), '--check', str(ROOT / 'base' / 'lap_ff.car_adcde919.py'), str(ROOT / 'candidate' / 'lap_ff.py')], capture_output=True, text=True)
    ok(r.returncode == 0, 'patch --check failed: ' + r.stdout + r.stderr)


def t2_key_absent_identical():
    """car json and the step-1 json (no new key): old and new modules give the same (steer, info) on every frame, also on lost frames and on a lap-0 stream."""
    for cfgp in (CAR, ROOT / 'on' / 'c1_step1_band' / 'lap_ff.json', ROOT / 'on' / 'c2_step1_tail10' / 'lap_ff.json'):
        for seed in range(6):
            for kw in (dict(), dict(lap0=True), dict(anchor='C1e', gap=11.94), dict(over=12.)):
                st = stream(seed, **kw)
                for le in (0, 7):
                    a = drive(OLD, cfgp, st, le); b = drive(NEW, cfgp, st, le)
                    ok(a == b, 'old/new differ without the new keys: %s seed %d %s' % (Path(cfgp).parent.name, seed, kw))


def t3_step1_band_owns():
    """step 1 (car program): the c1 band triggers on a flying lap, owns the output (out == band target inside the curve), not on the lap from the start."""
    r = drive(OLD, ROOT / 'on' / 'c1_step1_band' / 'lap_ff.json', stream(1))
    own = [i for s, i in r if i and i.get('curve') == 'c1' and (i.get('band') or {}).get('own') and i['state'] == 'HOLD']
    ok(len(own) > 60, 'c1 band not active (%d own frames)' % len(own))
    for i in own:
        b = i['band']; tgt = b['prof_us'] + b['e_us'] + b['gz_us']
        ok(abs(i['steer_us'] - max(1475., min(1950., tgt))) < 0.2 or i['steer_us'] >= 1475., 'own frame not at the target')
    ok(max(i['steer_us'] for i in own) < 1475. + 420., 'band output above +420')
    r0 = drive(OLD, ROOT / 'on' / 'c1_step1_band' / 'lap_ff.json', stream(1, lap0=True))
    ok(all(i is None or i.get('curve') != 'c1' for s, i in r0), 'c1 armed on lap 0 without the start key')


def t4_own_lo():
    """own_lo_us: with the car over-rotated by 12 deg and the camera asking left, the new program steers left of the centre (bounded by own_lo_us); the old one stays at the centre."""
    st = stream(2, over=12., e_exit=-0.3)
    a = drive(OLD, ROOT / 'on' / 'c1_step1_band' / 'lap_ff.json', st); b = drive(NEW, ROOT / 'on' / 'c1_step2_exit_hold' / 'lap_ff.json', st)
    amin = min(i['steer_us'] for s, i in a if i and i.get('curve') == 'c1' and i.get('applied'))
    bown = [i for s, i in b if i and i.get('curve') == 'c1' and i['state'] == 'HOLD' and (i.get('band') or {}).get('turned_deg', 0) > 158]
    ok(amin >= 1475. - 1e-6, 'old program went below the centre')
    ok(bown and min(i['steer_us'] for i in bown) < 1475. - 60., 'new program does not steer left when over-rotated')
    ok(min(i['steer_us'] for s, i in b if i and i.get('curve') == 'c1' and i['state'] == 'HOLD') >= 1475. - 120. - 1e-6, 'below own_lo_us')


def t5_v_ref():
    """v_ref: the band profile no longer follows the frame speed."""
    st = stream(3)
    a = drive(NEW, ROOT / 'on' / 'c1_step1_band' / 'lap_ff.json', st); b = drive(NEW, ROOT / 'on' / 'c1_step2_exit_hold' / 'lap_ff.json', st)
    def prof_sd(r):
        x = [i['band']['prof_us'] for s, i in r if i and i.get('band') and 50 < i['band']['turned_deg'] < 95]
        m = sum(x) / len(x); return math.sqrt(sum((q - m) ** 2 for q in x) / len(x)), m
    sa, ma = prof_sd(a); sb, mb = prof_sd(b)
    ok(sa > 8., 'expected speed noise in the profile without v_ref (sd %.1f)' % sa); ok(sb < 0.01, 'profile still varies with v_ref (sd %.2f)' % sb)
    ok(abs(mb - (1490. + 40.9 / NEW.gain(json.loads(CAR.read_text(encoding='utf8'))['gain_table'], 1.10))) < 0.5, 'profile value with v_ref')


def t6_start_anchor():
    """start: lap from the start arms on the odometer, one trigger, reference heading from the start window; a later lap is unaffected."""
    st = stream(4, lap0=True, gap=3.2)
    r = drive(NEW, ROOT / 'on' / 'c1_step3_lap1' / 'lap_ff.json', st)
    inf = [i for s, i in r if i and i.get('curve') == 'c1']
    ok(len(inf) > 60 and inf[0]['kind'] == 'FAR', 'c1 not triggered on lap 0 with the start key')
    ok(all((i.get('band') or {}).get('plat') for i in inf if i.get('band')), 'no straight reference on lap 0')
    hold = [i for i in inf if i['state'] == 'HOLD' and 40 < i['band']['turned_deg'] < 100]
    ok(hold and all(abs(i['steer_us'] - (i['band']['prof_us'] + i['band']['e_us'] + i['band']['gz_us'])) < 0.2 for i in hold), 'lap-0 band does not own the output')
    a = drive(NEW, ROOT / 'on' / 'c1_step2_exit_hold' / 'lap_ff.json', stream(4)); b = drive(NEW, ROOT / 'on' / 'c1_step3_lap1' / 'lap_ff.json', stream(4))
    ok(a == b, 'start key changed a flying lap')
    rs = drive(NEW, ROOT / 'on' / 'c1_step3_lap1' / 'lap_ff.json', stream(4, lap0=True, gap=9.0))   # the curve does not come where expected -> window missed -> never armed
    ok(all(i is None or i.get('curve') != 'c1' for s, i in rs), 'armed outside the start window')


def t7_exit_hold_ends_by_distance():
    """step 2: the band ends by max_m (4.7 m from the trigger), fades over 0.5 m and then releases the chain value."""
    st = stream(5, n=520); r = drive(NEW, ROOT / 'on' / 'c1_step2_exit_hold' / 'lap_ff.json', st)
    inf = [i for s, i in r if i and i.get('curve') == 'c1']
    ends = [i for i in inf if i.get('end')]
    ok(ends and ends[0]['end'] == 'MAX_M' and 4.65 <= ends[0]['x_m'] <= 4.8, 'end rule: %s' % (ends[0] if ends else None))
    ok(inf[-1]['state'] == 'END' and inf[-1]['x_m'] < 4.7 + 0.5 + 0.1, 'fade length')
    ok(r[-1] == (st[-1][5], None), 'chain value not passed through after the band')
    r1 = drive(OLD, ROOT / 'on' / 'c1_step1_band' / 'lap_ff.json', st); e1 = [i for s, i in r1 if i and i.get('curve') == 'c1' and i.get('end')]
    ok(e1 and e1[0]['end'] == 'ANGLE' and 145 <= e1[0]['band']['turned_deg'] <= 152, 'step 1 ends by angle')


def t8_c2_untouched_by_c1_steps():
    """the c1 steps leave the curve-2 band output unchanged; the c2 tail steps change nothing before tb 156 and < 3 us up to 178."""
    st = stream(6, anchor='C1e', gap=11.94, n=560, total=208.)
    a = drive(OLD, CAR, st); b = drive(OLD, ROOT / 'on' / 'c1_step1_band' / 'lap_ff.json', st); c = drive(NEW, ROOT / 'on' / 'c1_step3_lap1' / 'lap_ff.json', st)
    ok(a == b == c, 'c1 step changed a curve-2 pass')
    t = drive(OLD, ROOT / 'on' / 'c2_step1_tail10' / 'lap_ff.json', st)
    for (sa, ia), (sb, ib) in zip(a, t):
        if ia and ia.get('band') and ia['band']['turned_deg'] < 155.: ok(sa == sb, 'c2 tail step changed the command before tb 156')
        elif ia and ia.get('band') and ia['band']['turned_deg'] < 177.: ok(abs(sa - sb) < 3., 'c2 tail step changed the command by >= 3 us at tb 156..178 (only the e_c interpolation 156 -> 180/190 moves)')


def t9_configs_only_differ_in_listed_keys():
    base = json.loads(CAR.read_text(encoding='utf8'))
    for p in sorted((ROOT / 'on').glob('*/lap_ff.json')):
        d = json.loads(p.read_text(encoding='utf8'))
        for k in set(base) | set(d):
            if k != 'curves': ok(base.get(k) == d.get(k), '%s: top-level key %s differs' % (p.parent.name, k))
        ok(base['curves']['c3'] == d['curves']['c3'], '%s: c3 differs' % p.parent.name)
        if 'c2' not in p.parent.name: ok(base['curves']['c2'] == d['curves']['c2'], '%s: c2 differs' % p.parent.name)
        if 'c1' not in p.parent.name: ok(base['curves']['c1'] == d['curves']['c1'], '%s: c1 differs' % p.parent.name)
        ok(base.get('c4') == d.get('c4'), '%s: c4 differs' % p.parent.name)
    ib = json.loads((EXP / 'orin-web-snapshot-20261122' / 'i_ramp.json').read_text(encoding='utf8')); i2 = json.loads((ROOT / 'on' / 'i_ramp_c2e' / 'i_ramp.json').read_text(encoding='utf8'))
    x = copy.deepcopy(i2); x['exit'].pop('C2e'); ok(x == ib, 'i_ramp differs outside exit.C2e')
    sys.path.insert(0, str(EXP / 'orin-web-snapshot-20261122')); import i_ramp
    R = i_ramp.load(ROOT / 'on' / 'i_ramp_c2e' / 'i_ramp.json'); ok(R is not None and set(R.exits) == {'C1e', 'C2e', 'C4e'}, 'i_ramp exits %s' % (None if R is None else sorted(R.exits)))


if __name__ == '__main__':
    for f in (t1_patch_reproducible, t2_key_absent_identical, t3_step1_band_owns, t4_own_lo, t5_v_ref, t6_start_anchor, t7_exit_hold_ends_by_distance, t8_c2_untouched_by_c1_steps, t9_configs_only_differ_in_listed_keys):
        n0 = N; f(); print('ok  %-40s %d checks' % (f.__name__, N - n0))
    try: (HERE / '_tmp_cfg.json').unlink()
    except OSError: pass
    print('ALL PASSED: %d tests, %d checks' % (9, N))
