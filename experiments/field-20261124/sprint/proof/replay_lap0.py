"""Open-loop replay of the car's own speed modules (snapshot 20261123d: speed_adapt.py, brake_zones.py, throttle_cap.py, rate_indep.py)
on the recorded lap-0 frames of a far-start run, once with the normal files and once with the sprint files of the same start position.
Open loop: the recorded camera error / steering / pitch / odometer are fed unchanged, so the speeds do not follow the new PWM; the output
shows what the PWM chain decides (where 100 is reached, where it ends, what curve 1 gets), not what the car would do.
python replay_lap0.py <run> <back_m>     e.g. run-cbc4-20261123 6.0  (cbc4 = 6 m behind the flag, start_bias off, 11-23 10:16)"""
import importlib, json, sys, os, tempfile, shutil
from pathlib import Path
EXP = Path(__file__).resolve().parents[3]
SNAP = EXP / 'orin-web-snapshot-20261123d'
TOOLS = Path(__file__).resolve().parents[1] / 'tools'
run, back = sys.argv[1], float(sys.argv[2])


def gen(back):
    out = Path(tempfile.mkdtemp(prefix='sprint_replay_'))
    import subprocess
    r = subprocess.run([sys.executable, str(TOOLS / 'mksprint.py'), '--base', str(SNAP), '--back', str(back), '--out', str(out)], capture_output=True, text=True, encoding='utf8', env=dict(os.environ, PYTHONIOENCODING='utf8', PYTHONDONTWRITEBYTECODE='1'))
    assert r.returncode == 0, r.stderr
    return out


def modules(cfgdir):
    """the snapshot modules, with speed_adapt / brake_zones reading the given folder's json."""
    sys.path.insert(0, str(SNAP))
    import speed_adapt, brake_zones
    speed_adapt.CONFIG_FILE = cfgdir / 'speed_adapt.json'; brake_zones.CONFIG_FILE = cfgdir / 'brake_zones.json'
    return speed_adapt.SpeedAdapt(), brake_zones.BrakeZones()


def frames(run):
    with open(EXP / run / 'pid.jsonl', encoding='utf8') as f:
        for l in f:
            if not l.startswith('{"kind": "sample"'): continue
            d = json.loads(l); p = d['proposal']
            if p.get('reason') != 'PROPOSAL_ONLY' or p.get('image_error') is None: continue
            yield d, p


CURVE_KEYS = {('start', 0): 'c1', ('start', 1): 'c2', ('start', 2): 'c3', ('after_down', 0): 'c4', ('after_down', 1): 'c1', ('after_down', 2): 'c2', ('after_down', 3): 'c3'}   # = speed_policy.CURVE_KEYS


def ckey(p):
    cm = p.get('curve_map') or {}
    return CURVE_KEYS.get((cm.get('seg'), cm.get('idx'))) if cm.get('reason') in ('FLOOR', 'RELEASED') else None


def replay(cfgdir, center):
    sa, bz = modules(cfgdir); rows = []
    for d, p in frames(run):
        i = p.get('inputs') or {}; ib = p.get('imu_boost') or {}; lp = p.get('lap_pos') or {}
        if i.get('t_send') is None or i.get('enc_b') is None: continue
        pwm, info = sa.step(i['t_send'], ib.get('pwm') or 15., p['image_error'], p['steer_us'], center, False, ib.get('pitch_rel_deg'),
                            route=(d['vision'].get('dash_route') or {}).get('route'), far=(p.get('turn_preview') or {}).get('far_projected_error'), wheel=None, curve_key=ckey(p), xp=isinstance(p.get('crossing_pass'), dict) and 'dist_after_m' in p['crossing_pass'])
        veh = {'enc_b': {'forward': i['enc_b'], 'reverse': 0}}
        cap, binfo = bz.update(veh, info.get('reason'), p['image_error'], None, ib.get('pitch_rel_deg'), lap=None)
        if cap is not None and pwm > cap:
            pwm = cap
            if sa.cur > cap: sa.cur = cap   # speed_policy.write_speed_cur
        rows.append((lp.get('odo_m'), lp.get('s_m'), round(pwm, 1), info.get('level'), cap, p.get('pwm')))
        if (lp.get('odo_m') or 0) > 14: break
    return rows


tmp = gen(back)
try:
    centre = 1475 if run[4:8] in ('cbc4', '2932', 'bc08', '2fa1', 'f8d4') else 1445
    A = replay(tmp / 'normal', centre)
    for m in ('speed_adapt', 'brake_zones', 'throttle_cap', 'rate_indep'): sys.modules.pop(m, None)
    sys.path.remove(str(SNAP))
    B = replay(tmp / 'sprint', centre)
    print('run %s, back %.1f m: odo / s_m / logged PWM / replay normal (level, cap) / replay sprint (level, cap)' % (run, back))
    last = -1
    for a, b in zip(A, B):
        k = int((a[0] or 0) / 0.5)
        if k == last: continue
        last = k
        print('odo %5.2f s %6.2f | logged %5.1f | normal %5.1f %-6s cap %-5s | sprint %5.1f %-6s cap %s' % (a[0], a[1], a[5] or 0, a[2], a[3], a[4], b[2], b[3], b[4]))
finally:
    shutil.rmtree(tmp, ignore_errors=True)
