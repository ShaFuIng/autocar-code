"""taper test: python test_taper.py <base lap_speed.py dir> <candidate dir>  (fake lap / frame objects; no Orin modules)"""
import sys, importlib.util, types, json, random
def load(d, name):
    sys.modules.pop('throttle_cap', None)
    tc = types.ModuleType('throttle_cap'); tc.max_pct = lambda: 100.; sys.modules['throttle_cap'] = tc
    s = importlib.util.spec_from_file_location(name, d + '/lap_speed.py'); m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
class Lap: lost = False
class Sp:
    def __init__(s): s.c = {'hard_cap': 50.}; s.cur = 50.
class LP:
    def __init__(s): s._lpos = Lap(); s.run = {'parameters': {'center_us': 1490}}; s.speed = Sp()
class F: special = False; dr = {'route': 'WAIT_CLIMB'}; pr = 0.5
ZON = [dict(anchor='C4e', from_m=0.8, to_m=6.8, pwm=100, name='hi'), dict(anchor='C4e', from_m=0.8, to_m=8.0, pwm=50, name='lo')]
def run(mod, cfg, frames):
    L = mod.LapSpeed(dict(mod.DEFAULTS, **cfg)); lp = LP(); out = []
    for t, x, base, e, lvl in frames:
        lp._lpos.D = x; lp._lpos.lap = 1; lp._lpos.events = [dict(name='C4e', d_cross=0., lap=1)]
        p = dict(reason='PROPOSAL_ONLY', pwm=base, image_error=e, steer_us=1500., speed_adapt=dict(reason='STABLE', level=lvl))
        lg = L.apply(lp, F, p, t); out.append((round(p['pwm'], 3), json.dumps(lg, sort_keys=True)))
    return out
base, cand = load(sys.argv[1], 'b'), load(sys.argv[2], 'c')
random.seed(3); fr = []; t = 0.; x = 0.
for i in range(600):
    t += random.choice([0.03, 0.033, 0.04, 0.07]); x += 0.045
    fr.append((t, x % 12., random.choice([28., 32., 50., 50., 50.]), random.uniform(-0.4, 0.4), random.choice(['HIGH', 'HIGH', 'MID', 'LOW'])))
cfg = dict(enabled=True, zones=ZON)
a = run(base, cfg, fr); b = run(cand, cfg, fr); assert a == b, 'key absent must be identical'
print('key absent: %d frames identical' % len(a))
fr2 = [(0.033 * i, 6.0 + 0.047 * i, 50., 0.05, 'HIGH') for i in range(45)]
a = run(base, dict(cfg), fr2); b = run(cand, dict(cfg, down_rate_pwm_s=80.), fr2)
print('x      base  taper'); [print('%.2f  %5.1f  %5.1f' % (f[1], p[0], q[0])) for f, p, q in list(zip(fr2, a, b))[10:40:2]]
pb = [q[0] for q in b]; i0 = next(i for i, f in enumerate(fr2) if f[1] > 6.8)
assert all(pb[i] >= pb[i + 1] - 1e-9 for i in range(i0 - 1, len(pb) - 1)) and min(pb[i0:]) == 50. and max(abs(pb[i] - pb[i + 1]) for i in range(i0 - 1, len(pb) - 1)) < 4.
fr3 = list(fr2); fr3[i0 + 3] = (fr3[i0 + 3][0], fr3[i0 + 3][1], 28., 0.5, 'HIGH')   # error above the guard inside the taper -> blocked at once
c = run(cand, dict(cfg, down_rate_pwm_s=80.), fr3); assert c[i0 + 3][0] == 28. and c[i0 + 4][0] == 50., c[i0 + 3:i0 + 6]
print('monotone step-down to 50, max step < 4 PWM/frame; a blocked frame drops at once and the taper does not resume: OK')
