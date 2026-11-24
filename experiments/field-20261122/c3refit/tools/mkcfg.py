"""writes ../on/<step>/arc2.json: the car's arc2.json (2b81e9b2 = c3c4/on/c3_own_step1_in10) with ONLY c3.xband replaced by the c3refit x-band-r2 block.
Also copies the two roll-back files (f4573960 = x-band r1 shadow, d61e3f58 = no x-band) and the curve_trim.json that must stay (40078635, curve-3 trim off).
usage: python mkcfg.py"""
import copy, hashlib, json, shutil
from pathlib import Path
import plan

HERE = Path(__file__).resolve().parent; ON = HERE.parent / 'on'; EXP = HERE.parents[2]
C3C4 = EXP / 'field-20261122' / 'c3c4' / 'on'
CAR = C3C4 / 'c3_own_step1_in10' / 'arc2.json'
h8 = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()[:8]
assert h8(CAR) == '2b81e9b2', h8(CAR)
base = json.loads(CAR.read_text(encoding='utf8'))
NOTE = ("c3refit-20261122 %s: distance band x-band-r2 (needs lap_ff.py 6b3632f1 + live_pid.py b3267293). Smooth path through C3s+0.6..4.4 m (tools/plan.py: %s); "
        "command = u_x (us, feed-forward) + k_h*(h_x - heading since C3s) + k_y*(arc_lat y - y_x) + k_e*(near - e_c_x) + gyro-rate term, rate limited; distance = odometer + speed*time since its last step (x_dr). "
        "Owns 0.9..4.05 m (ramps 0.6-0.9 / 4.05-4.35 m); corner, ramp foot and c3 top stay with the present control. Share falls to 0 between near error 0.92 and 1.00. "
        "Without the r2 keys = x-band-r1; delete the xband key = no band.")
STEPS = dict(step0_shadow=('standard', True), step1_own_conservative=('conservative', False), step2_own_standard=('standard', False), step3_own_prerotated=('prerotated', False))
for step, (var, shadow) in STEPS.items():
    pk, bk = plan.VARIANTS[var]; xb = plan.xband(plan.plan(**pk), shadow=shadow, **bk)
    xb['note'] = NOTE % (step, ', '.join('%s %s' % kv for kv in pk.items()))
    d = copy.deepcopy(base); d['c3']['xband'] = xb
    (ON / step).mkdir(parents=True, exist_ok=True); p = ON / step / 'arc2.json'; p.write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding='utf8')
    shutil.copyfile(C3C4 / 'c3_trim_off' / 'curve_trim.json', ON / step / 'curve_trim.json')
    diff = [k for k in set(d) | set(base) if d.get(k) != base.get(k)]; diff3 = [k for k in set(d['c3']) | set(base['c3']) if d['c3'].get(k) != base['c3'].get(k)]
    print('%-24s arc2.json %s | differs from the car (2b81e9b2) in top-level keys %s, inside c3 only %s | curve_trim.json %s (= the car)' % (step, h8(p), diff, diff3, h8(ON / step / 'curve_trim.json')))
for name, src in (('rollback_r1_shadow_f4573960', C3C4 / 'c3_own_step0_shadow' / 'arc2.json'), ('rollback_all_off_d61e3f58', EXP / 'orin-web-snapshot-20261122' / 'arc2.json'), ('rollback_car_now_2b81e9b2', CAR)):
    (ON / name).mkdir(parents=True, exist_ok=True); shutil.copyfile(src, ON / name / 'arc2.json'); print('%-32s arc2.json %s' % (name, h8(ON / name / 'arc2.json')))
